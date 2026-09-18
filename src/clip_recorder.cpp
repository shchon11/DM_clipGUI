// clip_recorder.cpp
// Dashcam-style clip recorder for ROS 2 Humble.
// Continuously buffers the last `pre_sec` seconds of serialized messages in RAM.
// On trigger (std_srvs/Trigger service, or a stamped std_msgs/Header on
// ~/trigger), keeps recording until `event + post_sec`, then writes
// [event - pre_sec, event + post_sec] to a rosbag2 bag in a background thread.
// Publishes ring-buffer statistics on /diagnostics so the real memory need
// (inflow MB/s x pre_sec) can be measured on the target system.
//
// Manual recording ("start" / "stop" on ~/record, or the ~/start_recording
// and ~/stop_recording services) writes every message of the same subscriptions
// straight into one bag until stopped (record_split_sec > 0 would split it; off
// by default — a recording is one bag from start to stop). It
// reuses the ring buffer's subscriptions instead of a second `ros2 bag record`,
// so the cameras are not asked to send everything twice, and it hands the
// writer the received buffer itself (no copy). Clips can still be cut while a
// recording runs.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <ctime>
#include <deque>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/create_timer.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

struct StampedMsg
{
  rclcpp::Time stamp;
  std::string topic;
  std::shared_ptr<rclcpp::SerializedMessage> data;
};

class ClipRecorder : public rclcpp::Node
{
public:
  ClipRecorder()
  : Node("clip_recorder")
  {
    pre_sec_ = declare_parameter<double>("pre_sec", 10.0);
    post_sec_ = declare_parameter<double>("post_sec", 5.0);
    // Extra history kept beyond pre_sec so a trigger that arrives up to this
    // late (CLI startup, discovery, network) still gets a full pre window.
    trigger_slack_sec_ = declare_parameter<double>("trigger_slack_sec", 2.0);
    max_buffer_mb_ = declare_parameter<double>("max_buffer_mb", 4096.0);
    queue_depth_ = declare_parameter<int>("queue_depth", 50);
    output_dir_ = declare_parameter<std::string>("output_dir", "clips");
    storage_id_ = declare_parameter<std::string>("storage_id", "sqlite3");
    topics_filter_ = declare_parameter<std::vector<std::string>>(
      "topics", std::vector<std::string>{});
    exclude_ = declare_parameter<std::vector<std::string>>(
      "exclude", std::vector<std::string>{"/rosout", "/parameter_events"});
    // Per-topic QoS overrides, one entry per topic:
    //   "/topic <auto|reliable|best_effort> <auto|volatile|transient_local>"
    // "auto" keeps the adapt-to-publishers behavior.
    topic_qos_raw_ = declare_parameter<std::vector<std::string>>(
      "topic_qos", std::vector<std::string>{});
    parseQosOverrides(topic_qos_raw_, qos_overrides_);
    // Manual recording: one bag from start to stop by default (record_split_sec
    // > 0 splits it every N seconds); rosbag2's cache thread does the disk
    // writes (record_cache_mb, 0 = write inline).
    record_split_sec_ = declare_parameter<double>("record_split_sec", 0.0);
    record_cache_mb_ = declare_parameter<double>("record_cache_mb", 256.0);

    max_buffer_bytes_ = static_cast<size_t>(max_buffer_mb_ * 1024.0 * 1024.0);

    // Trigger 1: plain service — event time = request arrival time.
    srv_ = create_service<std_srvs::srv::Trigger>(
      "~/trigger_clip",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = startClip(now(), "", res->message);
      });

    // Trigger 2: stamped topic. `stamp` = when the event actually happened
    // (zero = now), so a trigger that arrives late (CLI startup, discovery,
    // network) still cuts exactly [stamp - pre_sec, stamp + post_sec].
    // `frame_id` is an optional label appended to the clip directory name.
    // One publish reaches every recorder on every host.
    trigger_sub_ = create_subscription<std_msgs::msg::Header>(
      "~/trigger", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::Header::ConstSharedPtr h) {
        const bool zero = h->stamp.sec == 0 && h->stamp.nanosec == 0;
        const rclcpp::Time event =
          zero ? now() : rclcpp::Time(h->stamp, get_clock()->get_clock_type());
        std::string msg;
        if (!startClip(event, h->frame_id, msg)) {
          RCLCPP_WARN(get_logger(), "trigger ignored: %s", msg.c_str());
        }
      });

    // Manual recording: "start", "start|<label>", "stop".
    record_sub_ = create_subscription<std_msgs::msg::String>(
      "~/record", rclcpp::QoS(10).reliable(),
      [this](std_msgs::msg::String::ConstSharedPtr m) {
        std::string msg;
        const std::string & cmd = m->data;
        if (cmd.rfind("start", 0) == 0) {
          const auto bar = cmd.find('|');
          startRecording(bar == std::string::npos ? "" : cmd.substr(bar + 1), msg);
        } else if (cmd == "stop") {
          stopRecording(msg);
        } else {
          RCLCPP_WARN(get_logger(), "unknown record command '%s'", cmd.c_str());
        }
      });
    rec_start_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/start_recording",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = startRecording("", res->message);
      });
    rec_stop_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/stop_recording",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = stopRecording(res->message);
      });

    // Clip lifecycle events for GUIs: "started|<label>", "writing|<uri>",
    // "done|<uri>|<msgs>|<dur_s>", "busy|<reason>", "error|<what>".
    // Manual recording: "rec_started|<uri>", "rec_closing|<uri>",
    // "rec_stopped|<uri>|<msgs>|<dur_s>|<MB>", "rec_busy|<reason>", "rec_error|<what>".
    event_pub_ = create_publisher<std_msgs::msg::String>(
      "~/clip_event", rclcpp::QoS(10).reliable());

    // Ring-buffer statistics (log + /diagnostics). 0 disables.
    status_period_sec_ = declare_parameter<double>("status_period_sec", 5.0);
    if (status_period_sec_ > 0.0) {
      status_pub_ =
        create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
      status_timer_ = create_wall_timer(
        std::chrono::duration<double>(status_period_sec_), [this]() {publishStatus();});
    }

    discover_timer_ = create_wall_timer(2s, [this]() { discoverTopics(); });

    // Runtime reconfiguration (GUI / `ros2 param set`): the parameter service
    // and the discover timer share the node's default (mutually exclusive)
    // callback group, so these fields need no extra locking.
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == "topics") {
            topics_filter_ = p.as_string_array();
          } else if (p.get_name() == "exclude") {
            exclude_ = p.as_string_array();
          } else if (p.get_name() == "output_dir") {
            // Takes effect from the next clip / recording; one in progress keeps its folder.
            std::lock_guard<std::mutex> lk(dir_mtx_);
            output_dir_ = p.as_string();
            RCLCPP_INFO(get_logger(), "output_dir -> %s", output_dir_.c_str());
          } else if (p.get_name() == "topic_qos") {
            std::unordered_map<std::string, QosOverride> parsed;
            if (!parseQosOverrides(p.as_string_array(), parsed)) {
              result.successful = false;
              result.reason =
                "topic_qos entry must be \"/topic <auto|reliable|best_effort>"
                " <auto|volatile|transient_local>\"";
              return result;
            }
            topic_qos_raw_ = p.as_string_array();
            qos_overrides_ = std::move(parsed);
          }
        }
        return result;
      });

    RCLCPP_INFO(
      get_logger(),
      "clip_recorder up: pre=%.1fs (+%.1fs trigger slack) post=%.1fs cap=%.0fMB out=%s storage=%s",
      pre_sec_, trigger_slack_sec_, post_sec_, max_buffer_mb_, output_dir_.c_str(),
      storage_id_.c_str());
  }

  ~ClipRecorder() override
  {
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    // A recording still open at shutdown: closing flushes the cache to disk.
    {
      std::lock_guard<std::mutex> lk(rec_mtx_);
      rec_writer_.reset();
    }
    if (rec_close_thread_.joinable()) {
      rec_close_thread_.join();
    }
  }

private:
  // ---------- topic discovery / subscription ----------

  struct QosOverride
  {
    std::string reliability{"auto"};
    std::string durability{"auto"};
  };

  static bool parseQosOverrides(
    const std::vector<std::string> & raw,
    std::unordered_map<std::string, QosOverride> & out)
  {
    std::unordered_map<std::string, QosOverride> parsed;
    for (const auto & entry : raw) {
      std::istringstream iss(entry);
      std::string topic, rel, dur;
      iss >> topic >> rel >> dur;
      if (topic.empty()) {continue;}
      QosOverride ov;
      if (!rel.empty()) {
        if (rel != "auto" && rel != "reliable" && rel != "best_effort") {return false;}
        ov.reliability = rel;
      }
      if (!dur.empty()) {
        if (dur != "auto" && dur != "volatile" && dur != "transient_local") {return false;}
        ov.durability = dur;
      }
      parsed[topic] = ov;
    }
    out = std::move(parsed);
    return true;
  }

  std::string qosSignature(const std::string & topic) const
  {
    auto it = qos_overrides_.find(topic);
    if (it == qos_overrides_.end()) {return "auto auto";}
    return it->second.reliability + " " + it->second.durability;
  }

  bool wanted(const std::string & topic) const
  {
    for (const auto & ex : exclude_) {
      if (topic == ex) {return false;}
    }
    if (topics_filter_.empty()) {return true;}
    for (const auto & t : topics_filter_) {
      if (topic == t) {return true;}
    }
    return false;
  }

  void discoverTopics()
  {
    // Drop subscriptions that are no longer selected, or whose QoS override
    // changed — QoS is fixed at subscription time, so a change means
    // resubscribing.
    std::vector<std::string> drop;
    for (const auto & [topic, sub] : subs_) {
      if (!wanted(topic) || applied_qos_[topic] != qosSignature(topic)) {
        drop.push_back(topic);
      }
    }
    for (const auto & t : drop) {unsubscribeFrom(t);}

    const auto names_types = get_topic_names_and_types();
    for (const auto & [name, types] : names_types) {
      if (types.empty() || subs_.count(name) || failed_topics_.count(name) ||
        !wanted(name)) {continue;}
      subscribeTo(name, types.front());
    }
  }

  void unsubscribeFrom(const std::string & topic)
  {
    subs_.erase(topic);
    latched_topics_.erase(topic);
    applied_qos_.erase(topic);
    // types_ is kept so an in-flight clip can still write this topic.
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      latched_last_.erase(topic);
      std::deque<StampedMsg> kept;
      for (auto & m : buffer_) {
        if (m.topic == topic) {
          buffer_bytes_ -= m.data->size();
        } else {
          kept.push_back(std::move(m));
        }
      }
      buffer_.swap(kept);
    }
    RCLCPP_INFO(get_logger(), "unsubscribed: %s", topic.c_str());
  }

  void subscribeTo(const std::string & topic, const std::string & type)
  {
    QosOverride ov;
    if (auto oit = qos_overrides_.find(topic); oit != qos_overrides_.end()) {
      ov = oit->second;
    }

    // "auto": adapt QoS to the current publishers (same idea as rosbag2 record).
    const auto infos = get_publishers_info_by_topic(topic);
    bool all_reliable = !infos.empty();
    bool all_transient = !infos.empty();
    for (const auto & info : infos) {
      const auto & q = info.qos_profile();
      if (q.reliability() != rclcpp::ReliabilityPolicy::Reliable) {
        all_reliable = false;
      }
      if (q.durability() != rclcpp::DurabilityPolicy::TransientLocal) {
        all_transient = false;
      }
    }

    const bool reliable =
      ov.reliability == "auto" ? all_reliable : ov.reliability == "reliable";
    const bool transient =
      ov.durability == "auto" ? all_transient : ov.durability == "transient_local";

    rclcpp::QoS qos(rclcpp::KeepLast(static_cast<size_t>(queue_depth_)));
    reliable ? qos.reliable() : qos.best_effort();
    transient ? qos.transient_local() : qos.durability_volatile();

    // Type support for `type` may be missing on this machine (e.g. a custom
    // message published from another host). Skip such topics instead of
    // letting the exception unwind through the timer and kill the node.
    rclcpp::GenericSubscription::SharedPtr sub;
    try {
      sub = create_generic_subscription(
        topic, type, qos,
        [this, topic, type](std::shared_ptr<rclcpp::SerializedMessage> msg) {
          onMessage(topic, type, std::move(msg));
        });
    } catch (const std::exception & e) {
      failed_topics_.insert(topic);
      RCLCPP_WARN(
        get_logger(), "cannot subscribe %s [%s]: %s -- skipping this topic",
        topic.c_str(), type.c_str(), e.what());
      return;
    }

    if (transient) {
      latched_topics_.insert(topic);
    }
    subs_[topic] = sub;
    types_[topic] = type;
    applied_qos_[topic] = qosSignature(topic);
    RCLCPP_INFO(
      get_logger(), "subscribed: %s [%s] (%s%s)",
      topic.c_str(), type.c_str(),
      reliable ? "reliable" : "best_effort",
      transient ? ", transient_local" : "");
  }

  // ---------- ring buffer ----------

  void onMessage(
    const std::string & topic, const std::string & type,
    std::shared_ptr<rclcpp::SerializedMessage> msg)
  {
    const rclcpp::Time stamp = now();
    const size_t sz = msg->size();

    {
      std::lock_guard<std::mutex> rk(rec_mtx_);
      if (rec_writer_) {
        try {
          writeShared(*rec_writer_, topic, type, msg, stamp);
          ++rec_msgs_;
          rec_bytes_ += sz;
        } catch (const std::exception & e) {
          ++rec_errors_;
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 5000, "recording write failed (%s): %s",
            topic.c_str(), e.what());
        }
      }
    }

    std::lock_guard<std::mutex> lk(buf_mtx_);

    // Latched topics (e.g. /tf_static): keep the latest sample separately so
    // every clip can include it, even long after it was evicted from the ring.
    if (latched_topics_.count(topic)) {
      latched_last_[topic] = msg;
    }

    buffer_.push_back(StampedMsg{stamp, topic, msg});
    buffer_bytes_ += sz;

    if (clip_active_) {
      clip_.push_back(buffer_.back());
    }

    // Evict: older than pre_sec horizon, or over the byte cap.
    topic_bytes_[topic] += sz;
    topic_msgs_[topic] += 1;

    const rclcpp::Time horizon =
      stamp - rclcpp::Duration::from_seconds(pre_sec_ + trigger_slack_sec_);
    while (!buffer_.empty() &&
      (buffer_.front().stamp < horizon || buffer_bytes_ > max_buffer_bytes_))
    {
      if (buffer_.front().stamp >= horizon) {++cap_evictions_;}  // evicted by cap, not age
      buffer_bytes_ -= buffer_.front().data->size();
      buffer_.pop_front();
    }
  }

  // ---------- manual recording ----------

  // Hand the writer the received buffer itself: the bag message points at the
  // same bytes the ring buffer holds, and its deleter only keeps `msg` alive
  // until rosbag2's cache has written it. Writer::write(shared_ptr<rclcpp::
  // SerializedMessage>) would take the buffer away from the ring buffer instead.
  static void writeShared(
    rosbag2_cpp::Writer & writer, const std::string & topic, const std::string & type,
    const std::shared_ptr<rclcpp::SerializedMessage> & msg, const rclcpp::Time & stamp)
  {
    auto bag = std::make_shared<rosbag2_storage::SerializedBagMessage>();
    bag->topic_name = topic;
    bag->time_stamp = stamp.nanoseconds();
    bag->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
      &msg->get_rcl_serialized_message(), [msg](rcutils_uint8_array_t *) {});
    writer.write(bag, topic, type, "cdr");
  }

  bool startRecording(const std::string & label, std::string & msg_out)
  {
    std::unordered_map<std::string, std::shared_ptr<rclcpp::SerializedMessage>> latched;
    std::unordered_map<std::string, std::string> types;
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      latched = latched_last_;
      types = types_;
    }

    std::lock_guard<std::mutex> lk(rec_mtx_);
    if (rec_writer_) {
      msg_out = "already recording: " + rec_uri_;
      publishEvent("rec_busy|" + msg_out);
      return false;
    }

    std::time_t tt = std::time(nullptr);
    std::tm tm{};
    localtime_r(&tt, &tm);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
    std::string uri = outputDir() + "/rec_" + ts;
    if (!label.empty()) {uri += "_" + sanitizeLabel(label);}

    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = uri;
    storage_opts.storage_id = storage_id_;
    storage_opts.max_cache_size =
      static_cast<uint64_t>(std::max(0.0, record_cache_mb_) * 1024.0 * 1024.0);
    storage_opts.max_bagfile_duration =
      static_cast<uint64_t>(std::max(0.0, record_split_sec_));
    rosbag2_cpp::ConverterOptions conv_opts;
    conv_opts.input_serialization_format = "cdr";
    conv_opts.output_serialization_format = "cdr";

    auto writer = std::make_unique<rosbag2_cpp::Writer>();
    try {
      writer->open(storage_opts, conv_opts);
    } catch (const std::exception & e) {
      msg_out = std::string("cannot open ") + uri + ": " + e.what();
      RCLCPP_ERROR(get_logger(), "%s", msg_out.c_str());
      publishEvent("rec_error|" + msg_out);
      return false;
    }

    rec_start_ = now();
    rec_msgs_ = rec_bytes_ = rec_errors_ = 0;
    // Latched topics (/tf_static, /ouster/metadata, ...) were published once,
    // long ago — put the last sample at the start so the recording is usable.
    for (const auto & [topic, msg] : latched) {
      auto it = types.find(topic);
      if (it != types.end()) {
        writeShared(*writer, topic, it->second, msg, rec_start_);
      }
    }
    rec_writer_ = std::move(writer);
    rec_uri_ = uri;
    msg_out = "recording to " + uri;
    RCLCPP_INFO(
      get_logger(), "recording started: %s (%s, cache %.0fMB, %zu latched)", uri.c_str(),
      record_split_sec_ > 0.0 ? ("split every " + std::to_string(record_split_sec_) + "s").c_str() :
      "one file until stop", record_cache_mb_, latched.size());
    publishEvent("rec_started|" + uri);
    return true;
  }

  bool stopRecording(std::string & msg_out)
  {
    std::unique_ptr<rosbag2_cpp::Writer> writer;
    std::string uri;
    size_t msgs = 0, bytes = 0, errors = 0;
    double dur = 0.0;
    {
      std::lock_guard<std::mutex> lk(rec_mtx_);
      if (!rec_writer_) {
        msg_out = "not recording";
        publishEvent("rec_busy|" + msg_out);
        return false;
      }
      writer = std::move(rec_writer_);
      uri = rec_uri_;
      msgs = rec_msgs_;
      bytes = rec_bytes_;
      errors = rec_errors_;
      dur = (now() - rec_start_).seconds();
    }
    msg_out = "stopping " + uri;
    publishEvent("rec_closing|" + uri);

    // Closing flushes rosbag2's cache — up to record_cache_mb of disk writes.
    // Do it off the executor so the node keeps buffering meanwhile.
    if (rec_close_thread_.joinable()) {
      rec_close_thread_.join();
    }
    rec_close_thread_ = std::thread(
      [this, w = std::move(writer), uri, msgs, bytes, errors, dur]() mutable {
        try {
          w.reset();
        } catch (const std::exception & e) {
          publishEvent(std::string("rec_error|close failed: ") + e.what());
        }
        RCLCPP_INFO(
          get_logger(), "recording stopped: %s: %zu msgs, %.1fs, %.1f MB%s", uri.c_str(), msgs,
          dur, static_cast<double>(bytes) / (1024.0 * 1024.0),
          errors ? (", " + std::to_string(errors) + " write errors").c_str() : "");
        std::ostringstream ev;
        ev << "rec_stopped|" << uri << "|" << msgs << "|" << std::fixed << std::setprecision(1)
           << dur << "|" << static_cast<double>(bytes) / (1024.0 * 1024.0);
        publishEvent(ev.str());
      });
    return true;
  }

  // ---------- trigger / finalize ----------

  // Start a clip around `event` (node clock). Returns false with a reason in
  // msg_out if a clip is already in progress or a bag write is pending.
  bool startClip(rclcpp::Time event, const std::string & label, std::string & msg_out)
  {
    const rclcpp::Time t_now = now();
    if (event > t_now) {event = t_now;}
    const double late = (t_now - event).seconds();
    const rclcpp::Time w_start = event - rclcpp::Duration::from_seconds(pre_sec_);
    const rclcpp::Time w_end = event + rclcpp::Duration::from_seconds(post_sec_);
    const double remaining = (w_end - t_now).seconds();

    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      if (clip_active_ || writing_.load()) {
        msg_out = "busy: clip in progress or bag write pending";
        publishEvent("busy|" + msg_out);
        return false;
      }
      trigger_time_ = event;
      clip_label_ = label;
      clip_.clear();
      for (const auto & m : buffer_) {
        if (m.stamp >= w_start && m.stamp <= w_end) {clip_.push_back(m);}
      }
      // Keep appending live messages only if the window extends into the future.
      clip_active_ = remaining > 0.0;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "clip started: -" << pre_sec_ << "s .. +" << post_sec_ << "s around "
        << event.seconds() << " (trigger arrived " << static_cast<int>(late * 1000.0)
        << " ms after event)";
    if (!label.empty()) {oss << " label=" << label;}
    msg_out = oss.str();
    RCLCPP_INFO(get_logger(), "%s", msg_out.c_str());
    publishEvent("started|" + label);

    if (remaining > 0.0) {
      // Node-clock timer so post_sec also respects use_sim_time.
      finalize_timer_ = rclcpp::create_timer(
        this, get_clock(), rclcpp::Duration::from_seconds(remaining),
        [this]() {
          finalize_timer_->cancel();
          finalizeClip();
        });
    } else {
      // The whole window is already in the past: write it right away.
      finalizeClip();
    }
    return true;
  }

  void finalizeClip()
  {
    std::vector<StampedMsg> clip;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::SerializedMessage>> latched;
    std::unordered_map<std::string, std::string> types;
    rclcpp::Time t0;
    std::string label;
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      clip_active_ = false;
      clip.swap(clip_);
      latched = latched_last_;
      types = types_;
      t0 = trigger_time_;
      label = clip_label_;
    }

    writing_.store(true);
    if (writer_thread_.joinable()) {
      writer_thread_.join();
    }
    writer_thread_ = std::thread(
      [this, clip = std::move(clip), latched = std::move(latched),
       types = std::move(types), t0, label]() mutable {
        try {
          writeBag(clip, latched, types, t0, label);
        } catch (const std::exception & e) {
          RCLCPP_ERROR(get_logger(), "bag write failed: %s", e.what());
          publishEvent(std::string("error|") + e.what());
        }
        writing_.store(false);
      });
  }

  void writeBag(
    const std::vector<StampedMsg> & clip,
    const std::unordered_map<std::string, std::shared_ptr<rclcpp::SerializedMessage>> & latched,
    const std::unordered_map<std::string, std::string> & types,
    const rclcpp::Time & t0,
    const std::string & label)
  {
    if (clip.empty() && latched.empty()) {
      RCLCPP_WARN(get_logger(), "clip empty, nothing to write");
      publishEvent("error|clip empty, nothing to write");
      return;
    }

    // Wall-clock timestamped directory name.
    std::time_t tt = std::time(nullptr);
    std::tm tm{};
    localtime_r(&tt, &tm);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
    std::string uri = outputDir() + "/clip_" + ts;
    if (!label.empty()) {uri += "_" + sanitizeLabel(label);}

    publishEvent("writing|" + uri);

    rosbag2_storage::StorageOptions storage_opts;
    storage_opts.uri = uri;
    storage_opts.storage_id = storage_id_;
    rosbag2_cpp::ConverterOptions conv_opts;
    conv_opts.input_serialization_format = "cdr";
    conv_opts.output_serialization_format = "cdr";

    rosbag2_cpp::Writer writer;
    writer.open(storage_opts, conv_opts);

    // Register every topic that appears in this clip (plus latched ones).
    std::unordered_set<std::string> clip_topics;
    for (const auto & m : clip) {clip_topics.insert(m.topic);}
    for (const auto & [t, _] : latched) {clip_topics.insert(t);}
    for (const auto & t : clip_topics) {
      auto it = types.find(t);
      if (it == types.end()) {continue;}
      rosbag2_storage::TopicMetadata meta;
      meta.name = t;
      meta.type = it->second;
      meta.serialization_format = "cdr";
      writer.create_topic(meta);
    }

    // Writer::write(shared_ptr<SerializedMessage>) takes ownership of the
    // buffer ("the serialized data will no longer be managed by message"),
    // but the ring buffer / latched map still reference these messages —
    // hand the writer a copy instead.
    const auto write_copy =
      [&writer, &types](
      const std::string & topic,
      const std::shared_ptr<rclcpp::SerializedMessage> & msg,
      const rclcpp::Time & stamp) {
        auto it = types.find(topic);
        if (it == types.end()) {return;}
        writer.write(
          std::make_shared<rclcpp::SerializedMessage>(*msg),
          topic, it->second, stamp);
      };

    // Prepend latched messages at the start of the clip window so playback
    // has /tf_static etc. from t=0 — unless that exact sample is already in
    // the clip, which would duplicate it.
    std::unordered_set<const rclcpp::SerializedMessage *> in_clip;
    for (const auto & m : clip) {
      if (latched.count(m.topic)) {in_clip.insert(m.data.get());}
    }
    const rclcpp::Time clip_start =
      clip.empty() ? t0 - rclcpp::Duration::from_seconds(pre_sec_) : clip.front().stamp;
    for (const auto & [topic, msg] : latched) {
      if (in_clip.count(msg.get())) {continue;}
      write_copy(topic, msg, clip_start);
    }

    for (const auto & m : clip) {
      write_copy(m.topic, m.data, m.stamp);
    }

    const double dur = clip.empty() ? 0.0 :
      (clip.back().stamp - clip.front().stamp).seconds();
    RCLCPP_INFO(
      get_logger(), "wrote %s: %zu msgs, %.1fs (trigger at %.3f)",
      uri.c_str(), clip.size(), dur, t0.seconds());
    std::ostringstream ev;
    ev << "done|" << uri << "|" << clip.size() << "|"
       << std::fixed << std::setprecision(1) << dur;
    publishEvent(ev.str());
  }

  std::string outputDir()
  {
    std::lock_guard<std::mutex> lk(dir_mtx_);
    return output_dir_;
  }

  void publishEvent(const std::string & text)
  {
    std_msgs::msg::String m;
    m.data = text;
    event_pub_->publish(m);
  }

  static std::string sanitizeLabel(const std::string & label)
  {
    std::string out;
    for (char c : label) {
      if (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-') {
        out += c;
      }
      if (out.size() >= 32) {break;}
    }
    return out;
  }

  // ---------- status ----------

  void publishStatus()
  {
    size_t bytes = 0, msgs = 0, cap_evictions = 0;
    double span = 0.0;
    std::unordered_map<std::string, size_t> topic_bytes, topic_msgs;
    {
      std::lock_guard<std::mutex> lk(buf_mtx_);
      bytes = buffer_bytes_;
      msgs = buffer_.size();
      if (!buffer_.empty()) {
        span = (buffer_.back().stamp - buffer_.front().stamp).seconds();
      }
      cap_evictions = cap_evictions_;
      cap_evictions_ = 0;
      topic_bytes = topic_bytes_;
      topic_msgs = topic_msgs_;
    }

    // Per-topic inflow since the previous tick (MB/s + Hz), heaviest first.
    std::vector<std::pair<std::string, double>> rates;
    std::unordered_map<std::string, double> hz, bps;
    double total_rate = 0.0;
    for (const auto & [t, b] : topic_bytes) {
      const auto prev = last_topic_bytes_.find(t);
      const size_t before = prev == last_topic_bytes_.end() ? 0 : prev->second;
      const double r = static_cast<double>(b - before) / status_period_sec_ / (1024.0 * 1024.0);
      rates.emplace_back(t, r);
      bps[t] = static_cast<double>(b - before) / status_period_sec_;
      total_rate += r;
      const auto pm = last_topic_msgs_.find(t);
      const size_t m_before = pm == last_topic_msgs_.end() ? 0 : pm->second;
      hz[t] = static_cast<double>(topic_msgs[t] - m_before) / status_period_sec_;
    }
    last_topic_bytes_ = std::move(topic_bytes);
    last_topic_msgs_ = std::move(topic_msgs);
    std::sort(
      rates.begin(), rates.end(),
      [](const auto & a, const auto & b) {return a.second > b.second;});

    const double buffer_mb = static_cast<double>(bytes) / (1024.0 * 1024.0);
    const double retain_sec = pre_sec_ + trigger_slack_sec_;
    const double needed_mb = total_rate * retain_sec;   // what this inflow costs to retain
    const bool cap_hit = cap_evictions > 0;

    auto f1 = [](double v) {
        std::ostringstream o;
        o << std::fixed << std::setprecision(1) << v;
        return o.str();
      };

    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = std::string(get_name()) + ": ring buffer";
    st.hardware_id = get_name();
    st.level = cap_hit ? diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    st.message =
      f1(buffer_mb) + " MB / " + f1(max_buffer_mb_) + " MB cap, " +
      f1(span) + " s of " + f1(retain_sec) + " s retained, " +
      f1(total_rate) + " MB/s in, " + f1(needed_mb) + " MB needed" +
      (cap_hit ? " -- CAP HIT, pre window truncated" : "");
    auto kv = [&st](const std::string & k, const std::string & v) {
        diagnostic_msgs::msg::KeyValue e;
        e.key = k;
        e.value = v;
        st.values.push_back(e);
      };
    kv("buffer_mb", f1(buffer_mb));
    kv("cap_mb", f1(max_buffer_mb_));
    kv("span_sec", f1(span));
    kv("pre_sec", f1(pre_sec_));
    kv("retain_sec", f1(retain_sec));
    kv("msgs", std::to_string(msgs));
    kv("rate_mb_s", f1(total_rate));
    kv("needed_mb", f1(needed_mb));
    kv("cap_hit", cap_hit ? "true" : "false");
    {
      std::lock_guard<std::mutex> lk(rec_mtx_);
      kv("rec_active", rec_writer_ ? "true" : "false");
      if (rec_writer_) {
        kv("rec_uri", rec_uri_);
        kv("rec_sec", f1((now() - rec_start_).seconds()));
        kv("rec_mb", f1(static_cast<double>(rec_bytes_) / (1024.0 * 1024.0)));
        kv("rec_msgs", std::to_string(rec_msgs_));
        kv("rec_errors", std::to_string(rec_errors_));
      }
    }
    // "<topic>" = "X.X MB/s" (buffer_probe 호환). 추가로 "<topic>|hz", "<topic>|bps":
    // 작은 토픽도 정밀하게 보이도록 정수 bytes/s와 메시지 주기를 따로 싣는다.
    for (const auto & [t, r] : rates) {
      kv(t, f1(r) + " MB/s");
      kv(t + "|hz", f1(hz[t]));
      kv(t + "|bps", std::to_string(static_cast<long long>(bps[t])));
    }

    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = now();
    arr.status.push_back(st);
    status_pub_->publish(arr);

    if (cap_hit) {
      std::string top;
      for (size_t i = 0; i < rates.size() && i < 5; ++i) {
        top += "\n    " + f1(rates[i].second) + " MB/s  " + rates[i].first;
      }
      RCLCPP_WARN(get_logger(), "%s -- heaviest topics:%s", st.message.c_str(), top.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "status: %s", st.message.c_str());
    }
  }

  // ---------- members ----------

  double pre_sec_{}, post_sec_{}, trigger_slack_sec_{}, max_buffer_mb_{};
  int queue_depth_{};
  size_t max_buffer_bytes_{};
  std::string output_dir_, storage_id_;
  std::mutex dir_mtx_;          // output_dir_ can change at runtime (GUI folder picker)
  std::vector<std::string> topics_filter_, exclude_, topic_qos_raw_;
  std::unordered_map<std::string, QosOverride> qos_overrides_;
  std::unordered_map<std::string, std::string> applied_qos_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::unordered_map<std::string, rclcpp::GenericSubscription::SharedPtr> subs_;
  std::unordered_map<std::string, std::string> types_;
  std::unordered_set<std::string> latched_topics_;
  std::unordered_set<std::string> failed_topics_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::SerializedMessage>> latched_last_;

  std::mutex buf_mtx_;
  std::deque<StampedMsg> buffer_;
  size_t buffer_bytes_{0};

  bool clip_active_{false};
  std::vector<StampedMsg> clip_;
  rclcpp::Time trigger_time_;
  std::string clip_label_;
  std::atomic<bool> writing_{false};
  std::thread writer_thread_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;
  rclcpp::TimerBase::SharedPtr discover_timer_;
  rclcpp::TimerBase::SharedPtr finalize_timer_;

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr trigger_sub_;

  // manual recording
  double record_split_sec_{}, record_cache_mb_{};
  std::mutex rec_mtx_;
  std::unique_ptr<rosbag2_cpp::Writer> rec_writer_;
  std::string rec_uri_;
  rclcpp::Time rec_start_;
  size_t rec_msgs_{0}, rec_bytes_{0}, rec_errors_{0};
  std::thread rec_close_thread_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr record_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr rec_start_srv_, rec_stop_srv_;
  double status_period_sec_{};
  std::unordered_map<std::string, size_t> topic_bytes_, last_topic_bytes_;
  std::unordered_map<std::string, size_t> topic_msgs_, last_topic_msgs_;
  size_t cap_evictions_{0};
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClipRecorder>();
  // MultiThreadedExecutor: subscription callbacks keep filling the ring buffer
  // while the trigger service / timers run.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
