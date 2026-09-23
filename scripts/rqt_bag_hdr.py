#!/usr/bin/env python3
# rqt_bag_hdr.py — rqt_bag 을 header.stamp 시간축으로 띄운다.
#
# rqt_bag(humble) 은 메시지를 bag 기록(수신) 시각에 찍는다. 센서끼리 시각이 맞는지(GNSS PPS 동기)
# 보려면 센서가 매긴 header.stamp 로 찍어야 한다. rqt_bag 의 시간 조회(타임라인 막대, 재생 헤드,
# 재생, Plot · Image 뷰)는 전부 Rosbag2 의 get_entry* / get_entries_in_range 와 entry.timestamp 를
# 거치므로, 그 클래스를 header.stamp 로 인덱스하는 것으로 바꿔 끼운다. 설치된 rqt_bag 은 건드리지 않는다.
#
# 토픽마다 시간 기준은 bag_diagnostics 와 같은 규칙이다: header.stamp 가 95% 이상 믿을 만하면(bag
# 시각과 5분 안) header, 아니면 수신 시각 — Ouster 패킷 · String 처럼 header 가 없거나 센서 내부
# 시계를 쓰는 토픽. 수신 시각으로 찍는 토픽은 타임라인 이름 뒤에 [수신 시각] 이 붙는다.
# header 로 찍는 토픽에서도 stamp 가 빠지거나 튄 메시지 하나하나는 수신 시각에 찍는다.
#
# 구간 복사(Save region)는 header 시각으로 고른 메시지를 원래 수신 시각 그대로 새 bag 에 쓴다.
#
# 실행:  rqt → Plugins → Logging → Bag (header.stamp)          (plugin.xml 로 등록, 처음 한 번 rqt --force-discover)
#        ros2 run clip_recorder rqt_bag_hdr [clip_dir ...]      (rqt_bag 과 같은 인자)
# rqt 한 창에 원래 Bag 과 같이 띄워도 바뀌는 건 이 플러그인 창뿐이다 — 아래 바꿔 끼우기는 전부
# HeaderStampRosbag2 로 연 bag 에만 걸린다.

import bisect
import sqlite3
import sys
import threading

from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from rqt_bag import bag as bag_plugin
from rqt_bag import bag_timeline, bag_widget, message_loader_thread, timeline_frame
from rqt_bag.rosbag2 import Rosbag2

import bag_diagnostics as bd

RECV_LABEL = " [수신 시각]"
_tls = threading.local()       # 스레드마다 db 연결 (sqlite 연결은 스레드끼리 못 나눈다) + header_mode


def _connect(db):
    cons = _tls.__dict__.setdefault("cons", {})
    if db not in cons:
        cons[db] = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
    return cons[db]


def _has_header(type_name):
    """첫 필드가 std_msgs/Header 인가. 메시지 정의가 설치돼 있지 않으면 None (본문으로 판정)."""
    try:
        fields = get_message(type_name).get_fields_and_field_types()
    except Exception:
        return None
    return next(iter(fields.items()), None) == ("header", "std_msgs/Header")


class _Entry:
    """rqt_bag 의 Entry(topic, data, timestamp) 와 같은 모양. timestamp 는 타임라인 시각(header.stamp),
    recv_timestamp 는 bag 기록 시각. data 는 쓸 때 읽는다 — 타임라인 인덱스는 시각만 필요한데
    원래 rqt_bag 은 클립 본문 전체를 읽었다."""
    __slots__ = ("topic", "timestamp", "recv_timestamp", "_src")

    def __init__(self, topic, timestamp, recv_timestamp, src):
        self.topic = topic
        self.timestamp = timestamp
        self.recv_timestamp = recv_timestamp
        self._src = src                                  # (db 경로, messages.id)

    @property
    def data(self):
        db, rid = self._src
        row = _connect(db).execute("SELECT data FROM messages WHERE id=?", (rid,)).fetchone()
        return bytes(row[0]) if row and row[0] is not None else b""

    def as_recorded(self):
        return _Entry(self.topic, self.recv_timestamp, self.recv_timestamp, self._src)

    def __eq__(self, other):
        return isinstance(other, _Entry) and (self._src, self.timestamp) == (other._src, other.timestamp)

    def __hash__(self):
        return hash((self._src, self.timestamp))


class HeaderStampRosbag2(Rosbag2):
    """메시지 시각을 header.stamp 로 돌려주는 Rosbag2. 여러 파일로 나뉜 bag(_0, _1 …)도 읽는다."""

    def __init__(self, bag_path):
        super().__init__(bag_path)
        dbs = [str(p) for p in bd._db3_files(bag_path)]
        if not dbs:
            raise RuntimeError("sqlite3(.db3) 클립만 열 수 있음")
        rows = {}                                        # topic -> [(recv, hdr|None, db, id)]
        for db in dbs:
            con = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
            names, no_header = {}, []
            for tid, name, typ in con.execute("SELECT id, name, type FROM topics"):
                names[tid] = rows.setdefault(name, [])
                if _has_header(typ) is False:
                    no_header.append(tid)
            # 한 번만 훑는다 (topic_id 인덱스가 없다). header 가 없는 타입은 본문을 안 읽는다 (Ouster 패킷)
            skip = ",".join(str(t) for t in no_header) or "NULL"
            for rid, tid, recv, h in con.execute(
                    "SELECT id, topic_id, timestamp, CASE WHEN topic_id IN "
                    f"({skip}) THEN NULL ELSE substr(data,1,12) END FROM messages"):
                names[tid].append((recv, bd._parse_header_ns(h), db, rid))
            con.close()

        self._index = {}                                 # topic -> (시각들, [(시각, topic, recv, src)])
        every = []
        recv_mode, patched = [], {}
        for name, msgs in rows.items():
            use_header = bool(msgs) and bd.use_header_stamps(msgs)
            if not use_header:
                recv_mode.append(name)
            recs = []
            for recv, h, db, rid in msgs:
                ok = use_header and bd.header_usable(recv, h)
                if use_header and not ok:
                    patched[name] = patched.get(name, 0) + 1
                recs.append((h if ok else recv, name, recv, (db, rid)))
            recs.sort(key=lambda r: (r[0], r[2]))
            self._index[name] = ([r[0] for r in recs], recs)
            every.extend(recs)
        every.sort(key=lambda r: (r[0], r[2]))
        self._all = ([r[0] for r in every], every)
        if every:
            self.start_time = Time(nanoseconds=every[0][0])
            self.duration = Time(nanoseconds=every[-1][0]) - self.start_time

        self.recv_topics = set(recv_mode)
        print(f"[rqt_bag_hdr] {bag_path}: header.stamp 기준 {len(rows) - len(recv_mode)}개 토픽, "
              f"수신 시각 기준 {len(recv_mode)}개 {sorted(recv_mode)}", flush=True)
        for name, n in sorted(patched.items()):
            print(f"[rqt_bag_hdr]   {name}: header.stamp 가 없거나 튄 {n}개는 수신 시각에 찍음", flush=True)

    def _lookup(self, topic):
        if topic is None:
            return self._all
        return self._index.get(topic, ([], []))

    @staticmethod
    def _entry(rec):
        t, topic, recv, src = rec
        return _Entry(topic, t, recv, src)

    def get_entry(self, timestamp, topic=None):
        stamps, recs = self._lookup(topic)
        i = bisect.bisect_right(stamps, timestamp.nanoseconds) - 1
        return self._entry(recs[i]) if i >= 0 else None

    def get_entry_after(self, timestamp, topic=None):
        stamps, recs = self._lookup(topic)
        i = bisect.bisect_right(stamps, timestamp.nanoseconds)
        return self._entry(recs[i]) if i < len(recs) else None

    def get_entries_in_range(self, t_start, t_end, topic=None):
        stamps, recs = self._lookup(topic)
        lo = bisect.bisect_left(stamps, t_start.nanoseconds)
        hi = bisect.bisect_right(stamps, t_end.nanoseconds)
        return [self._entry(r) for r in recs[lo:hi]]


class HeaderBagWidget(bag_widget.BagWidget):
    """원래 BagWidget 에서 bag 을 HeaderStampRosbag2 로 여는 것만 다르다."""

    def __init__(self, context, publish_clock):
        _install()
        super().__init__(context, publish_clock)
        self.setWindowTitle(self.windowTitle() + " (header.stamp)")

    def load_bag(self, filename):
        # 원래 load_bag 은 모듈 이름 Rosbag2 로 bag 을 만든다 → 이 스레드에서만 header 판으로 (_install)
        _tls.header_mode = True
        try:
            super().load_bag(filename)
        finally:
            _tls.header_mode = False


class HeaderStampBag(bag_plugin.Bag):
    """rqt 플러그인 — Plugins → Logging → Bag (header.stamp). Bag.__init__ 과 같고 위젯만 다르다."""

    def __init__(self, context):
        super(bag_plugin.Bag, self).__init__(context)     # Bag.__init__ 은 원래 위젯을 만든다 — 건너뛴다
        self.setObjectName("HeaderStampBag")
        args = self._parse_args(context.argv())
        self._widget = HeaderBagWidget(context, args.clock)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + " (%d)" % context.serial_number())
        context.add_widget(self._widget)

        def load_bags():
            for bagfile in args.bagfiles:
                self._widget.load_bag(bagfile)
        threading.Thread(target=load_bags).start()


_installed = False


def _install():
    """rqt_bag 에 바꿔 끼운다. 전부 HeaderStampRosbag2 로 연 bag 에만 걸려서 같은 rqt 의 원래 Bag 창은 그대로다."""
    global _installed
    if _installed:
        return
    _installed = True

    def open_bag(path, *args, **kwargs):
        cls = HeaderStampRosbag2 if getattr(_tls, "header_mode", False) else Rosbag2
        return cls(path, *args, **kwargs)
    bag_widget.Rosbag2 = open_bag          # Recorder 는 Rosbag2 를 따로 import 한다 — 녹화는 그대로

    # 재생 헤드 위치의 메시지를 토픽 없이 시각으로만 찾던 것 → 토픽으로 찾는다. header.stamp 는
    # 토픽끼리 똑같은 게 흔해서(image_raw · camera_info · metadata) 토픽 없이 찾으면 남의 메시지가 나온다.
    orig_get = message_loader_thread.MessageLoaderThread._get_message

    def load_message(self, rosbag, position):
        if not isinstance(rosbag, HeaderStampRosbag2):
            return orig_get(self, rosbag, position)
        key = (rosbag.bag_path, position)
        if key not in self._message_cache:
            with self.timeline._bag_lock:
                self._message_cache[key] = rosbag.get_entry(Time(nanoseconds=position), self.topic)
            self._message_cache_keys.append(key)
            if len(self._message_cache) > self._message_cache_capacity:
                del self._message_cache[self._message_cache_keys.pop(0)]
        return self._message_cache[key]
    message_loader_thread.MessageLoaderThread._get_message = load_message

    orig_trimmed = timeline_frame.TimelineFrame._trimmed_topic_name

    def trimmed(self, topic_name):
        scene, topic = self.scene(), "/" + topic_name.lstrip("/")
        if scene is not None and any(topic in getattr(b, "recv_topics", ()) for b in scene._bags):
            topic_name += RECV_LABEL
        return orig_trimmed(self, topic_name)
    timeline_frame.TimelineFrame._trimmed_topic_name = trimmed

    orig_export = bag_timeline.BagTimeline._run_export_region

    def export(self, writer, topics, start, end, bag_entries, *rest):
        bag_entries = [(b, e.as_recorded() if isinstance(e, _Entry) else e) for b, e in bag_entries]
        return orig_export(self, writer, topics, start, end, bag_entries, *rest)
    bag_timeline.BagTimeline._run_export_region = export


def main():
    # 플러그인 탐색 캐시(rqt --force-discover) 없이도 뜨게 원래 Bag 플러그인 이름으로 띄우고 위젯만
    # 바꾼다 — 이 프로세스엔 Bag 창이 하나뿐이다.
    from rqt_gui.main import Main
    bag_plugin.BagWidget = HeaderBagWidget
    plugin = "rqt_bag.bag.Bag"
    return Main(filename=plugin).main(
        standalone=plugin, plugin_argument_provider=bag_plugin.Bag.add_arguments)


if __name__ == "__main__":
    sys.exit(main())
