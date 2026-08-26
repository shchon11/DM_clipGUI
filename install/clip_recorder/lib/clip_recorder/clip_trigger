#!/usr/bin/env python3
# clip_trigger.py — keyboard trigger for clip_recorder.
#
# Keeps a publisher on /<node>/trigger open, so pressing Enter fires a trigger
# stamped with the *current* time in well under a millisecond (no CLI startup,
# no discovery wait). The recorder cuts [now - pre_sec, now + post_sec].
#
#   ros2 run clip_recorder clip_trigger                 # Enter = trigger
#   ros2 run clip_recorder clip_trigger --label test    # default label
#   ros2 run clip_recorder clip_trigger --node my_rec   # non-default node name
#
# At the prompt:  Enter → trigger,  <text>+Enter → trigger with that label,  q → quit

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


def main():
    ap = argparse.ArgumentParser(description="clip_recorder 키보드 트리거")
    ap.add_argument("--node", default="clip_recorder", help="레코더 노드 이름")
    ap.add_argument("--label", default="", help="기본 라벨 (클립 폴더명 접미사)")
    args, ros_args = ap.parse_known_args()

    rclpy.init(args=ros_args or None)
    node = Node("clip_trigger")
    topic = f"/{args.node}/trigger"
    pub = node.create_publisher(Header, topic, 10)

    # Wait briefly for the recorder to match so the first trigger is not lost.
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    n_sub = pub.get_subscription_count()
    if n_sub:
        print(f"연결됨: {topic} ← 레코더 {n_sub}개")
    else:
        print(f"경고: {topic} 구독자가 없습니다. clip_recorder가 실행 중인가요? (그래도 계속 진행)")
    print("Enter = 트리거   |   라벨 입력 후 Enter = 라벨 붙여서 트리거   |   q = 종료")

    count = 0
    try:
        while rclpy.ok():
            try:
                line = input("> ")
            except EOFError:
                break
            line = line.strip()
            if line.lower() == "q":
                break
            h = Header()
            h.stamp = node.get_clock().now().to_msg()
            h.frame_id = line or args.label
            pub.publish(h)
            count += 1
            n_sub = pub.get_subscription_count()
            stamp = h.stamp.sec + h.stamp.nanosec * 1e-9
            note = "" if n_sub else "   (구독자 없음!)"
            print(f"  #{count} 트리거 전송  t={stamp:.3f}"
                  + (f"  label={h.frame_id}" if h.frame_id else "") + note)
    except KeyboardInterrupt:
        pass
    print(f"종료 (트리거 {count}회)")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
