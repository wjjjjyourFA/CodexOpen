#!/usr/bin/env python3
"""ROS1 central board showing vehicle 95 above vehicle 105."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from common.board_models import MonitorBoardStore
from ui.monitor_board_ui import MonitorBoardUI
from ros1_message_loader import load_process_status_messages


DEFAULT_TOPIC = "/process_monitor/status"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="显示 95/105 两车进程在线状态")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="ProcessStatusArray topic")
    parser.add_argument("--upper-profile", default="ros1_95", help="上半区 profile")
    parser.add_argument("--lower-profile", default="ros1_105", help="下半区 profile")
    parser.add_argument("--refresh", type=float, default=0.5, help="界面刷新周期（秒）")
    parser.add_argument("--stale-timeout", type=float, default=6.0, help="消息超时阈值（秒）")
    parser.add_argument("--node-name", default="monitor_board", help="ROS 节点名")
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto")
    parser.add_argument("--no-clear", action="store_true", help="刷新时不清屏")
    return parser


def main(argv: list[str] | None = None) -> int:
    try:
        import rospy
        _, ProcessStatusArray = load_process_status_messages()
    except ImportError as exc:
        print(
            "ROS1 消息尚不可用，请先编译 process_monitor 并 source devel/setup.bash",
            file=sys.stderr,
        )
        print(f"导入错误: {exc}", file=sys.stderr)
        return 3

    raw_argv = sys.argv if argv is None else ["monitor_board.py", *argv]
    args = build_parser().parse_args(rospy.myargv(argv=raw_argv)[1:])
    rospy.init_node(args.node_name)

    topic = rospy.get_param("~topic", args.topic)
    upper_profile = rospy.get_param("~upper_profile", args.upper_profile)
    lower_profile = rospy.get_param("~lower_profile", args.lower_profile)
    refresh = float(rospy.get_param("~refresh", args.refresh))
    stale_timeout = float(rospy.get_param("~stale_timeout", args.stale_timeout))
    if refresh <= 0 or stale_timeout <= 0:
        rospy.logerr("refresh 和 stale_timeout 必须大于 0")
        return 2

    try:
        store = MonitorBoardStore((upper_profile, lower_profile))
    except ValueError as exc:
        rospy.logerr("monitor_board 配置错误: %s", exc)
        return 2
    ui = MonitorBoardUI(
        upper_profile,
        lower_profile,
        stale_timeout,
        color_mode=args.color,
    )

    def receive(message) -> None:
        if not store.update_from_message(message):
            rospy.logwarn_throttle(10, "忽略未配置的 profile: %s", message.profile)

    subscriber = rospy.Subscriber(topic, ProcessStatusArray, receive, queue_size=10)
    rospy.loginfo(
        "monitor_board started: topic=%s upper=%s lower=%s",
        topic,
        upper_profile,
        lower_profile,
    )

    while not rospy.is_shutdown():
        content = ui.render(store.get(upper_profile), store.get(lower_profile))
        ui.display(content, clear=not args.no_clear)
        rospy.sleep(refresh)
    subscriber.unregister()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
