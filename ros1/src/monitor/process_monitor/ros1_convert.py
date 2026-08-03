#!/usr/bin/env python3
"""Publish process_monitor snapshots as ROS1 messages."""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Any, Sequence

# catkin's devel-space relay executes this source file from another directory.
# Keep the monitor's flat modules importable in both source and install spaces.
SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from common.config_loader import ConfigError, load_config, profile_config_path
from common.program_models import ProgramStatus
from monitor_service import collect_statuses
from process_inspector import ProcessInspector
from ros1_message_loader import load_process_status_messages


PACKAGE_NAME = "process_monitor"
DEFAULT_TOPIC = "/process_monitor/status"
DEFAULT_NODE_NAME = "process_monitor_converter"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="通过 ROS1 发布进程在线/离线状态")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--profile", help="加载 config/<profile>.json")
    source.add_argument("--config", type=Path, help="加载指定 JSON 配置")
    parser.add_argument("--topic", default=DEFAULT_TOPIC, help="状态快照 topic")
    parser.add_argument("--interval", type=float, help="覆盖 JSON 中的发布间隔（秒）")
    parser.add_argument("--node-name", help="覆盖 ROS 节点名；默认根据已加载的 profile 自动生成")
    parser.add_argument("--once", action="store_true", help="只发布一帧后退出")
    return parser


def find_config_dir(script_dir: Path) -> Path:
    local_config = script_dir / "config"
    if local_config.is_dir():
        return local_config
    try:
        import rospkg
    except ImportError as exc:
        raise ConfigError("找不到 process_monitor 的 config 目录") from exc
    try:
        return Path(rospkg.RosPack().get_path(PACKAGE_NAME)) / "config"
    except rospkg.ResourceNotFound as exc:
        raise ConfigError("找不到 process_monitor 的 config 目录") from exc


def resolve_config_path(
    script_dir: Path, config_arg: Path | None, profile: str | None
) -> Path:
    if config_arg:
        return config_arg
    if not profile:
        raise ConfigError("必须通过 --profile、--config 或 ROS 私有参数 ~profile/~config 指定配置")
    return profile_config_path(find_config_dir(script_dir), profile)


def node_name_for_profile(profile: str) -> str:
    """Build a valid, stable ROS node name from a configured profile."""
    safe_profile = re.sub(r"[^A-Za-z0-9_]", "_", profile)
    return f"{DEFAULT_NODE_NAME}_{safe_profile}"


def without_node_name_remap(argv: Sequence[str]) -> list[str]:
    """Let the loaded JSON profile, rather than roslaunch, own the node name."""
    return [argument for argument in argv if not argument.startswith("__name:=")]


def fill_snapshot_message(
    snapshot: Any,
    status_message_type: type,
    statuses: Sequence[ProgramStatus],
    profile: str,
    stamp: Any,
) -> Any:
    """Convert internal status models without depending on ROS during unit tests."""
    snapshot.header.stamp = stamp
    snapshot.profile = profile
    snapshot.online_count = sum(status.online for status in statuses)
    snapshot.total_count = len(statuses)
    snapshot.programs = []
    for status in statuses:
        message = status_message_type()
        message.name = status.program.name
        message.group = status.program.group
        message.state = message.ONLINE if status.online else message.OFFLINE
        message.pids = [process.pid for process in status.processes]
        message.detail = status.message
        snapshot.programs.append(message)
    return snapshot


def main(argv: list[str] | None = None) -> int:
    try:
        import rospy
        ProcessStatus, ProcessStatusArray = load_process_status_messages()
    except ImportError as exc:
        print(
            "ROS1 消息尚不可用，请先在 catkin 工作空间中编译 process_monitor 并 source devel/setup.bash",
            file=sys.stderr,
        )
        print(f"导入错误: {exc}", file=sys.stderr)
        return 3

    raw_argv = sys.argv if argv is None else ["ros1_convert.py", *argv]
    args = build_parser().parse_args(rospy.myargv(argv=raw_argv)[1:])

    # CLI config can be loaded before rospy.init_node(), so rosrun also gets a
    # profile-specific name. ROS private params remain supported for backwards
    # compatibility, but cannot influence a node name after ROS has initialized.
    config = None
    if args.profile or args.config:
        try:
            config_path = resolve_config_path(SCRIPT_DIR, args.config, args.profile)
            config = load_config(config_path)
        except ConfigError as exc:
            print(f"process_monitor 配置错误: {exc}", file=sys.stderr)
            return 2

    node_name = args.node_name or (
        node_name_for_profile(config.profile) if config else DEFAULT_NODE_NAME
    )
    # roslaunch injects a __name remap based on its <node> tag. Drop only that
    # remap so config.profile determines the name; keep __ns and topic remaps.
    rospy.init_node(node_name, argv=without_node_name_remap(raw_argv))

    if config is None:
        profile = rospy.get_param("~profile", None)
        config_value = rospy.get_param("~config", None)
        config_path_arg = Path(config_value) if config_value else None
        try:
            config_path = resolve_config_path(SCRIPT_DIR, config_path_arg, profile)
            config = load_config(config_path)
        except ConfigError as exc:
            rospy.logerr("process_monitor 配置错误: %s", exc)
            return 2

    topic = rospy.get_param("~topic", args.topic)

    configured_interval = args.interval if args.interval is not None else config.refresh_seconds
    interval = float(rospy.get_param("~interval", configured_interval))
    if interval <= 0:
        rospy.logerr("发布间隔必须大于 0")
        return 2

    publisher = rospy.Publisher(topic, ProcessStatusArray, queue_size=1, latch=True)
    inspector = ProcessInspector()
    rospy.loginfo(
        "process_monitor ROS1 publisher started: node=%s profile=%s topic=%s interval=%.3fs",
        rospy.get_name(),
        config.profile,
        topic,
        interval,
    )

    while not rospy.is_shutdown():
        statuses = collect_statuses(config, inspector)
        snapshot = fill_snapshot_message(
            ProcessStatusArray(), ProcessStatus, statuses, config.profile, rospy.Time.now()
        )
        publisher.publish(snapshot)
        if args.once:
            rospy.sleep(0.1)
            break
        rospy.sleep(interval)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
