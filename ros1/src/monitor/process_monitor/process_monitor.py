#!/usr/bin/env python3
"""Process monitor executable entry point and dependency wiring."""

from __future__ import annotations

import argparse
import signal
import sys
from pathlib import Path
from typing import Any

from common.config_loader import ConfigError, load_config, profile_config_path
from common.restart_handler import RestartHandler
from ui.terminal_ui import TerminalUI
from monitor_service import MonitorService
from process_inspector import ProcessInspector

DEFAULT_PROFILE = "default"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="按进程名监控程序状态")
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--profile", help="加载 config/<profile>.json，默认 null")
    source.add_argument("--config", type=Path, help="加载指定的单个 JSON 配置")
    parser.add_argument("--interval", type=float, help="覆盖配置中的刷新间隔（秒）")
    parser.add_argument("--once", action="store_true", help="检查一次后退出，不清屏")
    parser.add_argument("--auto-restart", action="store_true", help="允许拉起配置中已启用的离线程序")
    parser.add_argument("--color", choices=("auto", "always", "never"), default="auto")
    return parser


def main(argv: list[str] | None = None) -> int:
    script_dir = Path(__file__).resolve().parent
    root = script_dir.parent.parent

    args = build_parser().parse_args(argv)
    try:
        config_path = args.config or profile_config_path(
            script_dir / "config", args.profile or DEFAULT_PROFILE
        )
        config = load_config(config_path)
    except ConfigError as exc:
        print(f"配置错误: {exc}", file=sys.stderr)
        return 2

    interval = args.interval if args.interval is not None else config.refresh_seconds
    if interval <= 0:
        print("刷新间隔必须大于 0", file=sys.stderr)
        return 2

    service = MonitorService(
        config=config,
        inspector=ProcessInspector(),
        restarter=RestartHandler(root, script_dir / "logs"),
        ui=TerminalUI(args.color),
        interval=interval,
        auto_restart=args.auto_restart,
    )

    def stop(_signum: int, _frame: Any) -> None:
        service.stop()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    service.run(once=args.once)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
