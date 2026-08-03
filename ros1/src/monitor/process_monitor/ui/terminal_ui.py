"""Terminal presentation only; contains no process-monitoring decisions."""

from __future__ import annotations

import sys
from datetime import datetime
from typing import TextIO

from common.program_models import MonitorConfig, ProgramStatus


GREEN = "\033[1;32m"
RED = "\033[1;31m"
YELLOW = "\033[1;33m"
RESET = "\033[0m"
CLEAR_SCREEN = "\033[2J\033[H"


class TerminalUI:
    def __init__(self, color_mode: str = "auto", output: TextIO = sys.stdout):
        self.output = output
        self.color_enabled = color_mode == "always" or (
            color_mode == "auto" and output.isatty()
        )

    def _paint(self, text: str, color: str) -> str:
        return f"{color}{text}{RESET}" if self.color_enabled else text

    def render(
        self,
        config: MonitorConfig,
        statuses: tuple[ProgramStatus, ...],
        auto_restart: bool,
    ) -> str:
        online_count = sum(status.online for status in statuses)
        lines = [
            f"进程监控  配置: {config.profile}  更新时间: {datetime.now():%F %T}",
            f"在线 {online_count}/{len(statuses)}  自动拉起: {'已开启' if auto_restart else '未开启'}",
            "-" * 78,
            f"{'分组':<10} {'程序/进程名':<28} {'状态':<8} {'PID':<16} 说明",
            "-" * 78,
        ]
        for status in statuses:
            state = self._paint("在线", GREEN) if status.online else self._paint("离线", RED)
            pids = ",".join(str(item.pid) for item in status.processes) or "-"
            message = (
                self._paint(status.message, YELLOW) if status.message else ""
            )
            lines.append(
                f"{status.program.group:<10} {status.program.name:<28} "
                f"{state:<8} {pids:<16} {message}"
            )
        lines.extend(
            [
                "-" * 78,
                "Ctrl+C 退出；自动拉起需同时使用 --auto-restart 且配置 restart.enabled=true。",
            ]
        )
        return "\n".join(lines)

    def display(self, content: str, clear: bool) -> None:
        if clear and self.output.isatty():
            print(CLEAR_SCREEN, end="", file=self.output)
        print(content, flush=True, file=self.output)
