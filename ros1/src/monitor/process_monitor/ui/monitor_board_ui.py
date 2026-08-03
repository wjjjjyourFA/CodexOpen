"""Terminal renderer for the 95/105 central monitor board."""

from __future__ import annotations

import sys
import time
from datetime import datetime
from typing import TextIO

from common.board_models import BoardSnapshot


GREEN = "\033[1;32m"
RED = "\033[1;31m"
YELLOW = "\033[1;33m"
CYAN = "\033[1;36m"
RESET = "\033[0m"
CLEAR_SCREEN = "\033[2J\033[H"
LINE_WIDTH = 96


class MonitorBoardUI:
    def __init__(
        self,
        upper_profile: str,
        lower_profile: str,
        stale_timeout: float,
        color_mode: str = "auto",
        output: TextIO = sys.stdout,
    ):
        self.profiles = (upper_profile, lower_profile)
        self.stale_timeout = stale_timeout
        self.output = output
        self.color_enabled = color_mode == "always" or (
            color_mode == "auto" and output.isatty()
        )

    def _paint(self, text: str, color: str) -> str:
        return f"{color}{text}{RESET}" if self.color_enabled else text

    @staticmethod
    def _vehicle_label(profile: str) -> str:
        if profile.startswith("ros1_"):
            return profile[len("ros1_") :]
        return profile

    def _render_section(
        self, profile: str, snapshot: BoardSnapshot | None, now: float
    ) -> list[str]:
        label = self._vehicle_label(profile)
        if snapshot is None:
            title = f" {label} 车辆 / {profile}  "
            return [
                self._paint(title.center(LINE_WIDTH, "="), CYAN),
                self._paint("尚未收到 ROS 状态消息".center(LINE_WIDTH), YELLOW),
                "=" * LINE_WIDTH,
            ]

        age = max(0.0, now - snapshot.received_at)
        stale = age > self.stale_timeout
        health = "通信超时" if stale else f"在线 {snapshot.online_count}/{snapshot.total_count}"
        health_color = RED if stale else GREEN
        title = f" {label} 车辆 / {profile}  {health}  消息年龄 {age:.1f}s "
        lines = [
            self._paint(title.center(LINE_WIDTH, "="), health_color),
            f"{'分组':<12} {'程序名':<32} {'状态':<10} {'PID':<20} 说明",
            "-" * LINE_WIDTH,
        ]
        for program in snapshot.programs:
            if stale:
                state = self._paint("超时", YELLOW)
            elif program.online:
                state = self._paint("在线", GREEN)
            else:
                state = self._paint("离线", RED)
            pids = ",".join(str(pid) for pid in program.pids) or "-"
            lines.append(
                f"{program.group:<12} {program.name:<32} {state:<10} {pids:<20} {program.detail}"
            )
        lines.append("=" * LINE_WIDTH)
        return lines

    def render(
        self,
        upper: BoardSnapshot | None,
        lower: BoardSnapshot | None,
        now: float | None = None,
    ) -> str:
        current = time.monotonic() if now is None else now
        lines = [
            self._paint("UGV 进程总控面板".center(LINE_WIDTH), CYAN),
            f"刷新时间: {datetime.now():%F %T}".center(LINE_WIDTH),
            "",
        ]
        lines.extend(self._render_section(self.profiles[0], upper, current))
        lines.append("")
        lines.extend(self._render_section(self.profiles[1], lower, current))
        lines.append("Ctrl+C 退出；超过设定时间未收到新消息时显示“通信超时”。")
        return "\n".join(lines)

    def display(self, content: str, clear: bool = True) -> None:
        if clear and self.output.isatty():
            print(CLEAR_SCREEN, end="", file=self.output)
        print(content, flush=True, file=self.output)
