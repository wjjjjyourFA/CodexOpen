"""Offline process restart extension point and default implementation."""

from __future__ import annotations

import os
import shlex
import subprocess
from pathlib import Path

from common.program_models import ProgramConfig


def expand_value(value: str, root: Path) -> str:
    return os.path.expanduser(
        os.path.expandvars(value.replace("${ROOT}", str(root)))
    )


class RestartHandler:
    """Override restart() to integrate systemd, Docker, SSH, or another launcher."""

    def __init__(self, root: Path, log_dir: Path):
        self.root = root
        self.log_dir = log_dir
        self.last_attempt: dict[str, float] = {}

    def restart(self, program: ProgramConfig, now: float) -> tuple[bool, str]:
        restart = program.restart
        if not restart.enabled:
            return False, "单项未启用"
        if not restart.command:
            return False, "未配置启动命令"
        last = self.last_attempt.get(program.name, 0.0)
        if now - last < restart.cooldown_seconds:
            remaining = restart.cooldown_seconds - (now - last)
            return False, f"冷却中 {remaining:.0f}s"

        self.last_attempt[program.name] = now
        self.log_dir.mkdir(parents=True, exist_ok=True)
        log_path = self.log_dir / f"{program.name}.log"
        cwd = expand_value(restart.cwd, self.root) if restart.cwd else str(self.root)
        shell = isinstance(restart.command, str)
        if shell:
            command: str | list[str] = expand_value(restart.command, self.root)
        else:
            command = [expand_value(part, self.root) for part in restart.command]

        try:
            with log_path.open("ab", buffering=0) as log_stream:
                subprocess.Popen(
                    command,
                    cwd=cwd,
                    shell=shell,
                    stdin=subprocess.DEVNULL,
                    stdout=log_stream,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    close_fds=True,
                )
            shown = command if isinstance(command, str) else shlex.join(command)
            return True, f"已拉起: {shown}"
        except (OSError, ValueError) as exc:
            return False, f"拉起失败: {exc}"
