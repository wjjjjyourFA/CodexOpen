"""Linux /proc scanning and process matching."""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable

from common.program_models import ProcessInfo, ProgramConfig


class ProcessInspector:
    def __init__(self, proc_root: Path = Path("/proc")):
        self.proc_root = proc_root

    def scan(self) -> list[ProcessInfo]:
        processes: list[ProcessInfo] = []
        for entry in self.proc_root.iterdir():
            if not entry.name.isdigit():
                continue
            try:
                # 1. 读取进程名 (最多15字符，截断)
                comm = (entry / "comm").read_text(encoding="utf-8").strip()

                # 2. 读取完整命令行参数 (以 \0 分隔)
                raw_cmdline = (entry / "cmdline").read_bytes()

                if raw_cmdline:
                    # 将 \0 替换为空格，去除末尾空字符，规范化连续空格
                    cmdline = " ".join(
                        raw_cmdline.decode("utf-8", errors="replace").split("\0")
                    ).strip()
                else:
                    cmdline = ""

                # 3. 读取可执行文件实际名称 (不受15字符限制)
                try:
                    exe_name = (entry / "exe").resolve(strict=True).name
                except (FileNotFoundError, PermissionError, OSError):
                    exe_name = ""

                processes.append(ProcessInfo(int(entry.name), comm, exe_name, cmdline))
            except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
                # The process may exit between reading individual /proc files.
                continue
        return processes

    @staticmethod
    def matches(program: ProgramConfig, process: ProcessInfo) -> bool:
        if program.process_name:
            # exe_name is not truncated; Linux comm is limited to 15 characters.
            if program.process_name not in (process.exe_name, process.comm):
                return False
            
        if program.cmdline_contains:
            if not all(token in process.cmdline for token in program.cmdline_contains):
                return False
            
        if program.cmdline_pattern:
            # 在 Config 加载时将 regex 编译为 re.Pattern 对象，提高运行时性能
            if program.cmdline_pattern.search(process.cmdline) is None:
                return False

        return bool(program.process_name or program.cmdline_contains or program.cmdline_regex)

    def find_matches(
        self, program: ProgramConfig, processes: Iterable[ProcessInfo]
    ) -> tuple[ProcessInfo, ...]:
        return tuple(process for process in processes if self.matches(program, process))
