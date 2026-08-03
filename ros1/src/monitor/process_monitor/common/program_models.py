"""Shared data models for the process monitor."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Pattern, Optional


@dataclass(frozen=True)
class ProcessInfo:
    pid: int
    comm: str
    exe_name: str
    cmdline: str


@dataclass(frozen=True)
class RestartConfig:
    enabled: bool = False
    command: str | tuple[str, ...] | None = None
    cwd: str | None = None
    cooldown_seconds: float = 30.0


@dataclass(frozen=True)
class ProgramConfig:
    name: str
    group: str
    process_name: str | None = None
    cmdline_contains: tuple[str, ...] = field(default_factory=tuple)
    cmdline_regex: str | None = None
    cmdline_pattern: Optional[Pattern[str]] = None
    min_instances: int = 1
    restart: RestartConfig = field(default_factory=RestartConfig)


@dataclass(frozen=True)
class MonitorConfig:
    profile: str
    refresh_seconds: float
    programs: tuple[ProgramConfig, ...]


@dataclass(frozen=True)
class ProgramStatus:
    program: ProgramConfig
    processes: tuple[ProcessInfo, ...]
    message: str = ""

    @property
    def online(self) -> bool:
        return len(self.processes) >= self.program.min_instances
