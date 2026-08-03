"""Thread-safe state models used by the central monitor board."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Any


OFFLINE = 0
ONLINE = 1


@dataclass(frozen=True)
class BoardProgramStatus:
    name: str
    group: str
    state: int
    pids: tuple[int, ...]
    detail: str

    @property
    def online(self) -> bool:
        return self.state == ONLINE


@dataclass(frozen=True)
class BoardSnapshot:
    profile: str
    online_count: int
    total_count: int
    programs: tuple[BoardProgramStatus, ...]
    received_at: float
    source_stamp: float | None = None


class MonitorBoardStore:
    def __init__(self, expected_profiles: tuple[str, str]):
        if len(set(expected_profiles)) != 2:
            raise ValueError("上下区域必须配置两个不同的 profile")
        self.expected_profiles = expected_profiles
        self._snapshots: dict[str, BoardSnapshot] = {}
        self._lock = threading.Lock()

    def update_from_message(self, message: Any, now: float | None = None) -> bool:
        """Store one ROS message; return False when its profile is not displayed."""
        if message.profile not in self.expected_profiles:
            return False
        received_at = time.monotonic() if now is None else now
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        source_stamp = stamp.to_sec() if stamp and hasattr(stamp, "to_sec") else None
        programs = tuple(
            BoardProgramStatus(
                name=item.name,
                group=item.group,
                state=int(item.state),
                pids=tuple(int(pid) for pid in item.pids),
                detail=item.detail,
            )
            for item in message.programs
        )
        snapshot = BoardSnapshot(
            profile=message.profile,
            online_count=sum(program.online for program in programs),
            total_count=len(programs),
            programs=programs,
            received_at=received_at,
            source_stamp=source_stamp,
        )
        with self._lock:
            self._snapshots[message.profile] = snapshot
        return True

    def get(self, profile: str) -> BoardSnapshot | None:
        with self._lock:
            return self._snapshots.get(profile)
