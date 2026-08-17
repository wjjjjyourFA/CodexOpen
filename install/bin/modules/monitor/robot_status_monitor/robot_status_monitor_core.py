"""Transport-independent robot status aggregation.

The core accepts plain Python values and an explicit monotonic timestamp. ROS
messages, clocks, subscriptions and presentation are deliberately kept in the
adapter under ``ros1/src``.
"""

from dataclasses import dataclass
import configparser
from typing import Dict, Optional, Tuple


STREAM_DEFINITIONS = (
    ("odometry", "里程计 / state_estimation"),
    ("rog_map", "占据地图 / rog_map/occ"),
    ("free_paths", "候选路径 / free_paths"),
    ("cmd_vel", "控制指令 / cmd_vel_corrected"),
    ("goal_valid", "目标状态 / isgoal_vaild"),
    ("waypoint", "当前目标点 / way_point"),
)


@dataclass(frozen=True)
class MonitorConfig:
    display_mode: str
    timeout_sec: float
    waypoint_timeout_sec: float
    refresh_rate: float


@dataclass
class _StreamState:
    last_receive: Optional[float] = None
    detail: str = "等待消息"
    value: Optional[bool] = None


@dataclass(frozen=True)
class StatusRow:
    key: str
    label: str
    online: bool
    fresh: bool
    age_seconds: Optional[float]
    detail: str


def load_runtime_config(path: str,
                        display_mode_override: Optional[str] = None) -> MonitorConfig:
    parser = configparser.ConfigParser()
    if not parser.read(path, encoding="utf-8"):
        raise RuntimeError("cannot read runtime config: {}".format(path))
    if not parser.has_section("runtime"):
        raise RuntimeError("runtime config has no [runtime] section: {}".format(path))

    mode = (display_mode_override or
            parser.get("runtime", "display_mode")).strip().lower()
    if mode not in ("gui", "terminal"):
        raise ValueError("display_mode must be 'gui' or 'terminal', got: {}".format(mode))

    config = MonitorConfig(
        display_mode=mode,
        timeout_sec=parser.getfloat("runtime", "timeout_sec"),
        waypoint_timeout_sec=parser.getfloat("runtime", "waypoint_timeout_sec"),
        refresh_rate=parser.getfloat("runtime", "refresh_rate"),
    )
    if config.timeout_sec <= 0.0:
        raise ValueError("timeout_sec must be positive")
    if config.waypoint_timeout_sec <= 0.0:
        raise ValueError("waypoint_timeout_sec must be positive")
    if config.refresh_rate <= 0.0:
        raise ValueError("refresh_rate must be positive")
    return config


class RobotStatusMonitorCore:
    """Maintains freshness and semantic validity for monitored streams."""

    def __init__(self, config: MonitorConfig):
        self.config = config
        self._states: Dict[str, _StreamState] = {
            key: _StreamState() for key, _ in STREAM_DEFINITIONS
        }

    @staticmethod
    def stream_definitions() -> Tuple[Tuple[str, str], ...]:
        return STREAM_DEFINITIONS

    def _update(self, key: str, now_seconds: float, detail: str,
                value: Optional[bool] = None) -> None:
        if key not in self._states:
            raise KeyError("unknown status stream: {}".format(key))
        state = self._states[key]
        state.last_receive = float(now_seconds)
        state.detail = detail
        state.value = value

    def update_odometry(self, now_seconds: float, x: float, y: float,
                        z: float) -> None:
        self._update(
            "odometry", now_seconds,
            "位置 x={:.2f}, y={:.2f}, z={:.2f}".format(x, y, z))

    def update_rog_map(self, now_seconds: float, point_count: int) -> None:
        self._update("rog_map", now_seconds, "点数 {}".format(point_count))

    def update_free_paths(self, now_seconds: float, point_count: int) -> None:
        self._update(
            "free_paths", now_seconds, "候选路径点 {}".format(point_count))

    def update_command(self, now_seconds: float, linear_x: float,
                       linear_y: float, angular_z: float) -> None:
        self._update(
            "cmd_vel", now_seconds,
            "线速度 x={:.2f}, y={:.2f}；角速度 z={:.2f}".format(
                linear_x, linear_y, angular_z))

    def update_goal_valid(self, now_seconds: float, valid: bool) -> None:
        self._update("goal_valid", now_seconds,
                     "目标有效" if valid else "目标无效", bool(valid))

    def update_waypoint(self, now_seconds: float, x: float, y: float,
                        z: float) -> None:
        self._update(
            "waypoint", now_seconds,
            "x={:.2f}, y={:.2f}, z={:.2f}".format(x, y, z))

    def snapshot(self, now_seconds: float) -> Tuple[StatusRow, ...]:
        result = []
        now = float(now_seconds)
        for key, label in STREAM_DEFINITIONS:
            state = self._states[key]
            age = (None if state.last_receive is None
                   else max(0.0, now - state.last_receive))
            timeout = (self.config.waypoint_timeout_sec
                       if key == "waypoint" else self.config.timeout_sec)
            fresh = age is not None and age <= timeout
            online = fresh and (key != "goal_valid" or state.value is True)
            detail = state.detail
            if key == "goal_valid" and fresh and state.value is False:
                detail += "（话题在线）"
            result.append(StatusRow(key, label, online, fresh, age, detail))
        return tuple(result)
