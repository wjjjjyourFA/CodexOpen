"""Load generated ROS messages despite the process_monitor.py name collision."""

from __future__ import annotations

import sys
from pathlib import Path


PACKAGE_NAME = "process_monitor"


def _promote_generated_package_path() -> None:
    """Put catkin's generated Python package before local process_monitor.py."""
    for entry in tuple(sys.path):
        if not entry:
            continue
        if (Path(entry) / PACKAGE_NAME / "msg").is_dir():
            sys.path.remove(entry)
            sys.path.insert(0, entry)
            return


def load_process_status_messages():
    _promote_generated_package_path()
    from process_monitor.msg import ProcessStatus, ProcessStatusArray

    return ProcessStatus, ProcessStatusArray
