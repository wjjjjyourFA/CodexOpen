import unittest
from pathlib import Path
from types import SimpleNamespace

from common.config_loader import load_config
from common.program_models import ProcessInfo, ProgramConfig, ProgramStatus
from ros1_convert import (
    fill_snapshot_message,
    node_name_for_profile,
    without_node_name_remap,
)


class FakeProcessStatus:
    OFFLINE = 0
    ONLINE = 1

    def __init__(self):
        self.name = ""
        self.group = ""
        self.state = self.OFFLINE
        self.pids = []
        self.detail = ""


class FakeSnapshot:
    def __init__(self):
        self.header = SimpleNamespace(stamp=None)
        self.profile = ""
        self.online_count = 0
        self.total_count = 0
        self.programs = []


class Ros1ConvertTest(unittest.TestCase):
    def test_node_name_suffix_comes_from_json_profile(self):
        config = load_config(
            Path(__file__).resolve().parent.parent / "config" / "ros1_95.json"
        )
        self.assertEqual(
            "process_monitor_converter_ros1_95",
            node_name_for_profile(config.profile),
        )

    def test_node_name_replaces_invalid_ros_name_characters(self):
        self.assertEqual(
            "process_monitor_converter_vehicle_95_cn",
            node_name_for_profile("vehicle-95.cn"),
        )

    def test_json_derived_name_is_not_overridden_by_roslaunch(self):
        self.assertEqual(
            ["ros1_convert.py", "--profile", "config_file", "__ns:=/vehicle"],
            without_node_name_remap(
                [
                    "ros1_convert.py",
                    "--profile",
                    "config_file",
                    "__name:=process_monitor_converter",
                    "__ns:=/vehicle",
                ]
            ),
        )

    def test_builds_complete_online_offline_snapshot(self):
        online_program = ProgramConfig("online_app", "control", process_name="online_app")
        offline_program = ProgramConfig("offline_app", "plan", process_name="offline_app")
        statuses = (
            ProgramStatus(
                online_program,
                (ProcessInfo(42, "online_app", "online_app", "/opt/online_app"),),
            ),
            ProgramStatus(offline_program, (), "等待拉起"),
        )

        snapshot = fill_snapshot_message(
            FakeSnapshot(), FakeProcessStatus, statuses, "ros1_105", stamp=123
        )

        self.assertEqual("ros1_105", snapshot.profile)
        self.assertEqual(1, snapshot.online_count)
        self.assertEqual(2, snapshot.total_count)
        self.assertEqual(FakeProcessStatus.ONLINE, snapshot.programs[0].state)
        self.assertEqual([42], snapshot.programs[0].pids)
        self.assertEqual(FakeProcessStatus.OFFLINE, snapshot.programs[1].state)
        self.assertEqual("等待拉起", snapshot.programs[1].detail)


if __name__ == "__main__":
    unittest.main()
