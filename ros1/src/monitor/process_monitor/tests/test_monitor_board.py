import io
import unittest
from types import SimpleNamespace

from modules.monitor.process_monitor.common.board_models import MonitorBoardStore, ONLINE
from modules.monitor.process_monitor.ui.monitor_board_ui import MonitorBoardUI


def status_message(profile, programs):
    return SimpleNamespace(
        profile=profile,
        header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 100.0)),
        programs=[SimpleNamespace(**program) for program in programs],
    )


class MonitorBoardTest(unittest.TestCase):
    def test_routes_95_and_105_to_separate_snapshots(self):
        store = MonitorBoardStore(("ros1_95", "ros1_105"))
        store.update_from_message(
            status_message(
                "ros1_95",
                [{"name": "control", "group": "控制", "state": ONLINE, "pids": [42], "detail": ""}],
            ),
            now=10.0,
        )
        store.update_from_message(
            status_message(
                "ros1_105",
                [{"name": "lidar", "group": "驱动", "state": 0, "pids": [], "detail": ""}],
            ),
            now=11.0,
        )
        self.assertEqual("control", store.get("ros1_95").programs[0].name)
        self.assertEqual("lidar", store.get("ros1_105").programs[0].name)

    def test_renders_95_above_105_and_marks_stale_data(self):
        store = MonitorBoardStore(("ros1_95", "ros1_105"))
        for profile in store.expected_profiles:
            store.update_from_message(
                status_message(
                    profile,
                    [{"name": profile, "group": "test", "state": ONLINE, "pids": [1], "detail": ""}],
                ),
                now=10.0,
            )
        ui = MonitorBoardUI(
            "ros1_95", "ros1_105", stale_timeout=5, color_mode="never", output=io.StringIO()
        )
        current = ui.render(store.get("ros1_95"), store.get("ros1_105"), now=12.0)
        self.assertLess(current.index("95 车辆"), current.index("105 车辆"))
        self.assertIn("在线", current)

        stale = ui.render(store.get("ros1_95"), store.get("ros1_105"), now=20.0)
        self.assertIn("通信超时", stale)
        self.assertIn("超时", stale)


if __name__ == "__main__":
    unittest.main()
