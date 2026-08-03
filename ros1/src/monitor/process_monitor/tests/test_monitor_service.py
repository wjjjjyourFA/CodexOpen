import io
import unittest
from unittest.mock import Mock

from common.program_models import MonitorConfig, ProcessInfo, ProgramConfig
from monitor_service import MonitorService
from process_inspector import ProcessInspector
from ui.terminal_ui import TerminalUI


class StubInspector(ProcessInspector):
    def scan(self):
        return [ProcessInfo(42, "demo", "demo", "/opt/demo")]


class MonitorServiceTest(unittest.TestCase):
    def test_check_once_builds_online_and_offline_statuses(self):
        config = MonitorConfig(
            profile="test",
            refresh_seconds=2,
            programs=(
                ProgramConfig("demo", "test", process_name="demo"),
                ProgramConfig("missing", "test", process_name="missing"),
            ),
        )
        service = MonitorService(
            config=config,
            inspector=StubInspector(),
            restarter=Mock(),
            ui=TerminalUI("never", io.StringIO()),
            interval=2,
            auto_restart=False,
        )
        statuses = service.check_once()
        self.assertTrue(statuses[0].online)
        self.assertFalse(statuses[1].online)


if __name__ == "__main__":
    unittest.main()
