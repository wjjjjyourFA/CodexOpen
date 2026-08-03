import tempfile
import unittest
from pathlib import Path

from common.program_models import ProcessInfo, ProgramConfig
from process_inspector import ProcessInspector


class ProcessInspectorTest(unittest.TestCase):
    def test_exact_executable_name_match(self):
        program = ProgramConfig(name="Tracker", group="control", process_name="Tracker_module")
        processes = [
            ProcessInfo(10, "Tracker_module", "Tracker_module", "/opt/Tracker_module"),
            ProcessInfo(11, "Tracker_module_", "Tracker_module_debug", "/opt/Tracker_module_debug"),
        ]
        inspector = ProcessInspector()
        self.assertEqual([10], [item.pid for item in inspector.find_matches(program, processes)])

    def test_cmdline_tokens_are_all_required(self):
        program = ProgramConfig(
            name="bridge",
            group="plan",
            cmdline_contains=("planning-net-bridge", "bridge_node.py"),
        )
        processes = [
            ProcessInfo(20, "python3", "python3", "python3 /x/planning-net-bridge/bridge_node.py"),
            ProcessInfo(21, "python3", "python3", "python3 /other/bridge_node.py"),
        ]
        inspector = ProcessInspector()
        self.assertEqual([20], [item.pid for item in inspector.find_matches(program, processes)])

    def test_proc_reader_handles_long_executable_name(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            proc_root = Path(temp_dir)
            process_dir = proc_root / "123"
            process_dir.mkdir()
            (process_dir / "comm").write_text("getTargetInfoBy\n", encoding="utf-8")
            (process_dir / "cmdline").write_bytes(b"/tmp/getTargetInfoByRos\0--flag\0")
            executable = proc_root / "getTargetInfoByRos"
            executable.touch()
            (process_dir / "exe").symlink_to(executable)
            processes = ProcessInspector(proc_root).scan()
        self.assertEqual("getTargetInfoByRos", processes[0].exe_name)


if __name__ == "__main__":
    unittest.main()
