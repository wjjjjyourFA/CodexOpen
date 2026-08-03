import json
import tempfile
import unittest
from pathlib import Path

from common.config_loader import ConfigError, load_config, profile_config_path


PROJECT_DIR = Path(__file__).resolve().parent.parent


class ConfigLoaderTest(unittest.TestCase):
    def test_shell_configs_are_independent_and_valid(self):
        ros1_95 = load_config(PROJECT_DIR / "config" / "ros1_95.json")
        ros1_105 = load_config(PROJECT_DIR / "config" / "ros1_105.json")
        self.assertEqual("ros1_95", ros1_95.profile)
        self.assertEqual("ros1_105", ros1_105.profile)
        self.assertGreater(len(ros1_95.programs), 0)
        self.assertGreater(len(ros1_105.programs), 0)

    def test_rejects_program_without_match_rule(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "invalid.json"
            path.write_text(
                json.dumps({"profile": "test", "programs": [{"name": "bad"}]}),
                encoding="utf-8",
            )
            with self.assertRaises(ConfigError):
                load_config(path)

    def test_profile_cannot_escape_config_directory(self):
        with self.assertRaises(ConfigError):
            profile_config_path(PROJECT_DIR / "config", "../shell1")


if __name__ == "__main__":
    unittest.main()
