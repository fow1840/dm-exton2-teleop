"""Offline tests for the T31 read-only ROS audit collector.

The tests use a fake ros2 CLI and never connect to a ROS graph or robot.
"""
import os
from pathlib import Path
import stat
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools/dm-exo/t31_readonly_audit.sh"


class T31AuditScriptTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.call_log = self.root / "ros2-calls.txt"

        ros2 = self.bin / "ros2"
        ros2.write_text(
            "#!/usr/bin/env bash\n"
            'printf "%s\\n" "$*" >> "$T31_TEST_CALL_LOG"\n'
            'case " $* " in\n'
            '  *" topic hz "*) printf "average rate: 100.0\\n" ;;\n'
            '  *" topic echo "*) printf "header:\\n  frame_id: test_frame\\n" ;;\n'
            '  *) printf "fake ros2 output\\n" ;;\n'
            "esac\n"
        )
        ros2.chmod(0o755)

        pgrep = self.bin / "pgrep"
        pgrep.write_text("#!/usr/bin/env bash\nexit 1\n")
        pgrep.chmod(0o755)

        self.env = os.environ.copy()
        self.env.update({
            "PATH": str(self.bin) + os.pathsep + self.env.get("PATH", "/usr/bin:/bin"),
            "ROS_DOMAIN_ID": "42",
            "ROS_DISTRO": "humble",
            "T31_TEST_CALL_LOG": str(self.call_log),
            "T31_OBSERVATION": "clutch_released",
        })

    def test_shell_syntax(self):
        result = subprocess.run(
            ["bash", "-n", str(SCRIPT)],
            text=True, capture_output=True, timeout=5,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_missing_domain_id_fails_before_creating_capture(self):
        env = self.env.copy()
        env.pop("ROS_DOMAIN_ID")
        output = self.root / "must-not-exist"
        result = subprocess.run(
            ["bash", str(SCRIPT), str(output)],
            env=env, text=True, capture_output=True, timeout=5,
        )
        self.assertEqual(result.returncode, 2)
        self.assertFalse(output.exists())
        self.assertFalse(self.call_log.exists())
        self.assertIn("ROS_DOMAIN_ID", result.stderr)

    def test_collects_expected_read_only_command_set_with_private_permissions(self):
        output = self.root / "capture"
        result = subprocess.run(
            ["bash", str(SCRIPT), str(output)],
            env=self.env, text=True, capture_output=True, timeout=30,
        )
        self.assertEqual(result.returncode, 0, result.stderr + result.stdout)
        calls = self.call_log.read_text().splitlines()
        self.assertEqual(len(calls), 53)
        self.assertTrue(any("node list" in call for call in calls))
        self.assertTrue(any("param get" in call for call in calls))
        self.assertTrue(any("/target_robot/left_ee/target" in call for call in calls))
        self.assertTrue(any("/target_robot/right_ee/target" in call for call in calls))
        self.assertTrue(any("/client_robot/joint_states" in call for call in calls))
        self.assertTrue(any("/target_robot/joint_commands" in call for call in calls))
        self.assertTrue(all("movej" not in call for call in calls))
        self.assertTrue(all("stop_motion" not in call for call in calls))

        status = (output / "status.tsv").read_text().splitlines()
        self.assertEqual(len(status), 54)  # header + 53 bounded reads
        self.assertTrue(all(line.endswith("\t0") for line in status[1:]))
        self.assertTrue((output / "context.txt").exists())
        self.assertEqual(len(list(output.glob("*.txt"))), 54)  # context + 53 captures
        self.assertEqual(stat.S_IMODE(output.stat().st_mode), 0o700)
        self.assertEqual(stat.S_IMODE((output / "status.tsv").stat().st_mode), 0o600)
        self.assertIn("ROS_DOMAIN_ID=42", (output / "context.txt").read_text())
        self.assertIn("observed_condition=clutch_released", (output / "context.txt").read_text())


if __name__ == "__main__":
    unittest.main()
