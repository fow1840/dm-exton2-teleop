"""Pink 3.5.0 offline IK comparison tests for T39; no ROS or robot access."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
MODULE_DIR = ROOT / "tools/dm-exo/urdf_fk_calib"
sys.path.insert(0, str(MODULE_DIR))

import compare_pink_ik


class T39PinkIkComparisonTests(unittest.TestCase):
    def test_pink_solves_same_w_generated_target_under_both_models(self):
        report = compare_pink_ik.compare_pink_ik(
            compare_pink_ik.candidate.DEFAULT_URDF,
            compare_pink_ik.candidate.DEFAULT_OUTPUT,
            [compare_pink_ik.candidate.SAMPLE_FILES[0]],
            max_iter=120,
        )
        self.assertEqual(report["summary"]["baseline_W"]["sample_count"], 1)
        self.assertEqual(report["summary"]["csv_candidate_C"]["sample_count"], 1)
        self.assertGreater(report["summary"]["baseline_W"]["converged_count"], report["summary"]["csv_candidate_C"]["converged_count"])
        self.assertIn("W FK", report["targets"])
        for model_name in ("baseline_W", "csv_candidate_C"):
            summary = report["summary"][model_name]
            self.assertGreaterEqual(summary["converged_count"], 0)
            self.assertIsNotNone(summary["median_final_position_mm"])
            self.assertIsNotNone(summary["median_final_rotation_deg"])
            for row in report["per_sample"][model_name]:
                self.assertGreaterEqual(row["iterations"], 1)
                self.assertGreaterEqual(row["final_position_mm"], 0.0)
                self.assertGreaterEqual(row["final_rotation_deg"], 0.0)

    def test_pink_ik_report_explicitly_says_synthetic_not_ground_truth(self):
        report = compare_pink_ik.compare_pink_ik(
            compare_pink_ik.candidate.DEFAULT_URDF,
            compare_pink_ik.candidate.DEFAULT_OUTPUT,
            [compare_pink_ik.candidate.SAMPLE_FILES[0]],
            max_iter=20,
        )
        self.assertIn("not measured TCP", report["targets"])
        self.assertIn("not establish", report["interpretation"])


if __name__ == "__main__":
    unittest.main()
