"""Reproducible offline checks for the T32 URDF/CSV audit."""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/dm-exo/urdf_fk_calib"))
import audit_urdf_csv as audit


class T32UrdfCsvAuditTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = audit.run_audit()

    def test_four_urdfs_match_on_all_14_arm_joint_kinematic_fields(self):
        self.assertEqual(self.result["arm_joint_count"], 14)
        self.assertTrue(all(not diffs for diffs in self.result["urdf_arm_joint_diffs_vs_W"].values()))

    def test_four_csv_copies_are_byte_identical(self):
        self.assertTrue(self.result["csv_copies_identical"])
        self.assertEqual(len(set(self.result["csv_sha256"].values())), 1)

    def test_csv_left_world_axis_sign_candidates(self):
        self.assertEqual(
            self.result["left_axis_dot_signs"],
            {
                "Joint1L": 1,
                "Joint2L": -1,
                "Joint3L": -1,
                "Joint4L": -1,
                "Joint5L": 1,
                "Joint6L": -1,
                "Joint7L": 1,
            },
        )

    def test_csv_right_world_axes_match_working_urdf(self):
        for dot in self.result["right_axis_dots"].values():
            self.assertGreater(dot, 0.999999)

    def test_l1_sample_is_not_the_zero_joint_pose(self):
        sample = self.result["sample_L-1"]["W_vs_C"]
        neutral = self.result["zero_joint_pose"]["W_vs_C"]
        self.assertAlmostEqual(sample["position_mm"], 1193.9109, places=3)
        self.assertAlmostEqual(sample["rotation_deg"], 167.9297, places=3)
        self.assertAlmostEqual(neutral["position_mm"], 1196.8, places=3)
        self.assertAlmostEqual(neutral["rotation_deg"], 179.9992, places=3)
        self.assertNotAlmostEqual(sample["position_mm"], neutral["position_mm"], places=2)

    def test_l1_sample_tcp_residuals_are_reproducible(self):
        sample = self.result["sample_L-1"]
        self.assertAlmostEqual(sample["W_vs_TCP"]["position_mm"], 512.8182, places=3)
        self.assertAlmostEqual(sample["W_vs_TCP"]["rotation_deg"], 64.2608, places=3)
        self.assertAlmostEqual(sample["C_vs_TCP"]["position_mm"], 925.1267, places=3)
        self.assertAlmostEqual(sample["C_vs_TCP"]["rotation_deg"], 147.4783, places=3)
        self.assertAlmostEqual(
            sample["W_vs_TCP_tool_minus_145mm"]["position_mm"], 600.8763, places=3
        )


if __name__ == "__main__":
    unittest.main()
