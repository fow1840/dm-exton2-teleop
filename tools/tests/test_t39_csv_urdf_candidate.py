"""Offline regression tests for T39 CSV-derived left-arm URDF candidates."""
import csv
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
MODULE_DIR = ROOT / "tools/dm-exo/urdf_fk_calib"
import sys
sys.path.insert(0, str(MODULE_DIR))

import audit_urdf_csv as audit
import csv_urdf_candidate as candidate


class T39CsvUrdfCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = candidate.DEFAULT_URDF
        cls.csv_path = candidate.DEFAULT_CSV
        cls.source_sha = candidate._sha256(cls.source)
        cls.temp = tempfile.TemporaryDirectory()
        cls.temp_path = Path(cls.temp.name)
        cls.candidate_path = cls.temp_path / "left-candidate-NOT_FOR_ROBOT.urdf"
        cls.generated = candidate.generate_candidate(
            cls.source, cls.csv_path, cls.candidate_path, link_frames="preserve"
        )

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def _write_modified_csv(self, mutate, filename):
        with self.csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream)
            fieldnames = reader.fieldnames
            rows = list(reader)
        mutate(rows)
        path = self.temp_path / filename
        with path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
        return path

    def test_candidate_is_deterministic_and_never_changes_source(self):
        second = self.temp_path / "second-candidate-NOT_FOR_ROBOT.urdf"
        candidate.generate_candidate(self.source, self.csv_path, second)
        self.assertEqual(self.candidate_path.read_bytes(), second.read_bytes())
        self.assertEqual(candidate._sha256(self.source), self.source_sha)
        self.assertFalse(self.generated["robot_deployment_allowed"])

    def test_only_seven_left_joint_origins_and_axes_are_replaced(self):
        baseline = audit.parse_urdf(self.source)
        generated = audit.parse_urdf(self.candidate_path)
        csv_joints, _ = candidate._read_csv_rows(self.csv_path)
        self.assertEqual(set(candidate.LEFT_JOINTS), set(name for name in generated if name.endswith("L")))
        for name in baseline:
            if name in candidate.LEFT_JOINTS:
                expected = candidate._csv_joint_values(csv_joints[name], name)
                for field in ("xyz", "rpy", "axis"):
                    for actual, wanted in zip(generated[name][field], expected[field]):
                        self.assertAlmostEqual(actual, wanted, places=10)
            else:
                self.assertEqual(generated[name], baseline[name])

    def test_all_left_limits_are_preserved_not_imported_from_zero_csv_fields(self):
        baseline = audit.parse_urdf(self.source)
        generated = audit.parse_urdf(self.candidate_path)
        csv_joints, _ = candidate._read_csv_rows(self.csv_path)
        for name in candidate.LEFT_JOINTS:
            self.assertEqual(generated[name]["limit"], baseline[name]["limit"])
            self.assertTrue(all(float(csv_joints[name]["Limit " + key.capitalize()] or 0) == 0 for key in candidate.LIMIT_FIELDS))

    def test_default_candidate_preserves_link_geometry_and_inertial_frames(self):
        baseline_root = ET.parse(self.source).getroot()
        candidate_root = ET.parse(self.candidate_path).getroot()
        baseline_links = {x.attrib["name"]: x for x in baseline_root.findall("link")}
        candidate_links = {x.attrib["name"]: x for x in candidate_root.findall("link")}
        for name in candidate.LEFT_LINKS:
            self.assertEqual(
                ET.tostring(baseline_links[name], encoding="unicode"),
                ET.tostring(candidate_links[name], encoding="unicode"),
            )

    def test_optional_csv_visual_frame_variant_changes_only_visual_collision_origins(self):
        path = self.temp_path / "left-visual-candidate-NOT_FOR_ROBOT.urdf"
        candidate.generate_candidate(self.source, self.csv_path, path, link_frames="csv")
        baseline_root = ET.parse(self.source).getroot()
        candidate_root = ET.parse(path).getroot()
        baseline_links = {x.attrib["name"]: x for x in baseline_root.findall("link")}
        candidate_links = {x.attrib["name"]: x for x in candidate_root.findall("link")}
        for name in candidate.LEFT_LINKS:
            before, after = baseline_links[name], candidate_links[name]
            self.assertEqual(ET.tostring(before.find("inertial")), ET.tostring(after.find("inertial")))
            for tag in ("visual", "collision"):
                self.assertEqual(
                    before.find(tag).find("geometry").find("mesh").attrib,
                    after.find(tag).find("geometry").find("mesh").attrib,
                )
        self.assertEqual(candidate._sha256(self.source), self.source_sha)

    def test_candidate_xml_is_parseable_and_mount_is_unchanged(self):
        ET.parse(self.candidate_path)
        baseline = audit.parse_urdf(self.source)
        generated = audit.parse_urdf(self.candidate_path)
        self.assertEqual(generated["base_link_to_base"], baseline["base_link_to_base"])

    def test_rejects_output_path_that_is_not_explicitly_non_deployable(self):
        path = self.temp_path / "robot.urdf"
        with self.assertRaisesRegex(ValueError, "NOT_FOR_ROBOT"):
            candidate.generate_candidate(self.source, self.csv_path, path)

    def test_rejects_missing_left_joint_row(self):
        def mutate(rows):
            rows[:] = [row for row in rows if row["Joint Name"] != "Joint3L"]
        bad_csv = self._write_modified_csv(mutate, "missing-joint.csv")
        with self.assertRaisesRegex(ValueError, "missing left-arm joints"):
            candidate.generate_candidate(self.source, bad_csv, self.temp_path / "missing-candidate-NOT_FOR_ROBOT.urdf")

    def test_rejects_duplicate_joint_or_link_row(self):
        def mutate(rows):
            rows.append(dict(next(row for row in rows if row["Joint Name"] == "Joint1L")))
        bad_csv = self._write_modified_csv(mutate, "duplicate-joint.csv")
        with self.assertRaisesRegex(ValueError, "duplicate CSV"):
            candidate.generate_candidate(self.source, bad_csv, self.temp_path / "duplicate-candidate-NOT_FOR_ROBOT.urdf")

    def test_rejects_parent_child_mismatch(self):
        def mutate(rows):
            next(row for row in rows if row["Joint Name"] == "Joint4L")["Parent"] = "WrongParent"
        bad_csv = self._write_modified_csv(mutate, "wrong-parent.csv")
        with self.assertRaisesRegex(ValueError, "parent mismatch"):
            candidate.generate_candidate(self.source, bad_csv, self.temp_path / "parent-candidate-NOT_FOR_ROBOT.urdf")

    def test_rejects_zero_axis(self):
        def mutate(rows):
            row = next(row for row in rows if row["Joint Name"] == "Joint2L")
            row["Joint Axis X"] = "0"
            row["Joint Axis Y"] = "0"
            row["Joint Axis Z"] = "0"
        bad_csv = self._write_modified_csv(mutate, "zero-axis.csv")
        with self.assertRaisesRegex(ValueError, "zero joint axis"):
            candidate.generate_candidate(self.source, bad_csv, self.temp_path / "axis-candidate-NOT_FOR_ROBOT.urdf")

    def test_rejects_non_finite_joint_origin(self):
        def mutate(rows):
            next(row for row in rows if row["Joint Name"] == "Joint2L")["Joint Origin X"] = "nan"
        bad_csv = self._write_modified_csv(mutate, "nan-origin.csv")
        with self.assertRaisesRegex(ValueError, "non-finite"):
            candidate.generate_candidate(self.source, bad_csv, self.temp_path / "nan-candidate-NOT_FOR_ROBOT.urdf")

    def test_historical_comparison_has_seven_unique_poses_and_reproducible_pair_counts(self):
        report = candidate.compare_models(
            self.source, self.candidate_path, self.csv_path, list(candidate.SAMPLE_FILES)
        )
        self.assertEqual(report["sample_counts"]["raw_sample_count"], 11)
        self.assertEqual(report["sample_counts"]["unique_pose_count"], 7)
        all_pairs = report["pairwise_relative_rotation_angle_error_deg"]["all_unique_poses"]
        without_l01 = report["pairwise_relative_rotation_angle_error_deg"]["excluding_L01"]
        self.assertEqual(all_pairs["baseline_W"]["count"], 21)
        self.assertEqual(all_pairs["csv_candidate_C"]["count"], 21)
        self.assertEqual(without_l01["baseline_W"]["count"], 15)
        self.assertEqual(without_l01["csv_candidate_C"]["count"], 15)
        self.assertAlmostEqual(all_pairs["baseline_W"]["median"], 23.94, delta=0.05)
        self.assertAlmostEqual(all_pairs["baseline_W"]["max"], 140.37, delta=0.05)
        self.assertAlmostEqual(all_pairs["csv_candidate_C"]["median"], 29.52, delta=0.05)
        self.assertAlmostEqual(all_pairs["csv_candidate_C"]["max"], 75.80, delta=0.05)
        self.assertAlmostEqual(without_l01["csv_candidate_C"]["median"], 20.10, delta=0.05)
        self.assertAlmostEqual(without_l01["csv_candidate_C"]["max"], 75.80, delta=0.05)
        self.assertTrue(all(report["structural_checks"].values()))

    def test_report_marks_controller_tcp_and_frame_assumptions_unverified(self):
        report = candidate.compare_models(
            self.source, self.candidate_path, self.csv_path, list(candidate.SAMPLE_FILES)
        )
        assumptions = report["frame_and_data_assumptions"]
        self.assertIn("unverified", assumptions["tcp_assumption"])
        self.assertIn("not independent", assumptions["feedback_assumption"])
        self.assertFalse(report["structural_checks"]["robot_deployment_allowed"])


if __name__ == "__main__":
    unittest.main()
