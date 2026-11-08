#!/usr/bin/env python3
"""Generate and compare a non-deployable CSV-derived left-arm URDF candidate.

Standard-library implementation. Never connects to a robot and never writes the
production URDF. The default candidate changes only Joint1L..Joint7L origin/axis.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET

import audit_urdf_csv as audit

ROOT = Path(__file__).resolve().parents[3]
DEFAULT_URDF = ROOT / "ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf"
DEFAULT_CSV = ROOT / "ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.csv"
SAMPLE_FILES = (
    ROOT / "tools/dm-exo/urdf_fk_calib/data/calib_samples_L_1106_0943.jsonl",
    ROOT / "tools/dm-exo/urdf_fk_calib/data/calib_L_drive.jsonl",
    ROOT / "tools/dm-exo/urdf_fk_calib/data/calib_L2_drive.jsonl",
)
CANDIDATE_DIR = ROOT / "tools/dm-exo/urdf_fk_calib/candidates"
DEFAULT_OUTPUT = CANDIDATE_DIR / "HR-UP-7DOF-3KG-R-B01382-CSV-left-kinematics-candidate-NOT_FOR_ROBOT.urdf"
DEFAULT_REPORT = CANDIDATE_DIR / "HR-UP-7DOF-3KG-R-B01382-CSV-left-candidate-report.json"
LEFT_JOINTS = tuple("Joint{}L".format(i) for i in range(1, 8))
LEFT_LINKS = tuple("Link{}L".format(i) for i in range(1, 8))
RIGHT_JOINTS = tuple("Joint{}".format(i) for i in range(1, 8))
LIMIT_FIELDS = ("lower", "upper", "velocity", "effort")
ORIGIN_FIELDS = ("X", "Y", "Z")
RPY_FIELDS = ("Roll", "Pitch", "Yaw")


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _finite_vector(values, label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if not all(math.isfinite(value) for value in result):
        raise ValueError("{} contains a non-finite value".format(label))
    return result


def _read_csv_rows(path: Path) -> tuple[dict[str, dict], dict[str, dict]]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {
            "Link Name", "Joint Name", "Joint Type", "Parent",
            "Joint Origin X", "Joint Origin Y", "Joint Origin Z",
            "Joint Origin Roll", "Joint Origin Pitch", "Joint Origin Yaw",
            "Joint Axis X", "Joint Axis Y", "Joint Axis Z",
        }
        missing = required.difference(reader.fieldnames or ())
        if missing:
            raise ValueError("CSV missing required columns: {}".format(sorted(missing)))
        joint_rows: dict[str, dict] = {}
        link_rows: dict[str, dict] = {}
        for line_number, row in enumerate(reader, start=2):
            link_name = (row.get("Link Name") or "").strip()
            joint_name = (row.get("Joint Name") or "").strip()
            if link_name:
                if link_name in link_rows:
                    raise ValueError("duplicate CSV link row {} at line {}".format(link_name, line_number))
                link_rows[link_name] = row
            if joint_name:
                if joint_name in joint_rows:
                    raise ValueError("duplicate CSV joint row {} at line {}".format(joint_name, line_number))
                joint_rows[joint_name] = row
    missing_joints = sorted(set(LEFT_JOINTS).difference(joint_rows))
    if missing_joints:
        raise ValueError("CSV missing left-arm joints: {}".format(missing_joints))
    return joint_rows, link_rows


def _csv_joint_values(row: dict, name: str) -> dict:
    xyz = _finite_vector((row["Joint Origin " + field] for field in ORIGIN_FIELDS), name + " origin xyz")
    rpy = _finite_vector((row["Joint Origin " + field] for field in RPY_FIELDS), name + " origin rpy")
    axis = _finite_vector((row["Joint Axis " + field] for field in ORIGIN_FIELDS), name + " axis")
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= 1e-12:
        raise ValueError("{} has a zero joint axis".format(name))
    normalized_axis = tuple(value / norm for value in axis)
    return {
        "type": (row.get("Joint Type") or "").strip(),
        "parent": (row.get("Parent") or "").strip(),
        "child": (row.get("Link Name") or "").strip(),
        "xyz": xyz,
        "rpy": rpy,
        "axis": normalized_axis,
        "csv_limits": {
            key: (float(row.get("Limit " + key.capitalize()) or 0.0))
            for key in LIMIT_FIELDS
        },
    }


def _format_vector(values) -> str:
    return " ".join("{:.12g}".format(float(value)) for value in values)


def _set_origin(element: ET.Element, xyz, rpy) -> None:
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    origin.set("xyz", _format_vector(xyz))
    origin.set("rpy", _format_vector(rpy))


def _validate_source_and_csv(source_urdf: Path, csv_path: Path):
    if not source_urdf.is_file():
        raise FileNotFoundError(source_urdf)
    if not csv_path.is_file():
        raise FileNotFoundError(csv_path)
    source_joints = audit.parse_urdf(source_urdf)
    csv_joints, csv_links = _read_csv_rows(csv_path)
    normalized = {}
    for name in LEFT_JOINTS:
        source = source_joints.get(name)
        if source is None:
            raise ValueError("source URDF missing {}".format(name))
        candidate = _csv_joint_values(csv_joints[name], name)
        for field in ("type", "parent", "child"):
            if candidate[field] != source[field]:
                raise ValueError(
                    "{} {} mismatch: source={!r}, CSV={!r}".format(
                        name, field, source[field], candidate[field]
                    )
                )
        if candidate["type"] != "revolute":
            raise ValueError("{} must be revolute".format(name))
        normalized[name] = candidate
    missing_links = sorted(set(LEFT_LINKS).difference(csv_links))
    if missing_links:
        raise ValueError("CSV missing left-arm link rows: {}".format(missing_links))
    return source_joints, normalized, csv_links


def _apply_csv_visual_frames(root: ET.Element, csv_links: dict[str, dict]) -> None:
    """Optional display-only experiment; mesh URIs and inertial data stay from W."""
    link_elements = {element.attrib["name"]: element for element in root.findall("link")}
    for name in LEFT_LINKS:
        if name not in link_elements:
            raise ValueError("source URDF missing link {}".format(name))
        row = csv_links[name]
        link = link_elements[name]
        for tag, prefix in (("visual", "Visual"), ("collision", "Collision")):
            elements = link.findall(tag)
            if len(elements) != 1:
                raise ValueError("{} must contain exactly one {} element".format(name, tag))
            xyz = _finite_vector((row[prefix + " " + field] for field in ORIGIN_FIELDS), name + " " + tag + " xyz")
            rpy = _finite_vector((row[prefix + " " + field] for field in RPY_FIELDS), name + " " + tag + " rpy")
            _set_origin(elements[0], xyz, rpy)


def generate_candidate(
    source_urdf: Path,
    csv_path: Path,
    output_path: Path,
    link_frames: str = "preserve",
) -> dict:
    """Generate a separate diagnostic model; the output name is safety-gated."""
    source_urdf = Path(source_urdf)
    csv_path = Path(csv_path)
    output_path = Path(output_path)
    if "candidate" not in output_path.name.lower() or "not_for_robot" not in output_path.name.lower():
        raise ValueError("output filename must contain both 'candidate' and 'NOT_FOR_ROBOT'")
    if source_urdf.resolve() == output_path.resolve():
        raise ValueError("refusing to overwrite the source URDF")
    if link_frames not in ("preserve", "csv"):
        raise ValueError("link_frames must be 'preserve' or 'csv'")

    source_joints, csv_joints, csv_links = _validate_source_and_csv(source_urdf, csv_path)
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source_urdf, parser=parser)
    root = tree.getroot()
    joint_elements = {element.attrib["name"]: element for element in root.findall("joint")}

    changes = []
    for name in LEFT_JOINTS:
        element = joint_elements.get(name)
        if element is None:
            raise ValueError("source URDF XML missing {}".format(name))
        values = csv_joints[name]
        _set_origin(element, values["xyz"], values["rpy"])
        axis = element.find("axis")
        if axis is None:
            axis = ET.SubElement(element, "axis")
        axis.set("xyz", _format_vector(values["axis"]))
        # Deliberately do not import CSV limits: the export contains zero limits.
        changes.append({
            "joint": name,
            "origin_xyz": list(values["xyz"]),
            "origin_rpy": list(values["rpy"]),
            "axis_local": list(values["axis"]),
            "source_limits_preserved": dict(source_joints[name]["limit"]),
            "csv_limits_ignored": values["csv_limits"],
        })

    if link_frames == "csv":
        _apply_csv_visual_frames(root, csv_links)

    root.attrib["name"] = root.attrib.get("name", "robot") + "_CSV_LEFT_CANDIDATE_NOT_FOR_ROBOT"
    root.insert(0, ET.Comment(
        " DIAGNOSTIC ONLY - CSV-derived left-arm candidate; NEVER deploy to a robot. "
        "Generated by csv_urdf_candidate.py. "
    ))
    ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)

    generated = audit.parse_urdf(output_path)
    for name in LEFT_JOINTS:
        expected = csv_joints[name]
        actual = generated[name]
        if any(abs(a - b) > 1e-10 for a, b in zip(actual["xyz"], expected["xyz"])):
            raise AssertionError("{} origin xyz generation mismatch".format(name))
        if any(abs(a - b) > 1e-10 for a, b in zip(actual["rpy"], expected["rpy"])):
            raise AssertionError("{} origin rpy generation mismatch".format(name))
        if any(abs(a - b) > 1e-10 for a, b in zip(actual["axis"], expected["axis"])):
            raise AssertionError("{} axis generation mismatch".format(name))
        if actual["limit"] != source_joints[name]["limit"]:
            raise AssertionError("{} limits were not preserved".format(name))

    return {
        "output": str(output_path),
        "output_sha256": _sha256(output_path),
        "source_urdf_sha256": _sha256(source_urdf),
        "source_csv_sha256": _sha256(csv_path),
        "link_frames": link_frames,
        "updated_left_joints": changes,
        "right_arm_and_fixed_mount": "preserved from source URDF",
        "limits": "preserved from source URDF; CSV zero limits ignored",
        "robot_deployment_allowed": False,
    }


def _load_samples(sample_paths: list[Path]) -> tuple[list[dict], dict]:
    raw = []
    files = {}
    for path in sample_paths:
        if not path.is_file():
            raise FileNotFoundError(path)
        count = 0
        for line_number, line in enumerate(path.read_text(encoding="utf-8").splitlines(), start=1):
            if not line.strip():
                continue
            row = json.loads(line)
            if len(row.get("q17", [])) != 17 or len(row.get("tcp14", [])) != 14:
                raise ValueError("{}:{} must contain q17[17] and tcp14[14]".format(path, line_number))
            q = _finite_vector(row["q17"][1:8], "{}:{} left q".format(path, line_number))
            tcp_position = _finite_vector(row["tcp14"][0:3], "{}:{} tcp position".format(path, line_number))
            quaternion = _finite_vector(row["tcp14"][3:7], "{}:{} tcp quaternion".format(path, line_number))
            audit.quaternion_matrix_wxyz(list(quaternion))
            raw.append({
                "note": str(row.get("note", "")),
                "source": str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                "q_left": q,
                "tcp_position": tcp_position,
                "tcp_quaternion_wxyz": quaternion,
            })
            count += 1
        files[str(path)] = count
    unique = []
    for row in raw:
        if not any(
            max(abs(a - b) for a, b in zip(row["q_left"], kept["q_left"])) <= 1e-3
            for kept in unique
        ):
            unique.append(row)
    return unique, {"raw_sample_count": len(raw), "unique_pose_count": len(unique), "sample_counts_by_file": files}


def _pose_for_sample(joints: dict, mount: dict, sample: dict) -> dict:
    q = dict(zip(LEFT_JOINTS, sample["q_left"]))
    flange = audit.fk(joints, q, "base", "Link7L")
    return audit.apply_mount(flange, mount)


def _summary(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "median": None, "max": None}
    return {
        "count": len(values),
        "median": statistics.median(values),
        "max": max(values),
    }


def _relative_rotation_angle(a: dict, b: dict) -> float:
    return audit.rotation_error_deg(a["R"], b["R"])


def _pairwise_rotation_errors(model_poses: list[dict], tcp_poses: list[dict]) -> dict:
    errors = []
    for i in range(len(model_poses)):
        for j in range(i + 1, len(model_poses)):
            model_delta = _relative_rotation_angle(model_poses[i], model_poses[j])
            tcp_delta = _relative_rotation_angle(tcp_poses[i], tcp_poses[j])
            errors.append(abs(model_delta - tcp_delta))
    return _summary(errors)


def _direct_residuals(model_poses: list[dict], tcp_poses: list[dict]) -> dict:
    positions = []
    rotations = []
    for model, tcp in zip(model_poses, tcp_poses):
        delta = audit.pose_difference(model, tcp)
        positions.append(delta["position_mm"])
        rotations.append(delta["rotation_deg"])
    return {
        "position_mm": _summary(positions),
        "rotation_deg": _summary(rotations),
    }


def compare_models(
    source_urdf: Path,
    candidate_urdf: Path,
    csv_path: Path,
    sample_paths: list[Path],
    link_frames: str = "preserve",
) -> dict:
    baseline = audit.parse_urdf(source_urdf)
    candidate = audit.parse_urdf(candidate_urdf)
    mount = baseline.get("base_link_to_base")
    if mount is None or mount["type"] != "fixed":
        raise ValueError("source URDF must contain fixed base_link_to_base mount")
    if candidate.get("base_link_to_base") != mount:
        raise ValueError("candidate changed the fixed base mount")

    limits_preserved = all(candidate[name]["limit"] == baseline[name]["limit"] for name in LEFT_JOINTS)
    right_unchanged = all(candidate[name] == baseline[name] for name in RIGHT_JOINTS)
    fixed_mount_unchanged = candidate["base_link_to_base"] == baseline["base_link_to_base"]

    baseline_axes = audit.world_axes(baseline, mount, "left")
    candidate_axes = audit.world_axes(candidate, mount, "left")
    csv_joints, _ = _read_csv_rows(csv_path)
    axis_rows = []
    origin_rows = []
    for name in LEFT_JOINTS:
        row = _csv_joint_values(csv_joints[name], name)
        dot = sum(baseline_axes[name][i] * candidate_axes[name][i] for i in range(3))
        axis_rows.append({
            "joint": name,
            "baseline_world_axis": baseline_axes[name],
            "candidate_world_axis": candidate_axes[name],
            "baseline_vs_candidate_world_axis_dot": dot,
            "candidate_local_axis": candidate[name]["axis"],
        })
        origin_rows.append({
            "joint": name,
            "xyz_delta_m": [candidate[name]["xyz"][i] - baseline[name]["xyz"][i] for i in range(3)],
            "rpy_delta_rad": [candidate[name]["rpy"][i] - baseline[name]["rpy"][i] for i in range(3)],
            "candidate_matches_csv_origin": (
                all(abs(candidate[name]["xyz"][i] - row["xyz"][i]) <= 1e-10 for i in range(3))
                and all(abs(candidate[name]["rpy"][i] - row["rpy"][i]) <= 1e-10 for i in range(3))
            ),
        })

    samples, sample_counts = _load_samples(sample_paths)
    tcp_poses = [
        {
            "p": list(sample["tcp_position"]),
            "R": audit.quaternion_matrix_wxyz(list(sample["tcp_quaternion_wxyz"])),
        }
        for sample in samples
    ]
    baseline_poses = [_pose_for_sample(baseline, mount, sample) for sample in samples]
    candidate_poses = [_pose_for_sample(candidate, mount, sample) for sample in samples]

    keep_indices = [i for i, sample in enumerate(samples) if sample["note"] != "L01"]
    baseline_without_l01 = [baseline_poses[i] for i in keep_indices]
    candidate_without_l01 = [candidate_poses[i] for i in keep_indices]
    tcp_without_l01 = [tcp_poses[i] for i in keep_indices]

    return {
        "schema_version": 1,
        "baseline_urdf_sha256": _sha256(Path(source_urdf)),
        "candidate_urdf_sha256": _sha256(Path(candidate_urdf)),
        "source_csv_sha256": _sha256(Path(csv_path)),
        "candidate_link_frames_mode": link_frames,
        "sample_counts": sample_counts,
        "frame_and_data_assumptions": {
            "q17_order": "waist, left 7 joints, right 7 joints, head 2 joints",
            "left_joint_slice": "q17[1:8]",
            "tcp14_order": "left TCP 7D then right TCP 7D; left is xyz + qw,qx,qy,qz",
            "base_assumption": "controller base is the source URDF base_link mounted through base_link_to_base",
            "tcp_assumption": "Link7L is treated as controller TCP; this is unverified",
            "feedback_assumption": "historical tcp14 is controller-reported pose, not independent external metrology",
            "units": "joint angles rad; TCP position m; output position residual mm",
            "pairwise_rotation_metric": "relative rotation-angle differences; invariant to a fixed base rotation and fixed tool rotation",
        },
        "structural_checks": {
            "left_limits_preserved": limits_preserved,
            "right_arm_unchanged": right_unchanged,
            "fixed_base_mount_unchanged": fixed_mount_unchanged,
            "left_origins_match_csv": all(row["candidate_matches_csv_origin"] for row in origin_rows),
            "robot_deployment_allowed": False,
        },
        "left_joint_world_axes": axis_rows,
        "left_joint_origin_deltas": origin_rows,
        "residuals_to_controller_tcp_under_unverified_absolute_frames": {
            "baseline_W": _direct_residuals(baseline_poses, tcp_poses),
            "csv_candidate_C": _direct_residuals(candidate_poses, tcp_poses),
        },
        "pairwise_relative_rotation_angle_error_deg": {
            "all_unique_poses": {
                "baseline_W": _pairwise_rotation_errors(baseline_poses, tcp_poses),
                "csv_candidate_C": _pairwise_rotation_errors(candidate_poses, tcp_poses),
            },
            "excluding_L01": {
                "baseline_W": _pairwise_rotation_errors(baseline_without_l01, tcp_without_l01),
                "csv_candidate_C": _pairwise_rotation_errors(candidate_without_l01, tcp_without_l01),
            },
        },
        "interpretation": (
            "CSV candidate is a CAD-derived hypothesis only. Lower offline residuals do not prove "
            "real-robot direction or geometry; T31 frame/feedback verification and T34 physical "
            "direction validation remain mandatory."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument(
        "--link-frames", choices=("preserve", "csv"), default="preserve",
        help="preserve W visual/collision frames (default), or create a separate CSV visual-frame diagnostic variant",
    )
    parser.add_argument("--samples", nargs="*", type=Path, default=list(SAMPLE_FILES))
    args = parser.parse_args()

    generated = generate_candidate(args.urdf, args.csv, args.output, args.link_frames)
    report = compare_models(args.urdf, args.output, args.csv, args.samples, args.link_frames)
    report["generation"] = generated
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
