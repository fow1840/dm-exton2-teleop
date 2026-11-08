#!/usr/bin/env python3
"""Read-only URDF/CSV kinematic audit for the HR-UP-7DOF-3KG-R-B01382 arm.

Uses only the Python standard library. It never writes model files or connects to a robot.
"""
from __future__ import annotations

import csv
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
URDF_PATHS = {
    "W": ROOT / "ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf",
    "P": ROOT / "DM-EXton2ProductPackage_tianchuang/URDF/HR-UP-7DOF-3KG-R-B01382-URDF-1029-fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf",
    "D": ROOT / "DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/src/download/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf",
    "E": ROOT / "DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.urdf",
}
CSV_PATHS = {
    "W": ROOT / "ros2_ws/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.csv",
    "P": ROOT / "DM-EXton2ProductPackage_tianchuang/URDF/HR-UP-7DOF-3KG-R-B01382-URDF-1029-fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.csv",
    "D": ROOT / "DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/src/download/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.csv",
    "E": ROOT / "DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo/src/hr_up_7dof_fixed/urdf/HR-UP-7DOF-3KG-R-B01382-URDF-1029.csv",
}
SAMPLE_PATH = ROOT / "tools/dm-exo/urdf_fk_calib/data/calib_samples_L_1106_0943.jsonl"
RIGHT_JOINTS = tuple("Joint{}".format(i) for i in range(1, 8))
LEFT_JOINTS = tuple("Joint{}L".format(i) for i in range(1, 8))
ARM_JOINTS = RIGHT_JOINTS + LEFT_JOINTS
TOL = 1e-9


def _numbers(raw: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if raw is None or not raw.strip():
        return default
    return tuple(float(part) for part in raw.replace(",", " ").split())


def parse_urdf(path: Path) -> dict[str, dict]:
    root = ET.parse(path).getroot()
    joints: dict[str, dict] = {}
    for element in root.findall("joint"):
        origin = element.find("origin")
        axis = element.find("axis")
        limit = element.find("limit")
        parent = element.find("parent")
        child = element.find("child")
        name = element.attrib["name"]
        joints[name] = {
            "name": name,
            "type": element.attrib["type"],
            "parent": parent.attrib["link"],
            "child": child.attrib["link"],
            "xyz": _numbers(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0)),
            "rpy": _numbers(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0)),
            "axis": _numbers(axis.attrib.get("xyz") if axis is not None else None, (0.0, 0.0, 1.0)),
            "limit": {
                key: float(limit.attrib[key]) if limit is not None and key in limit.attrib else None
                for key in ("lower", "upper", "velocity", "effort")
            },
        }
    return joints


def parse_csv(path: Path) -> dict[str, dict]:
    joints: dict[str, dict] = {}
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            name = (row.get("Joint Name") or "").strip()
            if not name:
                continue
            joints[name] = {
                "name": name,
                "type": (row.get("Joint Type") or "").strip(),
                "parent": (row.get("Parent") or "").strip(),
                "child": (row.get("Link Name") or "").strip(),
                "xyz": tuple(float(row["Joint Origin " + axis]) for axis in ("X", "Y", "Z")),
                "rpy": tuple(float(row["Joint Origin " + axis]) for axis in ("Roll", "Pitch", "Yaw")),
                "axis": tuple(float(row["Joint Axis " + axis]) for axis in ("X", "Y", "Z")),
                "limit": {
                    key: float(row["Limit " + key.capitalize()] or 0.0)
                    for key in ("effort", "velocity", "lower", "upper")
                },
            }
    return joints


def _identity() -> list[list[float]]:
    return [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def _matmul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def _matvec(a: list[list[float]], v: tuple[float, float, float] | list[float]) -> list[float]:
    return [sum(a[i][k] * v[k] for k in range(3)) for i in range(3)]


def _add(a: list[float], b: list[float]) -> list[float]:
    return [a[i] + b[i] for i in range(3)]


def _transpose(a: list[list[float]]) -> list[list[float]]:
    return [[a[j][i] for j in range(3)] for i in range(3)]


def rpy_matrix(rpy: tuple[float, ...]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def axis_matrix(axis: tuple[float, ...], angle: float) -> list[list[float]]:
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= 1e-12:
        raise ValueError("revolute joint has a zero axis")
    x, y, z = (value / norm for value in axis)
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    return [
        [x * x * v + c, x * y * v - z * s, x * z * v + y * s],
        [x * y * v + z * s, y * y * v + c, y * z * v - x * s],
        [x * z * v - y * s, y * z * v + x * s, z * z * v + c],
    ]


def _chain(joints: dict[str, dict], root: str, target: str) -> list[dict]:
    current = target
    chain: list[dict] = []
    while current != root:
        parents = [joint for joint in joints.values() if joint["child"] == current]
        if len(parents) != 1:
            raise ValueError("expected one parent joint for {}, got {}".format(current, len(parents)))
        joint = parents[0]
        chain.append(joint)
        current = joint["parent"]
    return list(reversed(chain))


def fk(joints: dict[str, dict], q: dict[str, float], root: str, target: str) -> dict:
    rotation = _identity()
    position = [0.0, 0.0, 0.0]
    for joint in _chain(joints, root, target):
        position = _add(position, _matvec(rotation, joint["xyz"]))
        rotation = _matmul(rotation, rpy_matrix(joint["rpy"]))
        if joint["type"] == "revolute":
            rotation = _matmul(rotation, axis_matrix(joint["axis"], q.get(joint["name"], 0.0)))
    return {"R": rotation, "p": position}


def world_axes(joints: dict[str, dict], mount: dict, side: str) -> dict[str, list[float]]:
    names = LEFT_JOINTS if side == "left" else RIGHT_JOINTS
    rotation = _identity()
    axes: dict[str, list[float]] = {}
    for joint in _chain(joints, "base", "Link7L" if side == "left" else "Link7"):
        rotation = _matmul(rotation, rpy_matrix(joint["rpy"]))
        if joint["type"] == "revolute":
            axis_in_base = _matvec(rotation, joint["axis"])
            axes[joint["name"]] = _matvec(rpy_matrix(mount["rpy"]), axis_in_base)
            rotation = _matmul(rotation, axis_matrix(joint["axis"], 0.0))
    if set(axes) != set(names):
        raise ValueError("unexpected {} arm joint chain: {}".format(side, sorted(axes)))
    return axes


def apply_mount(pose: dict, mount: dict) -> dict:
    mount_rotation = rpy_matrix(mount["rpy"])
    return {
        "R": _matmul(mount_rotation, pose["R"]),
        "p": _add(list(mount["xyz"]), _matvec(mount_rotation, pose["p"])),
    }


def quaternion_matrix_wxyz(quaternion: list[float]) -> list[list[float]]:
    if len(quaternion) != 4:
        raise ValueError("expected qw,qx,qy,qz")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12:
        raise ValueError("zero quaternion")
    w, x, y, z = (value / norm for value in quaternion)
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def rotation_error_deg(a: list[list[float]], b: list[list[float]]) -> float:
    relative = _matmul(a, _transpose(b))
    cosine = max(-1.0, min(1.0, (sum(relative[i][i] for i in range(3)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def pose_difference(a: dict, b: dict) -> dict[str, float]:
    position_mm = math.sqrt(sum((a["p"][i] - b["p"][i]) ** 2 for i in range(3))) * 1000.0
    return {
        "position_mm": position_mm,
        "rotation_deg": rotation_error_deg(a["R"], b["R"]),
    }


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_audit() -> dict:
    for path in list(URDF_PATHS.values()) + list(CSV_PATHS.values()) + [SAMPLE_PATH]:
        if not path.is_file():
            raise FileNotFoundError(path)

    urdfs = {name: parse_urdf(path) for name, path in URDF_PATHS.items()}
    csvs = {name: parse_csv(path) for name, path in CSV_PATHS.items()}
    base = urdfs["W"]
    mount = base["base_link_to_base"]

    urdf_diffs: dict[str, list[str]] = {}
    for label, model in urdfs.items():
        diffs = []
        for name in ARM_JOINTS:
            a, b = base[name], model.get(name)
            if b is None:
                diffs.append(name + ":missing")
                continue
            for key in ("type", "parent", "child"):
                if a[key] != b[key]:
                    diffs.append(name + ":" + key)
            for key in ("xyz", "rpy", "axis"):
                if any(abs(x - y) > TOL for x, y in zip(a[key], b[key])):
                    diffs.append(name + ":" + key)
        urdf_diffs[label] = sorted(set(diffs))

    csv_bytes = {label: path.read_bytes() for label, path in CSV_PATHS.items()}
    csv_hashes = {label: hashlib.sha256(data).hexdigest() for label, data in csv_bytes.items()}
    csv_identical = len(set(csv_bytes.values())) == 1
    csv_model = csvs["W"]

    axes = {}
    axis_dots = {}
    for side in ("left", "right"):
        w_axes = world_axes(base, mount, side)
        c_axes = world_axes(csv_model, mount, side)
        for name in (LEFT_JOINTS if side == "left" else RIGHT_JOINTS):
            dot = sum(w_axes[name][i] * c_axes[name][i] for i in range(3))
            axes[name] = {
                "urdf_world_axis": w_axes[name],
                "csv_world_axis": c_axes[name],
                "dot": dot,
            }
            axis_dots[name] = dot

    samples = [
        json.loads(line)
        for line in SAMPLE_PATH.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    sample = next((row for row in samples if row.get("note") == "L-1"), None)
    if sample is None or len(sample.get("q17", [])) != 17 or len(sample.get("tcp14", [])) != 14:
        raise ValueError("expected L-1 sample with q17 and tcp14")
    q_left = dict(zip(LEFT_JOINTS, sample["q17"][1:8]))
    zero = {name: 0.0 for name in LEFT_JOINTS}

    w_sample = apply_mount(fk(base, q_left, "base", "Link7L"), mount)
    c_sample = apply_mount(fk(csv_model, q_left, "base", "Link7L"), mount)
    w_zero = apply_mount(fk(base, zero, "base", "Link7L"), mount)
    c_zero = apply_mount(fk(csv_model, zero, "base", "Link7L"), mount)

    tcp = {
        "p": sample["tcp14"][0:3],
        "R": quaternion_matrix_wxyz(sample["tcp14"][3:7]),
    }
    w_tool = {
        "R": w_sample["R"],
        "p": _add(w_sample["p"], _matvec(w_sample["R"], [0.0, 0.0, -0.145])),
    }

    return {
        "urdf_sha256": {label: _sha256(path) for label, path in URDF_PATHS.items()},
        "csv_sha256": csv_hashes,
        "csv_copies_identical": csv_identical,
        "urdf_arm_joint_diffs_vs_W": urdf_diffs,
        "arm_joint_count": len(ARM_JOINTS),
        "world_axis_comparison": axes,
        "left_axis_dot_signs": {name: (1 if axis_dots[name] > 0.0 else -1) for name in LEFT_JOINTS},
        "right_axis_dots": {name: axis_dots[name] for name in RIGHT_JOINTS},
        "sample_L-1": {
            "q17_left_slice": [1, 8],
            "q_left": q_left,
            "W_vs_C": pose_difference(w_sample, c_sample),
            "W_vs_TCP": pose_difference(w_sample, tcp),
            "W_vs_TCP_tool_minus_145mm": pose_difference(w_tool, tcp),
            "C_vs_TCP": pose_difference(c_sample, tcp),
        },
        "zero_joint_pose": {
            "W_vs_C": pose_difference(w_zero, c_zero),
        },
    }


def main() -> int:
    print(json.dumps(run_audit(), ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
