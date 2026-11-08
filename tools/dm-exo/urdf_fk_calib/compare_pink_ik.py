#!/usr/bin/env python3
"""Offline Pink IK comparison for W and the CSV-derived left-arm candidate.

Targets are synthesized from W FK at recorded joint configurations. This isolates
model differences; it is not a comparison against measured robot ground truth.
No ROS nodes, controller connections, or motion commands are used.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import statistics

import numpy as np
import pinocchio as pin
import pink
from pink import solve_ik
from pink.tasks import DampingTask, FrameTask, PostureTask

import csv_urdf_candidate as candidate

ROOT = candidate.ROOT
DEFAULT_OUTPUT = candidate.CANDIDATE_DIR / "pink-ik-comparison.json"


def _load_unique_samples(paths: list[Path]) -> list[dict]:
    samples, _counts = candidate._load_samples(paths)
    return [sample for sample in samples if sample["note"] != "L01"]


def _q_from_sample(model, sample: dict) -> np.ndarray:
    q = pin.neutral(model)
    for name, value in zip(candidate.LEFT_JOINTS, sample["q_left"]):
        if not model.existJointName(name):
            raise ValueError("model missing joint {}".format(name))
        jid = model.getJointId(name)
        joint = model.joints[jid]
        if joint.nq != 1 or joint.nv != 1:
            raise ValueError("{} is not a scalar revolute joint".format(name))
        q[joint.idx_q] = float(value)
    return q


def _within_position_limits(model, q: np.ndarray) -> bool:
    lower = np.asarray(model.lowerPositionLimit)
    upper = np.asarray(model.upperPositionLimit)
    finite_lower = np.isfinite(lower)
    finite_upper = np.isfinite(upper)
    return bool(
        np.all(q[finite_lower] >= lower[finite_lower] - 1e-9)
        and np.all(q[finite_upper] <= upper[finite_upper] + 1e-9)
    )


def _fk_target(model, data, q: np.ndarray):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    frame_id = model.getFrameId("Link7L")
    if frame_id >= model.nframes:
        raise ValueError("model has no Link7L frame")
    return data.oMf[frame_id].copy()


def _pose_error(config, target) -> tuple[float, float]:
    current = config.get_transform_frame_to_world("Link7L")
    position_m = float(np.linalg.norm(target.translation - current.translation))
    rotation_rad = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
    return position_m * 1000.0, rotation_rad * 180.0 / math.pi


def _solve_one(model, target, q_seed: np.ndarray, q_posture: np.ndarray, max_iter: int, dt: float):
    data = model.createData()
    config = pink.Configuration(model, data, q_seed.copy())
    config.update(q_seed.copy())
    frame_task = FrameTask("Link7L", position_cost=300.0, orientation_cost=40.0)
    posture_task = PostureTask(cost=0.3)
    damping_task = DampingTask(cost=12.0)
    frame_task.set_target(target)
    posture_task.set_target(q_posture.copy())
    tasks = [frame_task, posture_task, damping_task]

    q = q_seed.copy()
    initial_position_mm, initial_rotation_deg = _pose_error(config, target)
    final_position_mm, final_rotation_deg = initial_position_mm, initial_rotation_deg
    iterations = 0
    for iteration in range(max_iter):
        config.update(q)
        velocity = np.asarray(
            solve_ik(config, tasks, dt, solver="osqp", safety_break=False),
            dtype=float,
        )
        if velocity.shape != (model.nv,) or not np.all(np.isfinite(velocity)):
            raise RuntimeError("Pink returned invalid joint velocity")
        q = pin.integrate(model, q, dt * velocity)
        config.update(q)
        final_position_mm, final_rotation_deg = _pose_error(config, target)
        iterations = iteration + 1
        if final_position_mm <= 0.5 and final_rotation_deg <= 0.5:
            break

    return {
        "initial_position_mm": initial_position_mm,
        "initial_rotation_deg": initial_rotation_deg,
        "final_position_mm": final_position_mm,
        "final_rotation_deg": final_rotation_deg,
        "iterations": iterations,
        "converged": final_position_mm <= 0.5 and final_rotation_deg <= 0.5,
        "left_joint_solution_delta_rad": [
            float(q[model.joints[model.getJointId(name)].idx_q] - q_posture[model.joints[model.getJointId(name)].idx_q])
            for name in candidate.LEFT_JOINTS
        ],
    }


def compare_pink_ik(
    baseline_urdf: Path,
    candidate_urdf: Path,
    sample_paths: list[Path],
    max_iter: int = 400,
    dt: float = 1.0 / 250.0,
) -> dict:
    models = {
        "baseline_W": pin.buildModelFromUrdf(str(baseline_urdf)),
        "csv_candidate_C": pin.buildModelFromUrdf(str(candidate_urdf)),
    }
    samples = _load_unique_samples(sample_paths)
    per_model = {key: [] for key in models}
    skipped = []

    for sample in samples:
        q_w = _q_from_sample(models["baseline_W"], sample)
        q_c = _q_from_sample(models["csv_candidate_C"], sample)
        if not _within_position_limits(models["baseline_W"], q_w):
            skipped.append({"note": sample["note"], "reason": "recorded q outside W joint limits"})
            continue
        if not _within_position_limits(models["csv_candidate_C"], q_c):
            skipped.append({"note": sample["note"], "reason": "recorded q outside C joint limits"})
            continue

        # Same seed for W/C. Small alternating perturbation prevents the W
        # solve from trivially starting exactly at its synthetic target.
        delta = np.asarray([0.008, -0.008, 0.008, -0.008, 0.008, -0.008, 0.008])
        q_seed_w = q_w.copy()
        q_seed_c = q_c.copy()
        for name, offset in zip(candidate.LEFT_JOINTS, delta):
            idx_w = models["baseline_W"].joints[models["baseline_W"].getJointId(name)].idx_q
            idx_c = models["csv_candidate_C"].joints[models["csv_candidate_C"].getJointId(name)].idx_q
            q_seed_w[idx_w] += offset
            q_seed_c[idx_c] += offset
        if not _within_position_limits(models["baseline_W"], q_seed_w):
            q_seed_w = q_w.copy()
        if not _within_position_limits(models["csv_candidate_C"], q_seed_c):
            q_seed_c = q_c.copy()

        target = _fk_target(models["baseline_W"], models["baseline_W"].createData(), q_w)
        for key, model, seed, posture in (
            ("baseline_W", models["baseline_W"], q_seed_w, q_w),
            ("csv_candidate_C", models["csv_candidate_C"], q_seed_c, q_c),
        ):
            result = _solve_one(model, target, seed, posture, max_iter, dt)
            result["note"] = sample["note"]
            per_model[key].append(result)

    summary = {}
    for key, rows in per_model.items():
        summary[key] = {
            "sample_count": len(rows),
            "converged_count": sum(1 for row in rows if row["converged"]),
            "median_final_position_mm": statistics.median([row["final_position_mm"] for row in rows]) if rows else None,
            "max_final_position_mm": max([row["final_position_mm"] for row in rows]) if rows else None,
            "median_final_rotation_deg": statistics.median([row["final_rotation_deg"] for row in rows]) if rows else None,
            "max_final_rotation_deg": max([row["final_rotation_deg"] for row in rows]) if rows else None,
            "median_iterations": statistics.median([row["iterations"] for row in rows]) if rows else None,
        }

    return {
        "schema_version": 1,
        "baseline_urdf": str(baseline_urdf),
        "candidate_urdf": str(candidate_urdf),
        "pink_version": getattr(pink, "__version__", "unknown"),
        "pinocchio_version": getattr(pin, "__version__", "unknown"),
        "solver": "osqp",
        "frame": "Link7L in model world; target poses synthesized from W FK",
        "task_costs": {"position": 300.0, "orientation": 40.0, "posture": 0.3, "damping": 12.0},
        "integration": {"dt_seconds": dt, "max_iterations": max_iter, "convergence": "position <= 0.5 mm and rotation <= 0.5 deg"},
        "sample_policy": "deduplicate by q_left; exclude L01 because recorded J4L is outside the current W and C limits",
        "targets": "W FK poses at recorded q; not measured TCP targets and not robot ground truth",
        "summary": summary,
        "per_sample": per_model,
        "skipped": skipped,
        "interpretation": "This tests solver behavior under two kinematic models for identical synthetic targets. It does not establish which model matches the physical robot.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=candidate.DEFAULT_URDF)
    parser.add_argument("--candidate", type=Path, default=candidate.DEFAULT_OUTPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--max-iter", type=int, default=400)
    parser.add_argument("--dt", type=float, default=1.0 / 250.0)
    parser.add_argument("--samples", nargs="*", type=Path, default=list(candidate.SAMPLE_FILES))
    args = parser.parse_args()
    report = compare_pink_ik(args.baseline, args.candidate, args.samples, args.max_iter, args.dt)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
