"""Level 2 center/max/min sweep with a live MuJoCo 3D viewer.

The original scripts/v3/run_level2_sweep.py is not modified.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

VIEWER_DIR = Path(__file__).resolve().parent
ROOT = VIEWER_DIR.parent.parent
sys.path.insert(0, str(VIEWER_DIR))
sys.path.insert(0, str(ROOT / "scripts" / "common"))

from px_env import PxPanTiltEnv
from runtime_viewer import (
    EnvViewerBridge,
    RuntimeViewer,
    add_viewer_arguments,
    viewer_config_from_args,
)

MODEL_PATH = ROOT / "models" / "pattern_b_integrated.xml"
PARAMS_PATH = ROOT / "data" / "params" / "control_params.yaml"
OUTPUT_PATH = ROOT / "outputs" / "viewer" / "level2_sweep_viewer.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Level 2 pan/tilt sweep with live 3D visualization")
    parser.add_argument("--segment-duration", type=float, default=1.5)
    parser.add_argument("--commands-per-segment", type=int, default=30)
    add_viewer_arguments(parser)
    return parser.parse_args()


def build_sweep_waypoints(center: float, vmin: float, vmax: float) -> list[float]:
    return [center, vmax, center, vmin, center]


def main() -> None:
    args = parse_args()
    if args.segment_duration <= 0:
        raise ValueError("--segment-duration must be greater than zero")
    if args.commands_per_segment <= 0:
        raise ValueError("--commands-per-segment must be greater than zero")

    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    env.reset()
    config = viewer_config_from_args(args)

    yaw_waypoints = build_sweep_waypoints(env.yaw_center, env.yaw_min, env.yaw_max)
    pitch_waypoints = build_sweep_waypoints(env.pitch_center, env.pitch_min, env.pitch_max)
    segment_labels = ["center->max", "max->center", "center->min", "min->center"]
    steps_per_segment = int(args.segment_duration / env.model.opt.timestep)
    steps_per_cmd = max(1, steps_per_segment // args.commands_per_segment)

    t_log: list[float] = []
    yaw_cmd_log: list[float] = []
    pitch_cmd_log: list[float] = []
    yaw_actual_log: list[float] = []
    pitch_actual_log: list[float] = []

    with RuntimeViewer(env.model, env.data, config) as viewer:
        bridge = EnvViewerBridge(env, viewer).attach()

        for seg_idx, label in enumerate(segment_labels):
            yaw_cmd_seq = np.linspace(
                yaw_waypoints[seg_idx], yaw_waypoints[seg_idx + 1], args.commands_per_segment
            )
            pitch_cmd_seq = np.linspace(
                pitch_waypoints[seg_idx], pitch_waypoints[seg_idx + 1], args.commands_per_segment
            )

            for yaw_cmd, pitch_cmd in zip(yaw_cmd_seq, pitch_cmd_seq, strict=True):
                yaw_value = float(yaw_cmd)
                pitch_value = float(pitch_cmd)
                env.set_target_deg(yaw_value, pitch_value)
                bridge.set_state(
                    phase=f"Level 2: {label}",
                    segment=f"{seg_idx + 1}/4",
                    target_yaw_deg=yaw_value,
                    target_pitch_deg=pitch_value,
                )

                for _ in range(steps_per_cmd):
                    env.step()
                    yaw_actual, pitch_actual = env.get_angles_deg()
                    t_log.append(float(env.data.time))
                    yaw_cmd_log.append(yaw_value)
                    pitch_cmd_log.append(pitch_value)
                    yaw_actual_log.append(float(yaw_actual))
                    pitch_actual_log.append(float(pitch_actual))

            print(
                f"segment {seg_idx + 1}/4 ({label}) done: "
                f"yaw {yaw_waypoints[seg_idx]:.1f}->{yaw_waypoints[seg_idx + 1]:.1f}deg, "
                f"pitch {pitch_waypoints[seg_idx]:.1f}->{pitch_waypoints[seg_idx + 1]:.1f}deg"
            )

        bridge.detach()
        viewer.wait_until_closed({"phase": "Level 2 complete"})

    t_arr = np.asarray(t_log)
    yaw_cmd_arr = np.asarray(yaw_cmd_log)
    pitch_cmd_arr = np.asarray(pitch_cmd_log)
    yaw_actual_arr = np.asarray(yaw_actual_log)
    pitch_actual_arr = np.asarray(pitch_actual_log)

    yaw_out = int(np.sum((yaw_actual_arr < env.yaw_min - 0.5) | (yaw_actual_arr > env.yaw_max + 0.5)))
    pitch_out = int(
        np.sum((pitch_actual_arr < env.pitch_min - 0.5) | (pitch_actual_arr > env.pitch_max + 0.5))
    )

    print("\n=== Level 2 Viewer: Sweep Summary ===")
    print(
        f"yaw range=[{env.yaw_min}, {env.yaw_max}]deg  "
        f"actual_range=[{yaw_actual_arr.min():.1f}, {yaw_actual_arr.max():.1f}]deg  "
        f"out_of_range_samples={yaw_out}"
    )
    print(
        f"pitch range=[{env.pitch_min}, {env.pitch_max}]deg  "
        f"actual_range=[{pitch_actual_arr.min():.1f}, {pitch_actual_arr.max():.1f}]deg  "
        f"out_of_range_samples={pitch_out}"
    )

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(t_arr, yaw_cmd_arr, color="gray", ls="--", lw=1, label="command")
    axes[0].plot(t_arr, yaw_actual_arr, color="tab:blue", lw=1.6, label="actual")
    axes[0].axhline(env.yaw_min, color="red", ls=":", lw=0.8, alpha=0.6)
    axes[0].axhline(env.yaw_max, color="red", ls=":", lw=0.8, alpha=0.6, label="limits")
    axes[0].set_ylabel("yaw [deg]")
    axes[0].set_title("Level 2 Viewer: center/max/min sweep")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_arr, pitch_cmd_arr, color="gray", ls="--", lw=1, label="command")
    axes[1].plot(t_arr, pitch_actual_arr, color="tab:green", lw=1.6, label="actual")
    axes[1].axhline(env.pitch_min, color="red", ls=":", lw=0.8, alpha=0.6)
    axes[1].axhline(env.pitch_max, color="red", ls=":", lw=0.8, alpha=0.6, label="limits")
    axes[1].set_ylabel("pitch [deg]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    plt.tight_layout()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUTPUT_PATH, dpi=110)
    plt.close(fig)
    print(f"saved: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
