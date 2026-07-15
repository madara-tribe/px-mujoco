"""Level 1 static-hold test with a live MuJoCo 3D viewer.

The original scripts/v3/run_level1_static.py is not modified.  This viewer
version keeps the same simulation/evaluation logic and writes its plot to
outputs/viewer/.
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
OUTPUT_PATH = ROOT / "outputs" / "viewer" / "level1_static_viewer.png"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Level 1 static hold with live 3D visualization")
    parser.add_argument("--duration", type=float, default=2.0, help="Simulation duration in seconds")
    add_viewer_arguments(parser)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.duration <= 0:
        raise ValueError("--duration must be greater than zero")

    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    env.reset()
    config = viewer_config_from_args(args)

    t_log: list[float] = []
    yaw_log: list[float] = []
    pitch_log: list[float] = []
    n_steps = int(args.duration / env.model.opt.timestep)

    with RuntimeViewer(env.model, env.data, config) as viewer:
        bridge = EnvViewerBridge(env, viewer).attach()
        bridge.set_state(
            phase="Level 1: static hold",
            target_yaw_deg=env.yaw_center,
            target_pitch_deg=env.pitch_center,
        )

        for _ in range(n_steps):
            env.set_target_deg(env.yaw_center, env.pitch_center)
            env.step()
            yaw_deg, pitch_deg = env.get_angles_deg()
            t_log.append(float(env.data.time))
            yaw_log.append(float(yaw_deg))
            pitch_log.append(float(pitch_deg))

        bridge.detach()
        viewer.wait_until_closed({"phase": "Level 1 complete"})

    t_arr = np.asarray(t_log)
    yaw_arr = np.asarray(yaw_log)
    pitch_arr = np.asarray(pitch_log)

    yaw_final_error = abs(yaw_arr[-1] - env.yaw_center)
    pitch_final_error = abs(pitch_arr[-1] - env.pitch_center)

    print("=== Level 1 Viewer: Servo Static Hold ===")
    print(
        f"yaw   target={env.yaw_center:.2f}deg  final={yaw_arr[-1]:.3f}deg  "
        f"error={yaw_final_error:.4f}deg"
    )
    print(
        f"pitch target={env.pitch_center:.2f}deg  final={pitch_arr[-1]:.3f}deg  "
        f"error={pitch_final_error:.4f}deg"
    )

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t_arr, yaw_arr, color="tab:blue", lw=1.6)
    axes[0].axhline(env.yaw_center, color="gray", ls="--", lw=1, label="target")
    axes[0].set_ylabel("yaw [deg]")
    axes[0].set_title("Level 1 Viewer: static hold")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_arr, pitch_arr, color="tab:green", lw=1.6)
    axes[1].axhline(env.pitch_center, color="gray", ls="--", lw=1, label="target")
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
