"""Human-readable 3D visualization of the v4 TrackingModeFSM sequence.

This is intentionally separate from run_level4_fsm_tests.py.  The original
assertion-based tests remain unchanged; this script stretches the transitions
in time so WATCH/DETECT behavior can be inspected visually.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

VIEWER_DIR = Path(__file__).resolve().parent
ROOT = VIEWER_DIR.parent.parent
sys.path.insert(0, str(VIEWER_DIR))
sys.path.insert(0, str(ROOT / "scripts" / "common"))

from px_env import PxPanTiltEnv
from runtime_viewer import RuntimeViewer, add_viewer_arguments, viewer_config_from_args

MODEL_PATH = ROOT / "models" / "pattern_b_integrated.xml"
PARAMS_PATH = ROOT / "data" / "params" / "control_params.yaml"
DETECTION_FPS = 30.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Level 4 WATCH/DETECT FSM live visualization")
    parser.add_argument("--initial-watch", type=float, default=2.0)
    parser.add_argument("--detect-duration", type=float, default=4.0)
    parser.add_argument("--watch-hold", type=float, default=2.0)
    parser.add_argument("--resume-duration", type=float, default=4.0)
    add_viewer_arguments(parser)
    return parser.parse_args()


def mode_name(env: PxPanTiltEnv) -> str:
    return str(env.fsm.mode.value).upper()


def sync_viewer(
    viewer: RuntimeViewer,
    env: PxPanTiltEnv,
    *,
    phase: str,
    target_yaw: float | None,
    target_pitch: float | None,
    detected: bool | None = None,
    command_blocked: bool | None = None,
) -> None:
    yaw, pitch = env.get_angles_deg()
    status = {
        "phase": phase,
        "mode": mode_name(env),
        "first detect frame": env.fsm.first_detect_frame,
        "lost frames": f"{env.fsm.no_det_frames}/{env.lost_max_frames}",
        "detected": detected,
        "command blocked": command_blocked,
        "actual yaw [deg]": float(yaw),
        "actual pitch [deg]": float(pitch),
        "ctrl yaw [deg]": float(np.degrees(env.data.ctrl[0])),
        "ctrl pitch [deg]": float(np.degrees(env.data.ctrl[1])),
    }
    viewer.update(
        status=status,
        target_yaw_deg=target_yaw,
        target_pitch_deg=target_pitch,
        yaw_center_deg=float(env.yaw_center),
        pitch_center_deg=float(env.pitch_center),
    )


def run_static_phase(
    env: PxPanTiltEnv,
    viewer: RuntimeViewer,
    *,
    phase: str,
    duration: float,
    target_yaw: float | None = None,
    target_pitch: float | None = None,
    intentionally_call_tracking: bool = False,
) -> bool:
    dt = float(env.model.opt.timestep)
    detection_period_steps = max(1, int((1.0 / DETECTION_FPS) / dt))
    n_steps = int(duration / dt)
    command_blocked = False

    for i in range(n_steps):
        if intentionally_call_tracking and i % detection_period_steps == 0:
            result = env.track_target_deg(
                target_yaw_deg=float(target_yaw),
                target_pitch_deg=float(target_pitch),
                dt=dt * detection_period_steps,
                use_optics=True,
            )
            command_blocked = result[0] is None
        env.step()
        sync_viewer(
            viewer,
            env,
            phase=phase,
            target_yaw=target_yaw,
            target_pitch=target_pitch,
            command_blocked=command_blocked if intentionally_call_tracking else None,
        )
    return command_blocked


def run_detect_phase(
    env: PxPanTiltEnv,
    viewer: RuntimeViewer,
    *,
    phase: str,
    duration: float,
    target_yaw: float,
    target_pitch: float,
) -> None:
    dt = float(env.model.opt.timestep)
    detection_period_steps = max(1, int((1.0 / DETECTION_FPS) / dt))
    n_steps = int(duration / dt)

    env.enter_detect_track()
    for i in range(n_steps):
        if i % detection_period_steps == 0:
            env.on_detection_result(detected=True)
            env.track_target_deg(
                target_yaw_deg=target_yaw,
                target_pitch_deg=target_pitch,
                dt=dt * detection_period_steps,
                use_optics=True,
            )
        env.step()
        sync_viewer(
            viewer,
            env,
            phase=phase,
            target_yaw=target_yaw,
            target_pitch=target_pitch,
            detected=True,
        )


def run_detection_loss_phase(
    env: PxPanTiltEnv,
    viewer: RuntimeViewer,
    *,
    target_yaw: float,
    target_pitch: float,
) -> bool:
    dt = float(env.model.opt.timestep)
    detection_period_steps = max(1, int((1.0 / DETECTION_FPS) / dt))
    # Threshold plus a small visual tail after the automatic WATCH transition.
    n_detection_frames = env.lost_max_frames + 8
    n_steps = n_detection_frames * detection_period_steps
    transitioned = False

    for i in range(n_steps):
        if i % detection_period_steps == 0 and env.fsm.is_detect():
            transitioned = env.on_detection_result(detected=False) or transitioned
        env.step()
        sync_viewer(
            viewer,
            env,
            phase="Level 4: detection loss -> automatic WATCH",
            target_yaw=target_yaw,
            target_pitch=target_pitch,
            detected=False,
        )
    return transitioned


def main() -> None:
    args = parse_args()
    for name in ("initial_watch", "detect_duration", "watch_hold", "resume_duration"):
        if getattr(args, name) <= 0:
            raise ValueError(f"--{name.replace('_', '-')} must be greater than zero")

    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    env.reset()
    config = viewer_config_from_args(args)

    target_a = (135.0, 65.0)
    target_b = (45.0, 120.0)

    print("=" * 70)
    print("Level 4 Viewer: WATCH -> DETECT -> lost -> WATCH -> DETECT")
    print("Original scripts/v4 tests are unchanged")
    print("=" * 70)

    with RuntimeViewer(env.model, env.data, config) as viewer:
        run_static_phase(
            env,
            viewer,
            phase="Level 4: initial WATCH",
            duration=args.initial_watch,
            target_yaw=env.yaw_center,
            target_pitch=env.pitch_center,
        )

        print("transition: WATCH -> DETECT_TRACK")
        run_detect_phase(
            env,
            viewer,
            phase="Level 4: DETECT_TRACK target A",
            duration=args.detect_duration,
            target_yaw=target_a[0],
            target_pitch=target_a[1],
        )

        transitioned = run_detection_loss_phase(
            env,
            viewer,
            target_yaw=target_a[0],
            target_pitch=target_a[1],
        )
        print(f"automatic transition after lost frames: {transitioned}; mode={mode_name(env)}")

        blocked = run_static_phase(
            env,
            viewer,
            phase="Level 4: WATCH blocks new tracking command",
            duration=args.watch_hold,
            target_yaw=target_b[0],
            target_pitch=target_b[1],
            intentionally_call_tracking=True,
        )
        print(f"WATCH command blocking observed: {blocked}")

        print("transition: WATCH -> DETECT_TRACK (resume)")
        run_detect_phase(
            env,
            viewer,
            phase="Level 4: DETECT_TRACK target B",
            duration=args.resume_duration,
            target_yaw=target_b[0],
            target_pitch=target_b[1],
        )

        viewer.wait_until_closed({"phase": "Level 4 visual sequence complete", "mode": mode_name(env)})

    yaw, pitch = env.get_angles_deg()
    print("\n=== Level 4 Viewer Summary ===")
    print(f"lost-frame auto transition: {'PASS' if transitioned else 'FAIL'}")
    print(f"WATCH command blocking: {'PASS' if blocked else 'FAIL'}")
    print(f"final mode: {mode_name(env)}")
    print(f"final joint angles: yaw={yaw:.3f}deg, pitch={pitch:.3f}deg")


if __name__ == "__main__":
    main()
