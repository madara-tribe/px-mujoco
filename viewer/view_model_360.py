#!/usr/bin/env python3
"""Interactive 3D viewer and automatic 360-degree orbit for px_sim_v4.

Examples
--------
Manual inspection:
    python scripts/tools/view_model_360.py

Automatic camera orbit:
    python scripts/tools/view_model_360.py --auto

macOS (launch_passive requirement):
    mjpython scripts/tools/view_model_360.py --auto
"""

from __future__ import annotations

import argparse
import math
import platform
import sys
import time
from pathlib import Path

try:
    import mujoco
    import mujoco.viewer
except ImportError as exc:
    raise SystemExit(
        "MuJoCo is not installed. Run: python -m pip install -r requirements.txt"
    ) from exc


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XML = PROJECT_ROOT / "models" / "pattern_b_integrated.xml"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Display the PX pan-tilt model in 3D. Use --auto to orbit the "
            "viewer camera through 360 degrees."
        )
    )
    parser.add_argument("--xml", type=Path, default=DEFAULT_XML, help="MJCF XML path")
    parser.add_argument("--auto", action="store_true", help="Automatically orbit the camera")
    parser.add_argument(
        "--speed",
        type=float,
        default=30.0,
        help="Orbit speed in degrees/second (default: 30)",
    )
    parser.add_argument("--azimuth", type=float, default=45.0, help="Initial azimuth in degrees")
    parser.add_argument("--elevation", type=float, default=-20.0, help="Camera elevation in degrees")
    parser.add_argument("--distance", type=float, default=0.35, help="Camera distance in metres")
    parser.add_argument("--lookat-z", type=float, default=0.09, help="Camera look-at height in metres")
    parser.add_argument("--yaw", type=float, default=90.0, help="Initial PAN command in degrees")
    parser.add_argument("--pitch", type=float, default=90.0, help="Initial TILT command in degrees")
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=1.0,
        help="Physics settling time before opening the viewer",
    )
    return parser.parse_args()


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    idx = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if idx < 0:
        raise ValueError(f"Actuator not found: {name}")
    return idx


def initialise_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    yaw_deg: float,
    pitch_deg: float,
    settle_seconds: float,
) -> None:
    yaw_id = actuator_id(model, "servo_x")
    pitch_id = actuator_id(model, "servo_y")

    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    data.ctrl[yaw_id] = min(
        max(yaw_rad, float(model.actuator_ctrlrange[yaw_id, 0])),
        float(model.actuator_ctrlrange[yaw_id, 1]),
    )
    data.ctrl[pitch_id] = min(
        max(pitch_rad, float(model.actuator_ctrlrange[pitch_id, 0])),
        float(model.actuator_ctrlrange[pitch_id, 1]),
    )

    steps = max(0, int(settle_seconds / model.opt.timestep))
    for _ in range(steps):
        mujoco.mj_step(model, data)
    mujoco.mj_forward(model, data)


def main() -> int:
    args = parse_args()
    xml_path = args.xml.expanduser().resolve()
    if not xml_path.is_file():
        print(f"XML file not found: {xml_path}", file=sys.stderr)
        return 2
    if args.distance <= 0:
        print("--distance must be positive", file=sys.stderr)
        return 2

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)
    initialise_pose(model, data, args.yaw, args.pitch, args.settle_seconds)

    state = {"auto": bool(args.auto), "azimuth": float(args.azimuth)}

    def key_callback(keycode: int) -> None:
        # Space: pause/resume automatic rotation. R: reset azimuth.
        if keycode == 32:
            state["auto"] = not state["auto"]
            print(f"Automatic rotation: {'ON' if state['auto'] else 'OFF'}")
        elif keycode in (ord("R"), ord("r")):
            state["azimuth"] = float(args.azimuth)
            print("Camera azimuth reset")

    print(f"Loaded: {xml_path}")
    print("Mouse: orbit / pan / zoom using the standard MuJoCo viewer controls")
    print("Space: pause/resume auto rotation | R: reset view | close window: exit")
    if platform.system() == "Darwin" and Path(sys.executable).name != "mjpython":
        print("macOS note: if launch_passive fails, run this script with mjpython.")

    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            key_callback=key_callback,
            show_left_ui=True,
            show_right_ui=True,
        ) as viewer:
            with viewer.lock():
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                viewer.cam.trackbodyid = -1
                viewer.cam.lookat[:] = (0.0, 0.0, args.lookat_z)
                viewer.cam.distance = args.distance
                viewer.cam.elevation = args.elevation
                viewer.cam.azimuth = state["azimuth"]
            viewer.sync()

            last_time = time.perf_counter()
            while viewer.is_running():
                frame_start = time.perf_counter()
                dt = max(0.0, frame_start - last_time)
                last_time = frame_start

                if state["auto"]:
                    state["azimuth"] = (state["azimuth"] + args.speed * dt) % 360.0
                    with viewer.lock():
                        viewer.cam.azimuth = state["azimuth"]

                # Keep actuator physics live while the camera moves.
                target_sim_time = data.time + min(dt, 0.05)
                while data.time < target_sim_time:
                    mujoco.mj_step(model, data)

                viewer.sync()
                elapsed = time.perf_counter() - frame_start
                time.sleep(max(0.0, (1.0 / 60.0) - elapsed))
    except RuntimeError as exc:
        if platform.system() == "Darwin" and "mjpython" in str(exc):
            print(
                "MuJoCo passive viewer on macOS must be launched with:\n"
                f"  mjpython {Path(__file__).relative_to(PROJECT_ROOT)}"
                + (" --auto" if args.auto else ""),
                file=sys.stderr,
            )
            return 1
        raise

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
