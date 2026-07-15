"""MuJoCo runtime viewer utilities shared by v3_viewer and v4_viewer.

The module is intentionally separate from PxPanTiltEnv.  It adds only
visualization, real-time playback, camera orbit, overlay text, and an optional
target marker.  No control or physics behavior is changed.
"""

from __future__ import annotations

import argparse
import platform
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import mujoco
import numpy as np


@dataclass(frozen=True)
class ViewerConfig:
    enabled: bool = True
    realtime: bool = True
    render_fps: float = 60.0
    playback_speed: float = 1.0
    auto_orbit: bool = False
    orbit_speed_deg_s: float = 20.0
    azimuth_deg: float = 45.0
    elevation_deg: float = -20.0
    distance_m: float = 0.35
    lookat: tuple[float, float, float] = (0.0, 0.0, 0.09)
    keep_open: bool = False
    show_target: bool = True
    show_debug: bool = True


def add_viewer_arguments(parser: argparse.ArgumentParser) -> None:
    """Add a common set of viewer flags to an ArgumentParser."""
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Do not open the MuJoCo 3D viewer; useful for CI/syntax checks.",
    )
    parser.add_argument(
        "--no-realtime",
        action="store_true",
        help="Run simulation without matching wall-clock time.",
    )
    parser.add_argument("--render-fps", type=float, default=60.0)
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=1.0,
        help="1.0=real time, 0.5=half speed, 2.0=double speed.",
    )
    parser.add_argument("--auto-orbit", action="store_true")
    parser.add_argument("--orbit-speed", type=float, default=20.0)
    parser.add_argument("--azimuth", type=float, default=45.0)
    parser.add_argument("--elevation", type=float, default=-20.0)
    parser.add_argument("--distance", type=float, default=0.35)
    parser.add_argument("--lookat-z", type=float, default=0.09)
    parser.add_argument("--keep-open", action="store_true")
    parser.add_argument("--hide-target", action="store_true")
    parser.add_argument("--hide-debug", action="store_true")


def viewer_config_from_args(args: argparse.Namespace) -> ViewerConfig:
    if args.render_fps <= 0:
        raise ValueError("--render-fps must be greater than zero")
    if args.playback_speed <= 0:
        raise ValueError("--playback-speed must be greater than zero")
    return ViewerConfig(
        enabled=not args.headless,
        realtime=not args.no_realtime,
        render_fps=args.render_fps,
        playback_speed=args.playback_speed,
        auto_orbit=args.auto_orbit,
        orbit_speed_deg_s=args.orbit_speed,
        azimuth_deg=args.azimuth,
        elevation_deg=args.elevation,
        distance_m=args.distance,
        lookat=(0.0, 0.0, args.lookat_z),
        keep_open=args.keep_open,
        show_target=not args.hide_target,
        show_debug=not args.hide_debug,
    )


class RuntimeViewer:
    """Non-blocking MuJoCo viewer with throttled real-time synchronization."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, config: ViewerConfig):
        self.model = model
        self.data = data
        self.config = config
        self.handle = None
        self._paused = False
        self._orbit_paused = False
        self._camera_reset_requested = False
        self._last_render_sim_time: float | None = None
        self._last_render_wall_time: float | None = None
        self._sim_anchor: float | None = None
        self._wall_anchor: float | None = None
        self._last_sim_time: float | None = None
        self._initial_camera = (
            config.azimuth_deg,
            config.elevation_deg,
            config.distance_m,
            np.asarray(config.lookat, dtype=float),
        )

    def __enter__(self) -> "RuntimeViewer":
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    @property
    def enabled(self) -> bool:
        return self.config.enabled

    def open(self) -> None:
        if not self.enabled or self.handle is not None:
            return

        if platform.system() == "Darwin" and Path(sys.executable).name != "mjpython":
            raise RuntimeError(
                "macOSでMuJoCo passive viewerを使用する場合は、pythonではなく "
                "mjpython で実行してください。"
            )

        import mujoco.viewer

        self.handle = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            key_callback=self._key_callback,
        )
        self.reset_camera()
        self._reset_time_anchor(float(self.data.time))
        self.handle.sync()

    def close(self) -> None:
        if self.handle is not None:
            self.handle.close()
            self.handle = None

    def is_running(self) -> bool:
        return bool(self.handle is not None and self.handle.is_running())

    def reset_camera(self) -> None:
        if not self.is_running():
            return
        azimuth, elevation, distance, lookat = self._initial_camera
        with self.handle.lock():
            self.handle.cam.azimuth = azimuth
            self.handle.cam.elevation = elevation
            self.handle.cam.distance = distance
            self.handle.cam.lookat[:] = lookat

    def _key_callback(self, keycode: int) -> None:
        if keycode == ord(" "):
            self._paused = not self._paused
        elif keycode in (ord("R"), ord("r")):
            self._camera_reset_requested = True
        elif keycode in (ord("O"), ord("o")):
            self._orbit_paused = not self._orbit_paused

    def _reset_time_anchor(self, sim_time: float) -> None:
        now = time.monotonic()
        self._sim_anchor = sim_time
        self._wall_anchor = now
        self._last_sim_time = sim_time
        self._last_render_sim_time = None
        self._last_render_wall_time = now

    def _wait_while_paused(self) -> bool:
        while self._paused and self.is_running():
            self.handle.sync(state_only=True)
            time.sleep(0.02)
        return self.is_running()

    def _is_render_due(self, sim_time: float) -> bool:
        if self._last_render_sim_time is None:
            return True
        return sim_time - self._last_render_sim_time >= (1.0 / self.config.render_fps)

    def _apply_realtime_delay(self, sim_time: float) -> None:
        if not self.config.realtime:
            return
        assert self._sim_anchor is not None and self._wall_anchor is not None
        desired_elapsed = (sim_time - self._sim_anchor) / self.config.playback_speed
        actual_elapsed = time.monotonic() - self._wall_anchor
        sleep_sec = desired_elapsed - actual_elapsed
        if sleep_sec > 0:
            time.sleep(sleep_sec)

    def _update_camera_orbit(self, now: float) -> None:
        if not self.config.auto_orbit or self._orbit_paused:
            self._last_render_wall_time = now
            return
        previous = self._last_render_wall_time or now
        wall_dt = max(0.0, now - previous)
        with self.handle.lock():
            self.handle.cam.azimuth = (
                self.handle.cam.azimuth + self.config.orbit_speed_deg_s * wall_dt
            ) % 360.0
        self._last_render_wall_time = now

    def _set_overlay(self, status: dict[str, Any]) -> None:
        if not self.config.show_debug or not hasattr(self.handle, "set_texts"):
            return

        left_lines = [
            "PX MuJoCo Viewer",
            "Space: pause | O: orbit pause | R: reset camera",
        ]
        right_lines = ["", ""]
        for key, value in status.items():
            if value is None:
                continue
            left_lines.append(str(key))
            if isinstance(value, float):
                right_lines.append(f"{value:.3f}")
            else:
                right_lines.append(str(value))

        self.handle.set_texts(
            (
                int(mujoco.mjtFontScale.mjFONTSCALE_150),
                int(mujoco.mjtGridPos.mjGRID_TOPLEFT),
                "\n".join(left_lines),
                "\n".join(right_lines),
            )
        )

    def _set_target_marker(
        self,
        target_yaw_deg: float | None,
        target_pitch_deg: float | None,
        yaw_center_deg: float,
        pitch_center_deg: float,
    ) -> None:
        if not self.config.show_target or target_yaw_deg is None or target_pitch_deg is None:
            if self.handle is not None:
                self.handle.user_scn.ngeom = 0
            return

        yaw = np.radians(target_yaw_deg - yaw_center_deg)
        pitch = np.radians(target_pitch_deg - pitch_center_deg)
        radius = 0.18
        origin = np.array([0.0, 0.0, 0.11], dtype=float)
        direction = np.array(
            [
                np.cos(pitch) * np.cos(yaw),
                np.cos(pitch) * np.sin(yaw),
                np.sin(pitch),
            ],
            dtype=float,
        )
        position = origin + radius * direction

        scene = self.handle.user_scn
        scene.ngeom = 1
        mujoco.mjv_initGeom(
            scene.geoms[0],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            np.array([0.008, 0.008, 0.008], dtype=float),
            position,
            np.eye(3, dtype=float).reshape(-1),
            np.array([0.95, 0.25, 0.15, 1.0], dtype=np.float32),
        )
        scene.geoms[0].label = "target"

    def update(
        self,
        *,
        sim_time: float | None = None,
        status: dict[str, Any] | None = None,
        target_yaw_deg: float | None = None,
        target_pitch_deg: float | None = None,
        yaw_center_deg: float = 90.0,
        pitch_center_deg: float = 90.0,
        force: bool = False,
    ) -> bool:
        """Synchronize the viewer. Returns False after the window is closed."""
        if not self.enabled:
            return True
        if not self.is_running():
            return False
        if not self._wait_while_paused():
            return False

        current_sim_time = float(self.data.time if sim_time is None else sim_time)
        if self._last_sim_time is None or current_sim_time < self._last_sim_time - 1e-9:
            self._reset_time_anchor(current_sim_time)
        self._last_sim_time = current_sim_time

        if not force and not self._is_render_due(current_sim_time):
            return True

        self._apply_realtime_delay(current_sim_time)
        if self._camera_reset_requested:
            self.reset_camera()
            self._camera_reset_requested = False
        now = time.monotonic()
        self._update_camera_orbit(now)

        overlay = {"sim time [s]": current_sim_time}
        if status:
            overlay.update(status)

        with self.handle.lock():
            self._set_target_marker(
                target_yaw_deg,
                target_pitch_deg,
                yaw_center_deg,
                pitch_center_deg,
            )
        self._set_overlay(overlay)

        self.handle.sync()
        self._last_render_sim_time = current_sim_time
        return self.is_running()

    def wait_until_closed(self, status: dict[str, Any] | None = None) -> None:
        if not self.enabled or not self.config.keep_open or not self.is_running():
            return
        final_status = {"state": "finished; close window to exit"}
        if status:
            final_status.update(status)
        while self.is_running():
            self.update(status=final_status, force=True)
            time.sleep(0.02)


class EnvViewerBridge:
    """Attach viewer synchronization to an existing env without changing it."""

    def __init__(self, env: Any, viewer: RuntimeViewer):
        self.env = env
        self.viewer = viewer
        self.state: dict[str, Any] = {}
        self.target_yaw_deg: float | None = None
        self.target_pitch_deg: float | None = None
        self._original_step = env.step
        self._attached = False

    def attach(self) -> "EnvViewerBridge":
        if not self._attached:
            self.env.step = self.step
            self._attached = True
        return self

    def detach(self) -> None:
        if self._attached:
            self.env.step = self._original_step
            self._attached = False

    def set_state(
        self,
        *,
        target_yaw_deg: float | None = None,
        target_pitch_deg: float | None = None,
        **state: Any,
    ) -> None:
        self.target_yaw_deg = target_yaw_deg
        self.target_pitch_deg = target_pitch_deg
        self.state = state

    def step(self) -> None:
        self._original_step()
        yaw_deg, pitch_deg = self.env.get_angles_deg()
        ctrl_yaw = float(np.degrees(self.env.data.ctrl[0]))
        ctrl_pitch = float(np.degrees(self.env.data.ctrl[1]))
        status = dict(self.state)
        status.update(
            {
                "actual yaw [deg]": float(yaw_deg),
                "actual pitch [deg]": float(pitch_deg),
                "ctrl yaw [deg]": ctrl_yaw,
                "ctrl pitch [deg]": ctrl_pitch,
            }
        )
        self.viewer.update(
            status=status,
            target_yaw_deg=self.target_yaw_deg,
            target_pitch_deg=self.target_pitch_deg,
            yaw_center_deg=float(self.env.yaw_center),
            pitch_center_deg=float(self.env.pitch_center),
        )
