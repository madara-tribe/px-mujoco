"""
run_level2_sweep.py — Level 2: center -> max -> center -> min -> center スイープ

目的:
  Level 1(静止)に「動き」を追加する。run.sh 準拠の可動範囲内で、
  yaw(PAN)・pitch(TILT)それぞれを
    center -> max -> center -> min -> center
  の順に往復させ、MuJoCo上のjoint rangeと矛盾なく動作するか確認する。

  PD制御はまだ使わない(position actuatorへの直接角度指令のみ)。
  PD制御はLevel3以降で追加する。

実行方法:
  python3 scripts/run_level2_sweep.py
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from px_env import PxPanTiltEnv

ROOT = Path(__file__).parent.parent.parent  # px_sim_v4/scripts/v3/ から px_sim_v4/ へ
MODEL_PATH = ROOT / "models" / "pattern_b_integrated.xml"
PARAMS_PATH = ROOT / "data" / "params" / "control_params.yaml"
OUTPUT_PATH = ROOT / "outputs" / "level2_sweep.png"

# 各区間の所要時間(秒)。center->max->center->min->center の4区間。
SEGMENT_DURATION_SEC = 1.5
# 各区間内で、指令値をどれだけの分解能でステップさせるか(往復を滑らかに見せるため)
STEPS_PER_SEGMENT_COMMAND = 30


def build_sweep_waypoints(center, vmin, vmax):
    """center -> max -> center -> min -> center の5点を返す。"""
    return [center, vmax, center, vmin, center]


def interpolate_segment(start, end, n_points):
    """1区間を線形補間したコマンド列を返す(滑らかな往復のため)。"""
    return np.linspace(start, end, n_points)


def main():
    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    env.reset()

    yaw_waypoints = build_sweep_waypoints(env.yaw_center, env.yaw_min, env.yaw_max)
    pitch_waypoints = build_sweep_waypoints(env.pitch_center, env.pitch_min, env.pitch_max)

    n_segments = len(yaw_waypoints) - 1  # 4区間
    steps_per_segment = int(SEGMENT_DURATION_SEC / env.model.opt.timestep)

    t_log, yaw_cmd_log, pitch_cmd_log = [], [], []
    yaw_actual_log, pitch_actual_log = [], []

    segment_labels = ["center->max", "max->center", "center->min", "min->center"]

    for seg_idx in range(n_segments):
        yaw_cmd_seq = interpolate_segment(
            yaw_waypoints[seg_idx], yaw_waypoints[seg_idx + 1], STEPS_PER_SEGMENT_COMMAND
        )
        pitch_cmd_seq = interpolate_segment(
            pitch_waypoints[seg_idx], pitch_waypoints[seg_idx + 1], STEPS_PER_SEGMENT_COMMAND
        )
        steps_per_cmd = steps_per_segment // STEPS_PER_SEGMENT_COMMAND

        for cmd_idx in range(STEPS_PER_SEGMENT_COMMAND):
            yaw_cmd = yaw_cmd_seq[cmd_idx]
            pitch_cmd = pitch_cmd_seq[cmd_idx]
            env.set_target_deg(yaw_cmd, pitch_cmd)

            for _ in range(steps_per_cmd):
                env.step()
                yaw_actual, pitch_actual = env.get_angles_deg()
                t_log.append(env.data.time)
                yaw_cmd_log.append(yaw_cmd)
                pitch_cmd_log.append(pitch_cmd)
                yaw_actual_log.append(yaw_actual)
                pitch_actual_log.append(pitch_actual)

        print(f"segment {seg_idx+1}/{n_segments} ({segment_labels[seg_idx]}) done: "
              f"yaw {yaw_waypoints[seg_idx]:.1f}->{yaw_waypoints[seg_idx+1]:.1f}deg, "
              f"pitch {pitch_waypoints[seg_idx]:.1f}->{pitch_waypoints[seg_idx+1]:.1f}deg")

    t_log = np.array(t_log)
    yaw_cmd_log = np.array(yaw_cmd_log)
    pitch_cmd_log = np.array(pitch_cmd_log)
    yaw_actual_log = np.array(yaw_actual_log)
    pitch_actual_log = np.array(pitch_actual_log)

    # ---- 範囲逸脱チェック(joint rangeと矛盾しないか) ----
    yaw_out_of_range = np.sum((yaw_actual_log < env.yaw_min - 0.5) |
                               (yaw_actual_log > env.yaw_max + 0.5))
    pitch_out_of_range = np.sum((pitch_actual_log < env.pitch_min - 0.5) |
                                 (pitch_actual_log > env.pitch_max + 0.5))

    print("\n=== Level 2: Sweep Summary ===")
    print(f"yaw   range=[{env.yaw_min}, {env.yaw_max}]deg  "
          f"actual_range=[{yaw_actual_log.min():.1f}, {yaw_actual_log.max():.1f}]deg  "
          f"out_of_range_samples={yaw_out_of_range}")
    print(f"pitch range=[{env.pitch_min}, {env.pitch_max}]deg  "
          f"actual_range=[{pitch_actual_log.min():.1f}, {pitch_actual_log.max():.1f}]deg  "
          f"out_of_range_samples={pitch_out_of_range}")

    # ---- グラフ出力 ----
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(t_log, yaw_cmd_log, color="gray", ls="--", lw=1, label="command")
    axes[0].plot(t_log, yaw_actual_log, color="tab:blue", lw=1.6, label="actual")
    axes[0].axhline(env.yaw_min, color="red", ls=":", lw=0.8, alpha=0.6)
    axes[0].axhline(env.yaw_max, color="red", ls=":", lw=0.8, alpha=0.6, label="min/max limit")
    axes[0].set_ylabel("yaw [deg]")
    axes[0].set_title("Level 2: center -> max -> center -> min -> center sweep")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_log, pitch_cmd_log, color="gray", ls="--", lw=1, label="command")
    axes[1].plot(t_log, pitch_actual_log, color="tab:green", lw=1.6, label="actual")
    axes[1].axhline(env.pitch_min, color="red", ls=":", lw=0.8, alpha=0.6)
    axes[1].axhline(env.pitch_max, color="red", ls=":", lw=0.8, alpha=0.6, label="min/max limit")
    axes[1].set_ylabel("pitch [deg]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=8)
    axes[1].grid(alpha=0.3)

    plt.tight_layout()
    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(OUTPUT_PATH, dpi=110)
    print(f"\nsaved: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
