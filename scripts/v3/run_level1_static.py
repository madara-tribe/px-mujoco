"""
run_level1_static.py — Level 1: サーボの静止

目的:
  Pattern-B統合モデルをcenter_deg(yaw=90, pitch=90)に初期化し、
  外力(重力)下でその姿勢を維持できるか(静止状態)を確認する。

  これは今後すべての挙動を積み上げていく上での最も基礎的な検証であり、
  b1_static_balance.py で確認したチルトトルク釣り合い(安全率35倍)を
  統合モデル上で再現する位置づけを兼ねる。

実行方法:
  python3 scripts/run_level1_static.py
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
OUTPUT_PATH = ROOT / "outputs" / "level1_static.png"

DURATION_SEC = 2.0


def main():
    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    env.reset()

    n_steps = int(DURATION_SEC / env.model.opt.timestep)
    t_log = np.zeros(n_steps)
    yaw_log = np.zeros(n_steps)
    pitch_log = np.zeros(n_steps)

    for i in range(n_steps):
        # center_degのまま指令を変えない = 静止維持タスク
        env.set_target_deg(env.yaw_center, env.pitch_center)
        env.step()
        t_log[i] = env.data.time
        yaw_deg, pitch_deg = env.get_angles_deg()
        yaw_log[i] = yaw_deg
        pitch_log[i] = pitch_deg

    # ---- 結果評価 ----
    yaw_final_error = abs(yaw_log[-1] - env.yaw_center)
    pitch_final_error = abs(pitch_log[-1] - env.pitch_center)

    print("=== Level 1: Servo Static Hold ===")
    print(f"yaw   target={env.yaw_center:.2f}deg  final={yaw_log[-1]:.3f}deg  "
          f"error={yaw_final_error:.4f}deg")
    print(f"pitch target={env.pitch_center:.2f}deg  final={pitch_log[-1]:.3f}deg  "
          f"error={pitch_final_error:.4f}deg")

    # ---- グラフ出力 ----
    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(t_log, yaw_log, color="tab:blue", lw=1.6)
    axes[0].axhline(env.yaw_center, color="gray", ls="--", lw=1, label="target (center_deg)")
    axes[0].set_ylabel("yaw [deg]")
    axes[0].set_title("Level 1: Static hold at center_deg (yaw=90, pitch=90)")
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.3)

    axes[1].plot(t_log, pitch_log, color="tab:green", lw=1.6)
    axes[1].axhline(env.pitch_center, color="gray", ls="--", lw=1, label="target (center_deg)")
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
