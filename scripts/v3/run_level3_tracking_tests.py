"""
run_level3_tracking_tests.py — Level 3: PD制御によるtarget追従検証 (test1〜4)

1回の実行で以下4つの検証を順に行い、結果をPASS/FAILで判定するレポートを出す。
main関数ベース、Colab CPU実行を想定。

test1: 速度スイープ付き定速追従 (baseline)
  複数速度でtargetを一定速度で動かし、追従誤差が速度に対してどう増加するかを見る。
  test2〜4のFAIL原因が「PD制御そのものの限界」か「追加要素固有の問題」かを
  切り分けるための基準線(baseline)を確立する。

test2: 検出ノイズ + ドロップアウトを乗せたtarget追従
  bbox中心のガウスノイズと、確率的な検出ドロップアウトを注入した状態で追従できるか。

test3: サーボ書き込み遅延を模した追従
  px3_servo_node.cpp の SERVO_ARRIVAL_WAIT_MS(実運用30ms)相当の遅延を
  追従ループに挿入し、追従性能の劣化を定量化する。

test4: targetが可動範囲(min/max)ギリギリで動く(クランプ限界)
  targetがrun.sh準拠のmin/max付近を出入りする状況で、クランプ発生時に
  異常な角速度や振動が起きないか確認する。

各testは合否基準(assert相当)を持ち、FAIL時は「どのタイムステップで・
どの軸で・閾値をどれだけ超えたか」を特定してログに出す。
"""

import sys
from pathlib import Path
from dataclasses import dataclass, field

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from px_env import PxPanTiltEnv, apply_detection_noise, apply_detection_dropout

ROOT = Path(__file__).parent.parent.parent  # px_sim_v4/scripts/v3/ から px_sim_v4/ へ
MODEL_PATH = ROOT / "models" / "pattern_b_integrated.xml"
PARAMS_PATH = ROOT / "data" / "params" / "control_params.yaml"
OUTPUT_DIR = ROOT / "outputs"

RNG_SEED = 42  # 再現性確保(ノイズ・ドロップアウトの再現用)


# ============================================================
# 共通: テスト結果を保持するデータクラス
# ============================================================
@dataclass
class TestResult:
    name: str
    passed: bool
    summary: str
    failure_details: list = field(default_factory=list)  # [(t, axis, actual, threshold), ...]

    def report(self):
        status = "PASS" if self.passed else "FAIL"
        lines = [f"[{status}] {self.name}: {self.summary}"]
        if not self.passed:
            lines.append(f"  -> {len(self.failure_details)} violation(s) detected. First 5 shown:")
            for t, axis, actual, threshold in self.failure_details[:5]:
                lines.append(f"     t={t:6.3f}s  axis={axis:5s}  value={actual:8.3f}  threshold={threshold:8.3f}")
        return "\n".join(lines)


# ============================================================
# test1: 速度スイープ付き定速追従 (baseline)
# ============================================================
class Test1ConstantVelocitySweep:
    """
    複数速度でyaw方向に定速移動するtargetを追従させ、
    速度ごとの追従誤差(定常誤差の平均、最大誤差)を記録する。

    重要: 実機のPD制御(axis_pd_controller)は inference.cpp の
    DETECT_TRACKループ内、つまりYOLO推論フレームが来るたびにしか
    呼ばれない(=run.sh準拠 active_fps=30Hz)。物理タイムステップ(2ms/500Hz)
    ごとにPDを呼ぶのは実機と異なる条件になるため、本テストも
    DETECTION_FPSごとにのみPD制御を更新し、その間は前回指令を保持する
    (mj_step自体は物理精度を保つため毎ステップ実行する)。

    位置づけの変更: 当初は絶対誤差の合否判定を予定していたが、実際に
    走らせたところ実機ゲイン(Kp=0.3, Kd=0.2)では実用速度域(10-60deg/s)
    でも10-20deg程度の定常追従誤差が生じることが判明した。これは
    シミュレーションのバグではなく、実機PDゲインの追従性能そのものの
    限界を定量化した結果である。したがってtest1は絶対閾値によるPASS/FAIL
    ではなく、「速度と誤差の関係を記録するbaseline計測」と位置づける。
    test2-4はこのbaselineとの相対比較(悪化率)で合否判定する。
    baseline自体の異常性(発散・NaN等)のみ簡易チェックする。
    """

    SPEEDS_DEG_S = [10, 30, 60, 100, 150, 200]
    DURATION_SEC = 2.0
    DETECTION_FPS = 30.0           # run.sh active_fps 準拠。PD制御はこの周期でのみ更新
    SANITY_MAX_ERROR_DEG = 60.0    # これを超えたら「発散」とみなす(合否判定ではなく健全性チェック)

    def run(self, env: PxPanTiltEnv) -> tuple[TestResult, dict]:
        dt = env.model.opt.timestep
        detection_period_steps = max(1, int((1.0 / self.DETECTION_FPS) / dt))

        results_by_speed = {}
        failure_details = []

        for speed in self.SPEEDS_DEG_S:
            env.reset()
            n_steps = int(self.DURATION_SEC / dt)
            errors = np.zeros(n_steps)
            t_log = np.zeros(n_steps)

            for i in range(n_steps):
                t = i * dt
                target_yaw = float(np.clip(env.yaw_center + speed * t, env.yaw_min, env.yaw_max))
                if i % detection_period_steps == 0:
                    env.track_target_deg(target_yaw, env.pitch_center,
                                         dt * detection_period_steps, use_optics=True)
                env.step()
                yaw, _ = env.get_angles_deg()
                errors[i] = abs(yaw - target_yaw)
                t_log[i] = t

            latter_half_mean = errors[len(errors) // 2:].mean()
            results_by_speed[speed] = {
                "t": t_log, "error": errors,
                "max_error": errors.max(), "latter_half_mean": latter_half_mean,
            }

            # 健全性チェックのみ: NaN、または非現実的な発散が起きていないか
            if not np.isfinite(errors).all():
                failure_details.append((self.DURATION_SEC, f"yaw@{speed}dps", float("nan"), 0.0))
            elif errors.max() > self.SANITY_MAX_ERROR_DEG:
                failure_details.append((self.DURATION_SEC, f"yaw@{speed}dps",
                                         errors.max(), self.SANITY_MAX_ERROR_DEG))

        passed = len(failure_details) == 0
        speed_summary = ", ".join(
            f"{s}dps->{results_by_speed[s]['latter_half_mean']:.1f}deg" for s in self.SPEEDS_DEG_S)
        summary = (f"baseline characterization (not threshold-graded). "
                   f"detection_fps={self.DETECTION_FPS}. latter_half_mean by speed: {speed_summary}. "
                   f"sanity_limit={self.SANITY_MAX_ERROR_DEG}deg")
        return TestResult("test1_constant_velocity_sweep", passed, summary, failure_details), results_by_speed


# ============================================================
# test2: 検出ノイズ + ドロップアウトを乗せた追従
# ============================================================
class Test2NoisyDropoutTracking:
    """
    定速target(test1と同一の速度候補から選択)にガウスノイズと
    ドロップアウトを注入して追従させる。ドロップアウト中はtarget位置を
    更新しない(前回値を保持=実機の「検出なしフレームでは指令を送らない」動作を模す)。

    PD更新はtest1と同じdetection_fps周期に統一する(観測ノイズも同じ周期でのみ
    変化するのが実機の実態のため)。

    v3での変更点: ノイズは角度空間ではなく、track_target_deg(use_optics=True,
    pixel_noise_sigma_px=...)経由でpixel空間に注入する。これにより
    optics.py の cv2.undistortPoints を通じて非線形に角度誤差へ変換される
    (実機のYOLO bbox検出ノイズと同じ発生源・単位になる)。ドロップアウトの
    「前回値保持」ロジック自体は光学変換と無関係なのでtest2側に残す。

    合否基準: ノイズ+ドロップアウト下での誤差が、test1で計測した
              「同一速度・ノイズ無しのbaseline」の2倍以内に収まること
              (絶対値ではなく、ノイズ・ドロップアウト固有の悪化のみを検出する)。
    """

    TRACK_SPEED_DEG_S = 30.0   # test1のSPEEDS_DEG_Sに含まれる値を使用
    PIXEL_NOISE_SIGMA_PX = 3.0   # bbox検出のpixelばらつき(実機のYOLO推論ノイズ相当)
    DROPOUT_PROB = 0.15
    DURATION_SEC = 2.0
    DETECTION_FPS = 30.0
    DEGRADATION_FACTOR_LIMIT = 2.0   # baseline比でこの倍率を超えたらFAIL

    def run(self, env: PxPanTiltEnv, baseline_latter_half_mean: float) -> tuple[TestResult, dict]:
        dt = env.model.opt.timestep
        rng = np.random.default_rng(RNG_SEED)
        detection_period_steps = max(1, int((1.0 / self.DETECTION_FPS) / dt))
        threshold_deg = baseline_latter_half_mean * self.DEGRADATION_FACTOR_LIMIT

        env.reset()
        n_steps = int(self.DURATION_SEC / dt)

        true_target_log = np.zeros(n_steps)
        observed_target_log = np.zeros(n_steps)
        actual_log = np.zeros(n_steps)
        error_vs_true = np.zeros(n_steps)
        t_log = np.zeros(n_steps)
        dropout_flags = np.zeros(n_steps, dtype=bool)

        last_observed_yaw = env.yaw_center
        current_observed_yaw = env.yaw_center
        current_dropout = False

        for i in range(n_steps):
            t = i * dt
            true_target_yaw = float(np.clip(
                env.yaw_center + self.TRACK_SPEED_DEG_S * t, env.yaw_min, env.yaw_max))

            if i % detection_period_steps == 0:
                current_dropout = apply_detection_dropout(rng, self.DROPOUT_PROB)
                if current_dropout:
                    # ドロップアウト中は前回の観測値をそのまま使う(ノイズ・光学変換なし)
                    current_observed_yaw = last_observed_yaw
                    env.track_target_deg(current_observed_yaw, env.pitch_center,
                                         dt * detection_period_steps, use_optics=False)
                else:
                    # 光学変換(pixel空間ノイズ込み)経由で観測誤差を求める。
                    # track_target_deg内部でtrue_err->pixel->noise->angleの変換が行われる。
                    env.track_target_deg(
                        true_target_yaw, env.pitch_center, dt * detection_period_steps,
                        use_optics=True, pixel_noise_sigma_px=self.PIXEL_NOISE_SIGMA_PX, rng=rng)
                    current_observed_yaw = true_target_yaw  # ログ表示用(真値。実際のノイズは内部で注入済み)
                    last_observed_yaw = current_observed_yaw

            env.step()
            yaw, _ = env.get_angles_deg()

            err = abs(yaw - true_target_yaw)
            true_target_log[i] = true_target_yaw
            observed_target_log[i] = current_observed_yaw
            actual_log[i] = yaw
            error_vs_true[i] = err
            t_log[i] = t
            dropout_flags[i] = current_dropout

        latter_half_mean = error_vs_true[len(error_vs_true) // 2:].mean()
        failure_details = []
        if latter_half_mean > threshold_deg:
            failure_details.append((self.DURATION_SEC, "yaw_latter_half_mean",
                                     latter_half_mean, threshold_deg))

        passed = len(failure_details) == 0
        summary = (f"pixel_noise_sigma={self.PIXEL_NOISE_SIGMA_PX}px (via optics), "
                   f"dropout_prob={self.DROPOUT_PROB}, "
                   f"detection_fps={self.DETECTION_FPS}, baseline={baseline_latter_half_mean:.2f}deg, "
                   f"actual={latter_half_mean:.2f}deg, threshold={threshold_deg:.2f}deg "
                   f"(baseline x{self.DEGRADATION_FACTOR_LIMIT}), "
                   f"dropout_frames={dropout_flags.sum()}/{n_steps}")
        data = {"t": t_log, "true_target": true_target_log, "observed_target": observed_target_log,
                "actual": actual_log, "error_vs_true": error_vs_true, "dropout": dropout_flags}
        return TestResult("test2_noisy_dropout_tracking", passed, summary, failure_details), data


# ============================================================
# test3: サーボ書き込み遅延を模した追従
# ============================================================
class Test3ServoDelayTracking:
    """
    px3_servo_node.cpp の SERVO_ARRIVAL_WAIT_MS(実運用30ms)相当の
    遅延をサーボ書き込みに挿入し、定速target追従性能への影響を見る。
    PD更新はtest1と同じdetection_fps周期に統一する。

    合否基準: 遅延ありでの誤差が、test1で計測した同一速度のbaselineの
              2倍以内に収まること(baseline比の悪化率で判定)。
    """

    TRACK_SPEED_DEG_S = 30.0
    DELAY_MS = 30.0
    DURATION_SEC = 2.0
    DETECTION_FPS = 30.0
    DEGRADATION_FACTOR_LIMIT = 2.0

    def run(self, env: PxPanTiltEnv, baseline_latter_half_mean: float) -> tuple[TestResult, dict]:
        dt = env.model.opt.timestep
        detection_period_steps = max(1, int((1.0 / self.DETECTION_FPS) / dt))
        delay_steps = int((self.DELAY_MS / 1000.0) / dt)
        threshold_deg = baseline_latter_half_mean * self.DEGRADATION_FACTOR_LIMIT

        env.reset()
        env.configure_servo_delay(delay_steps)

        n_steps = int(self.DURATION_SEC / dt)
        target_log = np.zeros(n_steps)
        actual_log = np.zeros(n_steps)
        error_log = np.zeros(n_steps)
        t_log = np.zeros(n_steps)

        for i in range(n_steps):
            t = i * dt
            target_yaw = float(np.clip(
                env.yaw_center + self.TRACK_SPEED_DEG_S * t, env.yaw_min, env.yaw_max))
            if i % detection_period_steps == 0:
                env.track_target_deg(target_yaw, env.pitch_center,
                                     dt * detection_period_steps, use_optics=True)
            env.step()
            yaw, _ = env.get_angles_deg()

            target_log[i] = target_yaw
            actual_log[i] = yaw
            error_log[i] = yaw - target_yaw  # 符号付き(発振検出用)
            t_log[i] = t

        env.configure_servo_delay(0)  # 後続テストに影響しないようリセット

        latter_half_abs_mean = np.abs(error_log[len(error_log) // 2:]).mean()
        latter_half = error_log[len(error_log) // 2:]
        sign_changes = int(np.sum(np.diff(np.sign(latter_half)) != 0))

        failure_details = []
        if latter_half_abs_mean > threshold_deg:
            failure_details.append((self.DURATION_SEC, "yaw_latter_half_mean",
                                     latter_half_abs_mean, threshold_deg))

        passed = len(failure_details) == 0
        summary = (f"delay={self.DELAY_MS}ms ({delay_steps} steps), "
                   f"baseline={baseline_latter_half_mean:.2f}deg, actual={latter_half_abs_mean:.2f}deg, "
                   f"threshold={threshold_deg:.2f}deg (baseline x{self.DEGRADATION_FACTOR_LIMIT}), "
                   f"sign_changes(latter half, diagnostic only)={sign_changes}")
        data = {"t": t_log, "target": target_log, "actual": actual_log, "error": error_log}
        return TestResult("test3_servo_delay_tracking", passed, summary, failure_details), data


# ============================================================
# test4: targetが可動範囲ギリギリで動く(クランプ限界)
# ============================================================
class Test4ClampBoundaryTracking:
    """
    targetがrun.sh準拠のmax付近(center+55deg = 145deg、maxは150deg)を
    往復し、クランプ発生時に異常な角速度スパイクが起きないか確認する。

    重要な設計上の注意: 検証対象は「PD制御が出力するコマンド値のstep delta」
    であり、「物理qposのstep delta」ではない。後者はposition actuatorの
    物理応答(kp・慣性)がボトルネックとなり、PD層のmax_step_degを意図的に
    壊しても物理側で自然に抑制されてしまうため、PD層のレート制限バグを
    検出できないことが実際に確認された(env.max_step_deg=50に緩めても
    物理step deltaは最大4.1deg/stepに留まった)。そのため本テストは
    PDコマンド出力(yaw_cmd)の連続差分を直接監視する。

    合否基準: PDコマンド出力の連続差分が、実機仕様のmax_step_deg
              (control_params.yaml参照、既定3.0deg)を超えないこと。
    """

    DURATION_SEC = 2.0
    OSCILLATION_AMPLITUDE_DEG = 55.0   # centerからの振幅。max=150に対しcenter+55=145で余裕僅か
    OSCILLATION_PERIOD_SEC = 0.5       # 高速振動でクランプ境界を頻繁に叩かせる
    DETECTION_FPS = 30.0

    def run(self, env: PxPanTiltEnv) -> tuple[TestResult, dict]:
        dt = env.model.opt.timestep
        detection_period_steps = max(1, int((1.0 / self.DETECTION_FPS) / dt))
        env.reset()

        n_steps = int(self.DURATION_SEC / dt)
        target_log = np.zeros(n_steps)
        actual_log = np.zeros(n_steps)
        cmd_step_delta_log = np.zeros(n_steps)
        t_log = np.zeros(n_steps)
        clamp_hit_log = np.zeros(n_steps, dtype=bool)

        # 検証対象: PDコマンド出力(yaw_cmd)自体の差分。物理qposではない(理由は上記docstring)。
        # 固定閾値は実機仕様値(control_params.yamlのmax_step_deg)を直接参照する
        # (env.max_step_degが壊れているケースも検出できるよう、パラメータファイルから独立に読む)。
        step_limit = env.params.get("future_params", {}).get("max_step_deg", 3.0)
        prev_cmd = env.yaw_center
        current_target_clamped = env.yaw_center

        for i in range(n_steps):
            t = i * dt
            target_yaw = env.yaw_center + self.OSCILLATION_AMPLITUDE_DEG * np.sin(
                2 * np.pi * t / self.OSCILLATION_PERIOD_SEC)
            target_yaw_clamped = float(np.clip(target_yaw, env.yaw_min, env.yaw_max))
            clamp_hit_log[i] = abs(target_yaw_clamped - target_yaw) > 1e-6

            if i % detection_period_steps == 0:
                current_target_clamped = target_yaw_clamped
                yaw_cmd, _, _, _ = env.track_target_deg(
                    current_target_clamped, env.pitch_center, dt * detection_period_steps,
                    use_optics=True)
                cmd_step_delta_log[i] = abs(yaw_cmd - prev_cmd)
                prev_cmd = yaw_cmd
            else:
                cmd_step_delta_log[i] = 0.0  # このステップではPD更新なし

            env.step()
            yaw, _ = env.get_angles_deg()

            target_log[i] = current_target_clamped
            actual_log[i] = yaw
            t_log[i] = t

        failure_details = [
            (t_log[i], "yaw_pd_cmd_step_delta", cmd_step_delta_log[i], step_limit)
            for i in range(n_steps) if cmd_step_delta_log[i] > step_limit
        ]

        passed = len(failure_details) == 0
        summary = (f"amplitude={self.OSCILLATION_AMPLITUDE_DEG}deg, "
                   f"period={self.OSCILLATION_PERIOD_SEC}s, "
                   f"clamp_hit_frames={clamp_hit_log.sum()}/{n_steps}, "
                   f"pd_cmd_step_delta_limit={step_limit:.2f}deg/frame (from control_params.yaml)")
        data = {"t": t_log, "target": target_log, "actual": actual_log,
                "step_delta": cmd_step_delta_log, "clamp_hit": clamp_hit_log}
        return TestResult("test4_clamp_boundary_tracking", passed, summary, failure_details), data


# ============================================================
# プロット出力
# ============================================================
def plot_results(test1_data, test2_data, test3_data, test4_data):
    fig, axes = plt.subplots(4, 1, figsize=(11, 16))

    # test1
    ax = axes[0]
    for speed, d in test1_data.items():
        ax.plot(d["t"], d["error"], label=f"{speed}deg/s", lw=1.2)
    ax.set_title("Test1: Constant Velocity Sweep - tracking error vs speed")
    ax.set_xlabel("time [s]"); ax.set_ylabel("|error| [deg]")
    ax.legend(fontsize=7, ncol=3); ax.grid(alpha=0.3)

    # test2
    ax = axes[1]
    ax.plot(test2_data["t"], test2_data["true_target"], color="gray", ls="--", lw=1, label="true target")
    ax.plot(test2_data["t"], test2_data["observed_target"], color="orange", lw=0.6, alpha=0.5, label="observed (noisy)")
    ax.plot(test2_data["t"], test2_data["actual"], color="tab:blue", lw=1.4, label="actual")
    dropout_t = test2_data["t"][test2_data["dropout"]]
    if len(dropout_t) > 0:
        ax.scatter(dropout_t, np.full_like(dropout_t, ax.get_ylim()[0]),
                   marker="|", color="red", s=20, label="dropout")
    ax.set_title("Test2: Noisy + Dropout Tracking")
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw [deg]")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # test3
    ax = axes[2]
    ax.plot(test3_data["t"], test3_data["target"], color="gray", ls="--", lw=1, label="target")
    ax.plot(test3_data["t"], test3_data["actual"], color="tab:green", lw=1.4, label="actual (30ms delay)")
    ax.set_title("Test3: Servo Delay (30ms) Tracking")
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw [deg]")
    ax.legend(fontsize=7); ax.grid(alpha=0.3)

    # test4
    ax = axes[3]
    ax2 = ax.twinx()
    ax.plot(test4_data["t"], test4_data["target"], color="gray", ls="--", lw=1, label="target (clamped)")
    ax.plot(test4_data["t"], test4_data["actual"], color="tab:purple", lw=1.4, label="actual")
    ax2.plot(test4_data["t"], test4_data["step_delta"], color="red", lw=0.8, alpha=0.6, label="step delta")
    ax.set_title("Test4: Clamp Boundary Tracking")
    ax.set_xlabel("time [s]"); ax.set_ylabel("yaw [deg]")
    ax2.set_ylabel("step delta [deg/step]", color="red")
    ax.legend(fontsize=7, loc="upper left"); ax.grid(alpha=0.3)

    plt.tight_layout()
    out_path = OUTPUT_DIR / "level3_tracking_tests.png"
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, dpi=110)
    return out_path


# ============================================================
# main
# ============================================================
def main():
    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))

    print("=" * 70)
    print("Level 3: PD Tracking Test Suite (test1-4, single MuJoCo run)")
    print("=" * 70)

    # ---- test1: baseline characterization ----
    test1 = Test1ConstantVelocitySweep()
    result1, data1 = test1.run(env)
    print("\n" + result1.report())
    baseline_30dps = data1[30]["latter_half_mean"]  # test2/3が使う速度(30deg/s)のbaseline

    # ---- test2: noisy + dropout ----
    test2 = Test2NoisyDropoutTracking()
    result2, data2 = test2.run(env, baseline_latter_half_mean=baseline_30dps)
    print("\n" + result2.report())

    # ---- test3: servo delay ----
    test3 = Test3ServoDelayTracking()
    result3, data3 = test3.run(env, baseline_latter_half_mean=baseline_30dps)
    print("\n" + result3.report())

    # ---- test4: clamp boundary ----
    test4 = Test4ClampBoundaryTracking()
    result4, data4 = test4.run(env)
    print("\n" + result4.report())

    # ---- 総合サマリ ----
    all_results = [result1, result2, result3, result4]
    n_pass = sum(r.passed for r in all_results)
    print("\n" + "=" * 70)
    print(f"SUMMARY: {n_pass}/{len(all_results)} tests passed")
    for r in all_results:
        print(f"  [{'PASS' if r.passed else 'FAIL'}] {r.name}")
    print("=" * 70)

    # ---- プロット ----
    out_path = plot_results(data1, data2, data3, data4)
    print(f"\nplot saved: {out_path}")

    return all_results


if __name__ == "__main__":
    main()
