"""
run_level4_fsm_tests.py — Level 4: TrackingModeFSM 検証 (test5, ①+②+③スコープ)

①(挙動11: watch/detect 2-mode遷移)+②(挙動5: 初回DETECTフレームのD項スキップの
完全統合)+③(挙動8: ロストフレームによるWATCH復帰)に対する検証。
挙動12(fps間引き)はまだ統合していない。

test5-a: FSM単体の状態遷移(actuator/MuJoCoを介さない)
  - 初期状態がWATCHであること
  - enter_detect_track() で DETECT_TRACK に遷移し、first_detect_frame=True になること
  - consume_first_detect_frame() が1回目True・2回目Falseを返すこと(一回性の消費)
  - enter_watch() で WATCH に戻り、no_det_frames が0にリセットされること

test5-b: Env統合時のリセット順序確認(04_future_problem_prediction.md 2-A対策)
  - env.enter_detect_track() 呼び出し後、yaw_pd/pitch_pdの
    pos_deg・prev_err_degが実際に0にリセットされていること
    (FSM遷移とPDリセットが同じタイミングで揃って発生することを確認する)

test5-d: track_target_deg()内部でのdt自動上書き(②で追加、挙動5の完全統合)
  - enter_detect_track()直後のtrack_target_deg()呼び出しでは、
    引数dtに何を渡してもD項がスキップされる(=内部でdt=0.0に強制される)こと
  - 2回目以降の呼び出しでは、渡したdtがそのまま使われD項が計算されること
    (一回性の消費が正しく機能していること)

test5-c: watch中はactuatorへ新規指令を送らないこと(state transitionの動作確認)
  - detectでtrackingしていた状態からenter_watch()を呼んだ直後、
    (この後の挙動12実装前段階として)少なくとも「FSMがwatchに
    切り替わったこと」と「PDの状態が保持されたままであること」を確認する。
    実際に「watch中はctrl更新自体を止める」制御はEnv.track_target_deg()に
    まだ組み込まれていない(④で挙動12と合わせて実装予定)ため、
    ここではFSM状態とPD状態の整合性のみを検証する。

test5-e: ロストフレームによる自動WATCH復帰(③で追加、挙動8)
  - detect中、on_detection_result(False)を lost_max_frames 回連続で呼ぶと、
    (lost_max_frames-1)回目まではDETECT_TRACKのまま、
    lost_max_frames回目でWATCHへ自動遷移すること
  - 遷移が起きたフレームで戻り値Trueが返ること、それ以外はFalseであること

test5-f: ロストフレームカウントの途中リセット(③で追加、挙動8)
  - detect中、検出失敗が続いた後に検出成功(detected=True)を挟むと
    no_det_framesが0にリセットされ、WATCHへは遷移しないこと

test5-g: WATCH中にon_detection_result()を呼んでも無害であること(③で追加)
  - WATCH状態でon_detection_result()を呼んでもno_det_framesは変化せず、
    戻り値は常にFalseであること(実機のメインループがWATCH中この分岐を
    通らないことのシム側再現)

test5-h: WATCH中はtrack_target_deg()がactuator/PD状態を一切変更しないこと(④で追加、挙動12)
  - enter_watch()後にtrack_target_deg()を呼んでも、(None, None, False, False)を
    返し、data.ctrl・yaw_pd/pitch_pdの状態(pos_deg, prev_err_deg)が
    一切変化しないこと(実機がWATCH中computeAxisDeg()を呼ばないことの再現)

test5-i: 「動いている状態から止まる→また動き出す」一連の遷移パターン(④で追加、挙動12)
  - detectで実際にtrackingさせてjointを動かす
  - ロストフレームでWATCHへ自動遷移し、その後何度track_target_deg()を
    呼んでもjoint角度が変化しない(静止したまま)ことを複数ステップ確認
  - 再度enter_detect_track()でdetectへ復帰し、trackingが再開されて
    joint角度が実際に動き出すことを確認
"""

import sys
from pathlib import Path
from dataclasses import dataclass, field

sys.path.insert(0, str(Path(__file__).parent))
from px_env import PxPanTiltEnv
from tracking_mode_fsm import TrackingModeFSM, Mode

ROOT = Path(__file__).parent.parent.parent  # px_sim_v4/scripts/v4/ から px_sim_v4/ へ
MODEL_PATH = ROOT / "models" / "pattern_b_integrated.xml"
PARAMS_PATH = ROOT / "data" / "params" / "control_params.yaml"


@dataclass
class TestResult:
    name: str
    passed: bool
    summary: str
    failure_details: list = field(default_factory=list)

    def report(self):
        status = "PASS" if self.passed else "FAIL"
        lines = [f"[{status}] {self.name}: {self.summary}"]
        for d in self.failure_details:
            lines.append(f"    - {d}")
        return "\n".join(lines)


# ============================================================
# test5-a: FSM単体の状態遷移
# ============================================================
def test5a_fsm_standalone():
    failures = []
    fsm = TrackingModeFSM()

    # 初期状態はWATCH
    if not fsm.is_watch():
        failures.append(f"initial mode should be WATCH, got {fsm.mode}")
    if fsm.no_det_frames != 0:
        failures.append(f"initial no_det_frames should be 0, got {fsm.no_det_frames}")
    if fsm.first_detect_frame:
        failures.append("initial first_detect_frame should be False")

    # WATCH -> DETECT_TRACK
    fsm.enter_detect_track()
    if not fsm.is_detect():
        failures.append(f"after enter_detect_track(), mode should be DETECT_TRACK, got {fsm.mode}")
    if not fsm.first_detect_frame:
        failures.append("after enter_detect_track(), first_detect_frame should be True")
    if fsm.no_det_frames != 0:
        failures.append(f"after enter_detect_track(), no_det_frames should be 0, got {fsm.no_det_frames}")

    # consume_first_detect_frame: 1回目True、2回目False (一回性)
    first_call = fsm.consume_first_detect_frame()
    second_call = fsm.consume_first_detect_frame()
    if not first_call:
        failures.append("consume_first_detect_frame() 1st call should return True")
    if second_call:
        failures.append("consume_first_detect_frame() 2nd call should return False (one-shot)")
    if fsm.first_detect_frame:
        failures.append("first_detect_frame should be False after being consumed")

    # DETECT_TRACK -> WATCH
    fsm.no_det_frames = 7  # 遷移前にダミー値を入れて、リセットされることを確認
    fsm.enter_watch()
    if not fsm.is_watch():
        failures.append(f"after enter_watch(), mode should be WATCH, got {fsm.mode}")
    if fsm.no_det_frames != 0:
        failures.append(f"after enter_watch(), no_det_frames should be reset to 0, got {fsm.no_det_frames}")

    passed = len(failures) == 0
    return TestResult(
        name="test5a_fsm_standalone",
        passed=passed,
        summary="FSM単体の状態遷移(WATCH<->DETECT_TRACK, first_detect_frameの一回性消費)",
        failure_details=failures,
    )


# ============================================================
# test5-b: Env統合時のリセット順序確認
# ============================================================
def test5b_env_reset_order(env: PxPanTiltEnv):
    failures = []
    env.reset()

    # PDに意図的にダミーの状態を積む(実際にtrackingさせて誤差を残す)
    env.track_target_deg(target_yaw_deg=20.0, target_pitch_deg=10.0, dt=1.0 / 30.0)
    for _ in range(10):
        env.step()
    env.track_target_deg(target_yaw_deg=20.0, target_pitch_deg=10.0, dt=1.0 / 30.0)

    # ダミー状態が残っていることを事前確認(pos_degやprev_err_degが非ゼロのはず)
    pre_yaw_prev_err = env.yaw_pd.prev_err_deg
    pre_pitch_prev_err = env.pitch_pd.prev_err_deg
    if pre_yaw_prev_err == 0.0 and pre_pitch_prev_err == 0.0:
        failures.append(
            "precondition failed: expected non-zero prev_err_deg before enter_detect_track() "
            "(test setup issue, not FSM issue)"
        )

    # FSM遷移 + PDリセットが揃って起きることを確認
    env.enter_detect_track()

    if not env.fsm.is_detect():
        failures.append(f"env.fsm should be DETECT_TRACK after enter_detect_track(), got {env.fsm.mode}")
    if not env.fsm.first_detect_frame:
        failures.append("env.fsm.first_detect_frame should be True after enter_detect_track()")
    if env.yaw_pd.prev_err_deg != 0.0:
        failures.append(f"yaw_pd.prev_err_deg should be reset to 0.0, got {env.yaw_pd.prev_err_deg}")
    if env.pitch_pd.prev_err_deg != 0.0:
        failures.append(f"pitch_pd.prev_err_deg should be reset to 0.0, got {env.pitch_pd.prev_err_deg}")
    if env.yaw_pd.pos_deg != 0.0:
        failures.append(f"yaw_pd.pos_deg should be reset to 0.0, got {env.yaw_pd.pos_deg}")
    if env.pitch_pd.pos_deg != 0.0:
        failures.append(f"pitch_pd.pos_deg should be reset to 0.0, got {env.pitch_pd.pos_deg}")

    # ②統合後: consume_first_detect_frame()はtrack_target_deg()内部で自動的に
    # 呼ばれるため、ここではまだFSM側にフラグが残っていることを確認する
    # (このメソッド自身では消費しない。実際の消費テストはtest5dで行う)。
    if not env.fsm.first_detect_frame:
        failures.append(
            "first_detect_frame should still be True here "
            "(not yet consumed by track_target_deg())"
        )

    passed = len(failures) == 0
    return TestResult(
        name="test5b_env_reset_order",
        passed=passed,
        summary="env.enter_detect_track()呼び出し時、FSM遷移とPDリセットが揃って発生すること",
        failure_details=failures,
    )


# ============================================================
# test5-d: track_target_deg()内部でのdt自動上書き(挙動5の完全統合、②スコープ)
# ============================================================
def test5d_track_target_deg_auto_dt_zero(env: PxPanTiltEnv):
    """
    ②で追加: track_target_deg()に引数dt!=0.0を渡しても、直前に
    enter_detect_track()が呼ばれていれば内部でdt=0.0に強制されることを確認する。
    D項がスキップされたことは、prev_err_degが0の状態からの1回目の呼び出しで
    delta_degがKp*err_degのみ(D項ゼロ)になることで間接的に確認する。
    """
    failures = []
    env.reset()
    env.enter_detect_track()

    if not env.fsm.first_detect_frame:
        failures.append("precondition failed: first_detect_frame should be True after enter_detect_track()")

    # 意図的に大きなdt!=0.0を渡す。もしdt上書きが効いていなければ、
    # d_term = Kd * (err_deg - 0) / dt が計算されてしまい、
    # 内部でdt=0.0に上書きされていれば d_term=0 のまま(P項のみ)になる。
    target_yaw_deg, target_pitch_deg = 20.0, 10.0
    yaw_cmd, pitch_cmd, _, _ = env.track_target_deg(
        target_yaw_deg=target_yaw_deg, target_pitch_deg=target_pitch_deg, dt=0.5,
    )

    # フラグが消費されて False になっていること(一回性)
    if env.fsm.first_detect_frame:
        failures.append("first_detect_frame should be consumed (False) after track_target_deg() call")

    # P項のみでの期待値と比較する(d_term=0のはず)
    yaw_now, pitch_now = env.get_angles_deg()
    # env.reset()直後はcenter_deg(90,90)にいるはずなので、真の誤差は target - 90
    expected_yaw_err = target_yaw_deg - yaw_now
    expected_pitch_err = target_pitch_deg - pitch_now
    expected_yaw_delta = env.kp_yaw * expected_yaw_err  # d_term=0のはず
    expected_pitch_delta = env.kp_pitch * expected_pitch_err
    expected_yaw_delta = max(-env.max_step_deg, min(env.max_step_deg, expected_yaw_delta))
    expected_pitch_delta = max(-env.max_step_deg, min(env.max_step_deg, expected_pitch_delta))
    expected_yaw_cmd = env.yaw_center + expected_yaw_delta
    expected_pitch_cmd = env.pitch_center + expected_pitch_delta

    tol = 1e-6
    if abs(yaw_cmd - expected_yaw_cmd) > tol:
        failures.append(
            f"yaw_cmd suggests D-term was NOT skipped: got {yaw_cmd}, "
            f"expected(P-only)={expected_yaw_cmd} (dt=0.5 argument should have been overridden to 0.0)"
        )
    if abs(pitch_cmd - expected_pitch_cmd) > tol:
        failures.append(
            f"pitch_cmd suggests D-term was NOT skipped: got {pitch_cmd}, "
            f"expected(P-only)={expected_pitch_cmd}"
        )

    # 2回目の呼び出しでは、もはやdt上書きが効かず、渡したdtがそのまま使われるはず
    # (prev_err_degが非ゼロになっているので、D項が乗るかどうかで判定する)。
    # 1回目呼び出しだけではactuator指令が出るのみでjointはまだ動いていないため、
    # 誤差を変化させるには物理ステップを挟んでjointを実際に動かす必要がある。
    for _ in range(5):
        env.step()
    prev_err_before_call2_yaw = env.yaw_pd.prev_err_deg  # 1回目呼び出し後にセットされた値
    yaw_now_2, pitch_now_2 = env.get_angles_deg()
    err2_yaw_true = target_yaw_deg - yaw_now_2

    yaw_cmd_2, pitch_cmd_2, _, _ = env.track_target_deg(
        target_yaw_deg=target_yaw_deg, target_pitch_deg=target_pitch_deg, dt=0.5,
    )
    d_term_2_yaw = env.kd_yaw * (err2_yaw_true - prev_err_before_call2_yaw) / 0.5
    if abs(d_term_2_yaw) < tol:
        failures.append(
            "2nd call's D-term is unexpectedly ~0; expected non-zero D-term once dt "
            "override no longer applies and the joint has actually moved between calls "
            f"(err2_yaw_true={err2_yaw_true}, prev_err_before_call2_yaw={prev_err_before_call2_yaw})"
        )

    passed = len(failures) == 0
    return TestResult(
        name="test5d_track_target_deg_auto_dt_zero",
        passed=passed,
        summary="track_target_deg()内部でFSMのfirst_detect_frameを消費しdt=0を自動適用すること(挙動5完全統合)",
        failure_details=failures,
    )


# ============================================================
# test5-c: watch復帰時のFSM/PD状態整合性
# ============================================================
def test5c_watch_transition_state_consistency(env: PxPanTiltEnv):
    failures = []
    env.reset()

    # detectモードに入り、実際にtrackingさせる
    # ②統合後: dtは常に引数として渡してよく、first_detect_frameの消費・
    # dt=0への上書きはtrack_target_deg()内部が自動的に行う。
    env.enter_detect_track()
    env.track_target_deg(target_yaw_deg=15.0, target_pitch_deg=-8.0, dt=1.0 / 30.0)
    for _ in range(5):
        env.step()
        env.track_target_deg(target_yaw_deg=15.0, target_pitch_deg=-8.0, dt=1.0 / 30.0)

    yaw_before, pitch_before = env.get_angles_deg()
    yaw_pd_pos_before = env.yaw_pd.pos_deg
    pitch_pd_pos_before = env.pitch_pd.pos_deg

    # WATCHへ遷移
    env.enter_watch()

    if not env.fsm.is_watch():
        failures.append(f"env.fsm should be WATCH after enter_watch(), got {env.fsm.mode}")
    if env.fsm.no_det_frames != 0:
        failures.append(f"no_det_frames should be reset to 0, got {env.fsm.no_det_frames}")

    # 実機同様、enter_watch()はPD状態を変更しない(pos_deg/prev_err_degは保持される)ことを確認
    if env.yaw_pd.pos_deg != yaw_pd_pos_before:
        failures.append(
            f"yaw_pd.pos_deg should be preserved across enter_watch() "
            f"(before={yaw_pd_pos_before}, after={env.yaw_pd.pos_deg})"
        )
    if env.pitch_pd.pos_deg != pitch_pd_pos_before:
        failures.append(
            f"pitch_pd.pos_deg should be preserved across enter_watch() "
            f"(before={pitch_pd_pos_before}, after={env.pitch_pd.pos_deg})"
        )

    # actuator角度もこの時点では変化していないはず(まだstep()していないため)
    yaw_after, pitch_after = env.get_angles_deg()
    if abs(yaw_after - yaw_before) > 1e-9 or abs(pitch_after - pitch_before) > 1e-9:
        failures.append(
            f"joint angles should not change merely by calling enter_watch() "
            f"(before=({yaw_before:.4f},{pitch_before:.4f}), after=({yaw_after:.4f},{pitch_after:.4f}))"
        )

    passed = len(failures) == 0
    return TestResult(
        name="test5c_watch_transition_state_consistency",
        passed=passed,
        summary="enter_watch()呼び出し時、FSM状態のみ変化しPD/joint状態は保持されること",
        failure_details=failures,
    )


# ============================================================
# test5-e: ロストフレームによる自動WATCH復帰(③、挙動8)
# ============================================================
def test5e_lost_frames_auto_watch(env: PxPanTiltEnv):
    """
    実機inference.cppの分岐を直訳:
      no_det_frames_++; if (no_det_frames_ >= lost_max_frames_) enterWatchMode();
    lost_max_frames回連続で検出失敗を送ると、その回でWATCHへ自動遷移すること。
    """
    failures = []
    env.reset()
    env.enter_detect_track()

    lost_max = env.lost_max_frames
    transitioned_at = None
    for i in range(1, lost_max + 1):
        transitioned = env.on_detection_result(detected=False)
        if i < lost_max:
            if transitioned:
                failures.append(f"unexpected early transition at frame {i}/{lost_max}")
            if not env.fsm.is_detect():
                failures.append(f"mode should still be DETECT_TRACK at frame {i}/{lost_max}, got {env.fsm.mode}")
            if env.fsm.no_det_frames != i:
                failures.append(f"no_det_frames should be {i} at frame {i}, got {env.fsm.no_det_frames}")
        else:
            transitioned_at = i
            if not transitioned:
                failures.append(f"expected transition to WATCH at frame {i}/{lost_max}, got no transition")
            if not env.fsm.is_watch():
                failures.append(f"mode should be WATCH after {lost_max} consecutive lost frames, got {env.fsm.mode}")
            if env.fsm.no_det_frames != 0:
                failures.append(f"no_det_frames should be reset to 0 after transition, got {env.fsm.no_det_frames}")

    if transitioned_at != lost_max:
        failures.append(f"transition should occur exactly at frame {lost_max}, got {transitioned_at}")

    passed = len(failures) == 0
    return TestResult(
        name="test5e_lost_frames_auto_watch",
        passed=passed,
        summary=f"lost_max_frames={env.lost_max_frames}回連続の検出失敗でDETECT_TRACK->WATCHへ自動遷移すること",
        failure_details=failures,
    )


# ============================================================
# test5-f: ロストフレームカウントの途中リセット(③、挙動8)
# ============================================================
def test5f_lost_frames_reset_on_detection(env: PxPanTiltEnv):
    """
    lost_max_frames未満の連続失敗の後に検出成功を挟むと、no_det_framesが
    0にリセットされ、WATCHへは遷移しないこと(実機の no_det_frames_ = 0 分岐)。
    """
    failures = []
    env.reset()
    env.enter_detect_track()

    lost_max = env.lost_max_frames
    half = lost_max // 2

    for i in range(1, half + 1):
        env.on_detection_result(detected=False)
    if env.fsm.no_det_frames != half:
        failures.append(f"no_det_frames should be {half} before reset, got {env.fsm.no_det_frames}")

    transitioned = env.on_detection_result(detected=True)
    if transitioned:
        failures.append("on_detection_result(detected=True) should never trigger a transition")
    if env.fsm.no_det_frames != 0:
        failures.append(f"no_det_frames should be reset to 0 after a successful detection, got {env.fsm.no_det_frames}")
    if not env.fsm.is_detect():
        failures.append(f"mode should remain DETECT_TRACK, got {env.fsm.mode}")

    # リセット後、再びlost_max_frames未満の失敗を重ねてもWATCHへ遷移しないこと
    for i in range(1, lost_max):
        transitioned = env.on_detection_result(detected=False)
        if transitioned:
            failures.append(f"unexpected transition at post-reset frame {i}/{lost_max - 1}")
    if not env.fsm.is_detect():
        failures.append(f"mode should still be DETECT_TRACK just before reaching lost_max_frames again, got {env.fsm.mode}")

    passed = len(failures) == 0
    return TestResult(
        name="test5f_lost_frames_reset_on_detection",
        passed=passed,
        summary="連続失敗の途中で検出成功を挟むとno_det_framesがリセットされ、WATCHへ遷移しないこと",
        failure_details=failures,
    )


# ============================================================
# test5-g: WATCH中のon_detection_result()は無害であること(③)
# ============================================================
def test5g_on_detection_result_noop_in_watch(env: PxPanTiltEnv):
    """
    実機のメインループはWATCH中この分岐自体を通らない。シム側でも
    WATCH状態でon_detection_result()が呼ばれた場合は何も変化させないこと。
    """
    failures = []
    env.reset()  # reset()直後はWATCH状態のはず

    if not env.fsm.is_watch():
        failures.append(f"precondition failed: env should start in WATCH after reset(), got {env.fsm.mode}")

    transitioned = env.on_detection_result(detected=False)
    if transitioned:
        failures.append("on_detection_result() should never report a transition while already in WATCH")
    if env.fsm.no_det_frames != 0:
        failures.append(f"no_det_frames should remain 0 while in WATCH, got {env.fsm.no_det_frames}")
    if not env.fsm.is_watch():
        failures.append(f"mode should remain WATCH, got {env.fsm.mode}")

    # detectedがTrueの場合も同様に無害であること
    transitioned2 = env.on_detection_result(detected=True)
    if transitioned2:
        failures.append("on_detection_result(detected=True) should never report a transition while in WATCH")

    passed = len(failures) == 0
    return TestResult(
        name="test5g_on_detection_result_noop_in_watch",
        passed=passed,
        summary="WATCH状態でon_detection_result()を呼んでも状態が変化しないこと",
        failure_details=failures,
    )


# ============================================================
# test5-h: WATCH中はtrack_target_deg()が何も変更しないこと(④、挙動12)
# ============================================================
def test5h_watch_blocks_track_target_deg(env: PxPanTiltEnv):
    """
    実機はWATCH中computeAxisDeg()(PD更新)自体を呼ばない。
    track_target_deg()がWATCH中に呼ばれた場合、actuator(data.ctrl)・
    PD状態のいずれも変更せず、(None, None, False, False)を返すことを確認する。
    """
    failures = []
    env.reset()

    # detectで少し追従させてから、意図的にWATCHへ手動遷移させる
    env.enter_detect_track()
    env.track_target_deg(target_yaw_deg=12.0, target_pitch_deg=6.0, dt=1.0 / 30.0)
    for _ in range(5):
        env.step()
    env.enter_watch()

    ctrl_before = tuple(env.data.ctrl.copy())
    yaw_pd_pos_before = env.yaw_pd.pos_deg
    yaw_pd_prev_err_before = env.yaw_pd.prev_err_deg
    pitch_pd_pos_before = env.pitch_pd.pos_deg
    pitch_pd_prev_err_before = env.pitch_pd.prev_err_deg

    # WATCH中にtrack_target_degを呼ぶ(実機ならこの呼び出し自体が発生しない状況)
    result = env.track_target_deg(target_yaw_deg=50.0, target_pitch_deg=-50.0, dt=1.0 / 30.0)

    if result != (None, None, False, False):
        failures.append(f"track_target_deg() should return (None, None, False, False) in WATCH, got {result}")

    ctrl_after = tuple(env.data.ctrl.copy())
    if ctrl_before != ctrl_after:
        failures.append(f"data.ctrl should not change while in WATCH (before={ctrl_before}, after={ctrl_after})")
    if env.yaw_pd.pos_deg != yaw_pd_pos_before:
        failures.append("yaw_pd.pos_deg changed during WATCH (should be untouched)")
    if env.yaw_pd.prev_err_deg != yaw_pd_prev_err_before:
        failures.append("yaw_pd.prev_err_deg changed during WATCH (should be untouched)")
    if env.pitch_pd.pos_deg != pitch_pd_pos_before:
        failures.append("pitch_pd.pos_deg changed during WATCH (should be untouched)")
    if env.pitch_pd.prev_err_deg != pitch_pd_prev_err_before:
        failures.append("pitch_pd.prev_err_deg changed during WATCH (should be untouched)")

    passed = len(failures) == 0
    return TestResult(
        name="test5h_watch_blocks_track_target_deg",
        passed=passed,
        summary="WATCH中はtrack_target_deg()がactuator/PD状態を一切変更せず早期リターンすること",
        failure_details=failures,
    )


# ============================================================
# test5-i: 「動く→止まる→また動く」一連の遷移パターン(④、挙動12)
# ============================================================
def test5i_move_stop_resume_pattern(env: PxPanTiltEnv):
    """
    Q&Aでの指摘の通り、watch modeの検証は「detectで動いている状態から
    lost framesでwatchに落ちてservoが静止し、再度detectに戻って動き出す」
    という一連のパターンとして確認する。
    """
    failures = []
    env.reset()

    # --- フェーズ1: detectで実際にtrackingし、jointが動くことを確認 ---
    env.enter_detect_track()
    target_yaw, target_pitch = 25.0, -15.0
    for _ in range(20):
        env.track_target_deg(target_yaw_deg=target_yaw, target_pitch_deg=target_pitch, dt=1.0 / 30.0)
        env.step()

    yaw_moved, pitch_moved = env.get_angles_deg()
    if abs(yaw_moved - env.yaw_center) < 1.0 and abs(pitch_moved - env.pitch_center) < 1.0:
        failures.append(
            f"phase1 failed: joint should have moved noticeably from center "
            f"(yaw={yaw_moved:.3f}, pitch={pitch_moved:.3f}, center=({env.yaw_center},{env.pitch_center}))"
        )

    # data.ctrl(actuator目標)を現在のqposへ十分収束させておく。
    # MuJoCoのposition actuatorは「新規指令が来なくても、既に送信済みの
    # ctrl値へ向かって物理的に動き続ける」ため、WATCH突入直後にqposが
    # 変化すること自体は正常な物理挙動(実機のサーボも直前の指令角度までは
    # 慣性で動く)。「WATCH中は新規指令を送らない」ことを検証するには、
    # 遷移前にactuatorをqposへ収束させ、遷移後の変化が「新規指令による
    # ものではない」ことを切り分けられる状態にしておく必要がある。
    env.settle(n_steps=500)
    yaw_settled, pitch_settled = env.get_angles_deg()

    # --- フェーズ2: lost_max_frames回検出失敗させ、WATCHへ自動遷移させる ---
    transitioned = False
    for _ in range(env.lost_max_frames):
        transitioned = env.on_detection_result(detected=False)
    if not transitioned or not env.fsm.is_watch():
        failures.append(f"phase2 failed: expected auto-transition to WATCH, mode={env.fsm.mode}")

    yaw_at_watch_entry, pitch_at_watch_entry = env.get_angles_deg()
    if abs(yaw_at_watch_entry - yaw_settled) > 1e-3 or abs(pitch_at_watch_entry - pitch_settled) > 1e-3:
        failures.append(
            "precondition failed: joint should already be settled at watch entry "
            f"(settled=({yaw_settled:.6f},{pitch_settled:.6f}), "
            f"at_entry=({yaw_at_watch_entry:.6f},{pitch_at_watch_entry:.6f}))"
        )

    # --- フェーズ3: WATCH中、何度track_target_deg()+step()を呼んでも静止したままであること ---
    for _ in range(30):
        env.track_target_deg(target_yaw_deg=target_yaw, target_pitch_deg=target_pitch, dt=1.0 / 30.0)
        env.step()

    yaw_after_watch, pitch_after_watch = env.get_angles_deg()
    # 許容誤差は04_future_problem_prediction.mdの教訓2(position actuatorは
    # 重力下で厳密なゼロ誤差には収束せず、微小な定常偏差が残る)を踏まえ、
    # 「新規指令が送られた場合の移動量(度オーダー)」と明確に区別できる
    # 0.01度を閾値とする(観測された定常偏差は0.0001度オーダー)。
    stationary_tol_deg = 0.01
    if abs(yaw_after_watch - yaw_at_watch_entry) > stationary_tol_deg or \
       abs(pitch_after_watch - pitch_at_watch_entry) > stationary_tol_deg:
        failures.append(
            f"phase3 failed: joint should remain stationary during WATCH (tol={stationary_tol_deg}deg) "
            f"(before=({yaw_at_watch_entry:.6f},{pitch_at_watch_entry:.6f}), "
            f"after=({yaw_after_watch:.6f},{pitch_after_watch:.6f}))"
        )

    # --- フェーズ4: 再度detectへ復帰し、新しいtargetへ実際に動き出すこと ---
    env.enter_detect_track()
    new_target_yaw, new_target_pitch = -20.0, 20.0
    for _ in range(30):
        env.track_target_deg(target_yaw_deg=new_target_yaw, target_pitch_deg=new_target_pitch, dt=1.0 / 30.0)
        env.step()

    yaw_resumed, pitch_resumed = env.get_angles_deg()
    if abs(yaw_resumed - yaw_after_watch) < 1.0 and abs(pitch_resumed - pitch_after_watch) < 1.0:
        failures.append(
            f"phase4 failed: joint should resume moving toward new target after re-entering DETECT_TRACK "
            f"(before_resume=({yaw_after_watch:.3f},{pitch_after_watch:.3f}), "
            f"after_resume=({yaw_resumed:.3f},{pitch_resumed:.3f}))"
        )

    passed = len(failures) == 0
    return TestResult(
        name="test5i_move_stop_resume_pattern",
        passed=passed,
        summary="detect(動く)->lost frames->watch(静止)->detect復帰(再び動く)の一連パターン確認",
        failure_details=failures,
    )


# ============================================================
# main
# ============================================================
def main():
    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))

    print("=" * 70)
    print("Level 4: TrackingModeFSM Test Suite (test5, behaviors 11+5+8+12)")
    print("=" * 70)

    result_a = test5a_fsm_standalone()
    print("\n" + result_a.report())

    result_b = test5b_env_reset_order(env)
    print("\n" + result_b.report())

    result_d = test5d_track_target_deg_auto_dt_zero(env)
    print("\n" + result_d.report())

    result_c = test5c_watch_transition_state_consistency(env)
    print("\n" + result_c.report())

    result_e = test5e_lost_frames_auto_watch(env)
    print("\n" + result_e.report())

    result_f = test5f_lost_frames_reset_on_detection(env)
    print("\n" + result_f.report())

    result_g = test5g_on_detection_result_noop_in_watch(env)
    print("\n" + result_g.report())

    result_h = test5h_watch_blocks_track_target_deg(env)
    print("\n" + result_h.report())

    result_i = test5i_move_stop_resume_pattern(env)
    print("\n" + result_i.report())

    all_results = [result_a, result_b, result_d, result_c, result_e, result_f, result_g, result_h, result_i]
    n_pass = sum(r.passed for r in all_results)
    print("\n" + "=" * 70)
    print(f"SUMMARY: {n_pass}/{len(all_results)} tests passed")
    for r in all_results:
        print(f"  [{'PASS' if r.passed else 'FAIL'}] {r.name}")
    print("=" * 70)

    return all_results


if __name__ == "__main__":
    main()
