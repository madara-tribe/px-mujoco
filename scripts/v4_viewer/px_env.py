"""
px_env.py — PxPanTiltEnv

Pattern-B (2軸パンチルト) の MuJoCo 環境クラス。

設計方針 (DeepMind MuJoCo Playground の MjxEnv パターンを踏襲):
  - XMLはモデル構造(剛体・関節・アクチュエータ)のみを定義し、変更しない。
  - 制御ロジック・ノイズ・遅延・状態遷移(mode switching)など、
    「挙動」はすべてこのクラスのメソッドとして追加していく。
  - 状態(遅延バッファ、mode、前回誤差など)は self に集約する。
  - 光学変換(pixel<->角度)は optics.py に分離し、このファイルはそれを
    呼び出すだけに留める(役割分離。v3での変更点)。

実装範囲(Level 1/2/3 + 光学変換):
  - reset(): 中心角に初期化
  - set_target_deg(): 指定角度への直接指令 (PD制御なし、位置指令のみ。Level1/2用)
  - track_target_deg(): PD制御によるtarget追従
  - step(): 1ステップ物理を進める
  - get_angles_deg(): 現在角度を取得(度)

PD制御 (AxisPdController) は axis_pd_controller.hpp を直接踏襲:
  - Kp, Kd, max_step_deg, min_deg/max_deg/center_deg
  - dt正規化 (dt=0時はD項スキップ = 実機の first_detect_frame_ 相当)
  - レート制限 (delta_degをmax_step_degでクランプ = 実機のペンデュラム抑制相当)
  - 角度飽和 (最終出力をmin_deg/max_degでクランプ)
  I項は実機同様に持たない(意図的な非実装。教訓2型の定常偏差を再現するため)。

v3での変更点(光学変換の追加):
  - track_target_deg() は「目標角度(center基準の相対値)」を受け取る点は
    v2と同じインターフェースを維持しつつ、内部で optics.py の
    simulate_detection() を経由して「pixel検出を模した観測角度」に
    変換してからPD制御に渡すようになった(use_optics=True の場合)。
  - apply_detection_noise() は角度空間でノイズを乗せていたv2の実装から、
    pixel空間でノイズを乗せる optics.simulate_detection() 経由に変更。
    実機のノイズ発生源(YOLO bbox検出のpixelばらつき)と単位を一致させ、
    画面端に近いほど同じpixelノイズが大きな角度ノイズになる非線形性を
    再現する(reports/05_optics_reproduction_feasibility.md 参照)。

追加した検証用フック (test1-4で使用):
  - servo_delay_buffer: サーボ書き込み遅延の再現(Nステップ分の指令を遅延させる)
  - apply_detection_noise/dropout: 検出ノイズ・ドロップアウトの注入
  これらはすべて「挙動」としてメソッド単位で追加しており、XMLは一切変更していない。

v4での変更点(TrackingModeFSMの統合、①のスコープ):
  - self.mode / self.no_det_frames のスタブ実装を廃止し、
    tracking_mode_fsm.TrackingModeFSM に置き換えた。
  - FSMは状態管理のみを担当し、PDController.reset()の呼び出しやdt=0の
    受け渡しはEnv側(このファイル)が責務を持つ(責務分離の方針。
    04_future_problem_prediction.md 2-A「mode遷移時のリセット順序」対策)。
  - enter_detect_track() / enter_watch() をEnvに新設し、内部でFSMの遷移と
    連動する処理(PDリセット等)をまとめて実行する。
  - track_target_deg() はFSMの状態を直接は見ない(①のスコープでは呼び出し側が
    明示的にdt/first_detect_frameを管理する。挙動8・12の統合時に
    FSM連動をtrack_target_deg内に組み込む予定)。
"""

from pathlib import Path

import mujoco
import numpy as np
import yaml

from optics import CameraCalibration, simulate_detection
from tracking_mode_fsm import TrackingModeFSM


class AxisPdController:
    """
    axis_pd_controller.hpp の直訳。1軸分のPD制御状態を持つ。
    実機同様、積分項(I)は意図的に持たない。
    """

    def __init__(self, kp: float, kd: float, max_step_deg: float,
                 center_deg: float, min_deg: float, max_deg: float):
        self.Kp = kp
        self.Kd = kd
        self.max_step_deg = max_step_deg
        self.center_deg = center_deg
        self.min_deg = min_deg
        self.max_deg = max_deg

        self.pos_deg = 0.0        # center_degからの相対位置
        self.prev_err_deg = 0.0

    def reset(self, pos_deg: float = 0.0):
        """axis_pd_controller.hppのreset()相当。mode遷移時に呼ばれる。"""
        self.pos_deg = pos_deg
        self.prev_err_deg = 0.0

    def update_from_error_deg(self, err_deg: float, dt: float = 1.0) -> float:
        """
        誤差(度)からサーボ出力角度(度, 絶対値)を計算する。
        dt=0.0は「初回DETECTフレーム」を表し、D項をスキップする
        (inference.cpp の first_detect_frame_ ロジックと同一)。
        """
        d_term = self.Kd * (err_deg - self.prev_err_deg) / dt if dt > 0.0 else 0.0
        delta_deg = self.Kp * err_deg + d_term
        delta_deg = float(np.clip(delta_deg, -self.max_step_deg, self.max_step_deg))

        self.pos_deg += delta_deg
        self.prev_err_deg = err_deg

        servo_deg = self.center_deg + self.pos_deg
        return float(np.clip(servo_deg, self.min_deg, self.max_deg))

    def is_in_dead_zone(self, dead_zone_deg: float) -> bool:
        """inference.cpp のデッドゾーン判定(prev_err_degベース)と同一。"""
        return abs(self.prev_err_deg) < dead_zone_deg


class PxPanTiltEnv:
    def __init__(self, xml_path: str, params_path: str, camera_calib_path: str | None = None):
        self.model = mujoco.MjModel.from_xml_path(str(xml_path))
        self.data = mujoco.MjData(self.model)

        with open(params_path, "r", encoding="utf-8") as f:
            self.params = yaml.safe_load(f)

        # --- 光学変換用のカメラ校正(v3で追加)。指定がなければデフォルトパスを試す ---
        if camera_calib_path is None:
            camera_calib_path = str(Path(params_path).parent / "camera_calib.yaml")
        self.calib = CameraCalibration.from_yaml(camera_calib_path)

        # --- run.sh 準拠のcenter/min/maxをパラメータファイルから読む ---
        self.yaw_center = self.params["axes"]["yaw"]["center_deg"]
        self.yaw_min = self.params["axes"]["yaw"]["min_deg"]
        self.yaw_max = self.params["axes"]["yaw"]["max_deg"]

        self.pitch_center = self.params["axes"]["pitch"]["center_deg"]
        self.pitch_min = self.params["axes"]["pitch"]["min_deg"]
        self.pitch_max = self.params["axes"]["pitch"]["max_deg"]

        # --- PD制御パラメータ(future_paramsから読む。無ければ既定値) ---
        fp = self.params.get("future_params", {})
        pd = fp.get("pd_gains", {})
        self.kp_yaw = pd.get("kp_yaw", 0.3)
        self.kd_yaw = pd.get("kd_yaw", 0.2)
        self.kp_pitch = pd.get("kp_pitch", 0.3)
        self.kd_pitch = pd.get("kd_pitch", 0.2)
        self.max_step_deg = fp.get("max_step_deg", 3.0)
        self.dead_zone_deg = fp.get("dead_zone_deg", {}).get("yaw", 1.5)
        self.lost_max_frames = fp.get("lost_max_frames", 30)  # 挙動8(③): run.sh実運用値

        # joint / actuator index をキャッシュ
        self._yaw_qpos_adr = self.model.joint("yaw").qposadr[0]
        self._pitch_qpos_adr = self.model.joint("pitch").qposadr[0]
        self._yaw_ctrl_idx = 0   # actuator順: servo_x=0, servo_y=1 (XML定義順)
        self._pitch_ctrl_idx = 1

        # --- PD制御器(1軸ずつ独立。inference.cppの軸独立設計と同一) ---
        self.yaw_pd = AxisPdController(
            self.kp_yaw, self.kd_yaw, self.max_step_deg,
            self.yaw_center, self.yaw_min, self.yaw_max)
        self.pitch_pd = AxisPdController(
            self.kp_pitch, self.kd_pitch, self.max_step_deg,
            self.pitch_center, self.pitch_min, self.pitch_max)

        # --- mode状態(v4: TrackingModeFSMに置き換え。挙動11+8) ---
        self.fsm = TrackingModeFSM(lost_max_frames=self.lost_max_frames)
        # FSMベースのmode遷移が一度でも使われたかどうか(④)。
        # test1〜4のようなFSM非依存の呼び出しでtrack_target_deg()の
        # 早期リターンが誤発火しないようにするためのガード。
        self._fsm_enabled = False

        # --- サーボ書き込み遅延バッファ(test3用) ---
        self._servo_delay_steps = 0
        self._yaw_cmd_buffer = []
        self._pitch_cmd_buffer = []

    def reset(self):
        """中心角(center_deg)に初期化する。PD制御器・FSMもリセットする。"""
        mujoco.mj_resetData(self.model, self.data)
        self.set_target_deg(self.yaw_center, self.pitch_center)
        mujoco.mj_forward(self.model, self.data)
        self.yaw_pd.reset(0.0)
        self.pitch_pd.reset(0.0)
        self.fsm.reset()
        self._fsm_enabled = False
        self._yaw_cmd_buffer.clear()
        self._pitch_cmd_buffer.clear()

    def set_target_deg(self, yaw_deg: float, pitch_deg: float):
        """
        目標角度を直接actuatorに指令する(位置指令、PD制御なし)。
        Level1/2用。範囲外の値はrun.sh準拠のmin/maxでクランプする。
        """
        yaw_clamped = float(np.clip(yaw_deg, self.yaw_min, self.yaw_max))
        pitch_clamped = float(np.clip(pitch_deg, self.pitch_min, self.pitch_max))
        self.data.ctrl[self._yaw_ctrl_idx] = np.radians(yaw_clamped)
        self.data.ctrl[self._pitch_ctrl_idx] = np.radians(pitch_clamped)

    def configure_servo_delay(self, delay_steps: int):
        """サーボ書き込み遅延をNステップ分設定する(test3用)。0で遅延無し。"""
        self._servo_delay_steps = delay_steps
        self._yaw_cmd_buffer.clear()
        self._pitch_cmd_buffer.clear()

    def enter_detect_track(self):
        """
        WATCH -> DETECT_TRACK への遷移(挙動11)。
        実機の enterDetectTrackMode() は以下をまとめて実行する:
          mode_ = DETECT_TRACK; no_det_frames_ = 0; first_detect_frame_ = true;
          yaw_axis_.reset(0.0); pitch_axis_.reset(0.0); publishWakeup(true);

        FSM(状態管理)とPDController(制御状態)は責務が異なるクラスのため、
        Env側がこの2つを「同じ遷移イベント」として1メソッド内で順序通り呼ぶ。
        呼び出し順序: FSM遷移 -> PDリセット
        (04_future_problem_prediction.md 2-A で指摘された、リセット順序の
        取り違えを避けるため、この順序をこのメソッド1箇所に固定する)。
        publishWakeup()はROS2通信層のため対象外。
        """
        self.fsm.enter_detect_track()
        self.yaw_pd.reset(0.0)
        self.pitch_pd.reset(0.0)
        self._fsm_enabled = True

    def enter_watch(self):
        """
        DETECT_TRACK -> WATCH への遷移(挙動11)。
        実機の enterWatchMode() 相当:
          mode_ = WATCH; no_det_frames_ = 0; wakeup_requested_ = false; publishWakeup(false);
        wakeup_requested_/publishWakeup()はROS2通信層のため対象外。
        PDController側のリセットは行わない(実機コードにも存在しない。
        watch復帰時にpos_deg/prev_err_degは保持されたままになる=直前の
        追従状態を引き継ぐ、という実機の設計をそのまま踏襲する)。
        """
        self.fsm.enter_watch()
        self._fsm_enabled = True

    def on_detection_result(self, detected: bool) -> bool:
        """
        挙動8: ロストフレームによるWATCH復帰。

        実機inference.cppのメインループ分岐をEnv経由でFSMへ委譲する。
        呼び出し側(test5相当のループ)は、検出成功/失敗のイベントが
        起きるたびに毎フレームこれを呼ぶ想定(1フレーム=1回のYOLO推論に相当)。

        DETECT_TRACK状態でのみ意味を持つ。WATCH中に呼んでも何もしない
        (FSM側で無害化済み)。

        戻り値: このフレームでenter_watch()相当の自動遷移が発生したらTrue。
                ④により、WATCHへ遷移した場合、以降のtrack_target_deg()
                呼び出しは自動的に早期リターンしactuatorへ指令を出さなく
                なる(呼び出し側が個別に判断する必要はない)。
        """
        return self.fsm.on_detection_result(detected)

    def track_target_deg(self, target_yaw_deg: float, target_pitch_deg: float,
                          dt: float, apply_dead_zone: bool = False,
                          use_optics: bool = False, pixel_noise_sigma_px: float = 0.0,
                          rng: np.random.Generator | None = None):
        """
        PD制御でtargetを追従する。axis_pd_controller.hppのロジックを直接使用。
        サーボ遅延バッファが設定されている場合は、計算した指令をNステップ遅延させてから
        actuatorに反映する(px3_servo_node.cppのSERVO_ARRIVAL_WAIT_MS相当を模す)。

        use_optics=True の場合(v3で追加):
          「目標角度 - 現在角度」の真の誤差を、optics.simulate_detection()経由で
          一度pixel空間に変換し、実機と同じ cv2.undistortPoints で角度に戻した
          「観測誤差」をPD制御に渡す。pixel_noise_sigma_px>0ならpixel空間で
          ガウスノイズも注入する(実機のYOLO bbox検出ばらつきに相当)。
          use_optics=False(既定)の場合はv2までと同じ、角度の直接差分を使う。

        戻り値: (yaw_cmd_deg, pitch_cmd_deg, yaw_in_dead_zone, pitch_in_dead_zone)
                WATCH状態で呼ばれた場合は (None, None, False, False) を返す
                (④, 下記参照)。

        v4 ②スコープでの変更点(挙動5の統合):
          引数dtは「呼び出し側の既定値」として扱われるが、
          self.fsm.first_detect_frame が True の場合(=直前に
          enter_detect_track()が呼ばれ、まだ1回もtrack_target_deg()が
          実行されていない場合)、このメソッド内部で自動的に dt=0.0 に
          上書きし、self.fsm.consume_first_detect_frame() でフラグを
          消費する(一回性)。
          呼び出し側はFSMの状態を意識する必要がなくなり、実機の
          enterDetectTrackMode() -> 次フレームでdt不定 という流れを
          このメソッド単体で再現する。
          FSMを使わない呼び出し(test1〜4など、enter_detect_track()を
          一度も呼ばないケース)では first_detect_frame は常にFalseの
          ままなので、引数dtがそのまま使われ、既存挙動に影響しない。

        v4 ④スコープでの変更点(挙動12: WATCHモードのfps間引きに伴う帰結):
          実機inference.cppはWATCH中、computeAxisDeg()(=PD更新)自体を
          呼ばない(フレームを読み捨てるのみ)。このメソッドもそれに合わせ、
          self.fsm.is_watch() が True の場合は最初で早期リターンし、
          PD状態(pos_deg/prev_err_deg)・data.ctrl のいずれも更新しない。
          fps値そのもの(watch_fps=2.0)はYOLO推論頻度というperception層の
          パラメータであり、MuJoCo側の対象外(01_behavior_source_reference.md
          の検証区分方針)。MuJoCoが担うのは「WATCH中は指令を出さない」
          という結果のみ。
          FSMを使わない呼び出し(test1〜4)はenter_detect_track()を一度も
          呼ばないため self.fsm.mode は初期値のWATCHのままだが、これらの
          testはFSM導入前の実装と同じく track_target_deg() を直接使う
          運用であり、fsm.mode を参照させると全て早期リターンしてしまい
          既存挙動を壊す。そのため、この早期リターンは
          self.fsm.is_watch() and self._fsm_enabled の両方を条件とする
          (_fsm_enabled は enter_detect_track()/enter_watch() が一度でも
          呼ばれた時点でTrueになるフラグ。test1〜4はこれらを呼ばないため
          _fsm_enabled=Falseのままとなり、影響を受けない)。
        """
        if self.fsm.is_watch() and self._fsm_enabled:
            return None, None, False, False

        yaw_now, pitch_now = self.get_angles_deg()
        true_yaw_err = target_yaw_deg - yaw_now
        true_pitch_err = target_pitch_deg - pitch_now

        # 挙動5: FSMがfirst_detect_frameを立てている場合、dtを強制的に0.0にする
        # (enterDetectTrackMode()直後の1フレームはD項をスキップする実機挙動)。
        # 一回性の消費: 次回呼び出しからは引数dtがそのまま使われる。
        if self.fsm.consume_first_detect_frame():
            dt = 0.0

        if use_optics:
            # 真の誤差(center基準の相対オフセットとして) -> pixel -> [ノイズ] -> 角度(観測誤差)
            yaw_err, pitch_err = simulate_detection(
                self.calib, true_yaw_err, true_pitch_err,
                pixel_noise_sigma_px=pixel_noise_sigma_px, rng=rng)
        else:
            yaw_err, pitch_err = true_yaw_err, true_pitch_err

        yaw_cmd = self.yaw_pd.update_from_error_deg(yaw_err, dt)
        pitch_cmd = self.pitch_pd.update_from_error_deg(pitch_err, dt)

        yaw_dead = self.yaw_pd.is_in_dead_zone(self.dead_zone_deg) if apply_dead_zone else False
        pitch_dead = self.pitch_pd.is_in_dead_zone(self.dead_zone_deg) if apply_dead_zone else False

        if self._servo_delay_steps > 0:
            self._yaw_cmd_buffer.append(yaw_cmd)
            self._pitch_cmd_buffer.append(pitch_cmd)
            if len(self._yaw_cmd_buffer) > self._servo_delay_steps:
                applied_yaw = self._yaw_cmd_buffer.pop(0)
                applied_pitch = self._pitch_cmd_buffer.pop(0)
            else:
                # バッファが埋まるまでは指令を送らない(=直前の状態を保持)
                applied_yaw, applied_pitch = None, None
        else:
            applied_yaw, applied_pitch = yaw_cmd, pitch_cmd

        # デッドゾーン内、またはバッファ待機中は指令を送らない(= data.ctrl保持)
        if applied_yaw is not None and not yaw_dead:
            self.data.ctrl[self._yaw_ctrl_idx] = np.radians(applied_yaw)
        if applied_pitch is not None and not pitch_dead:
            self.data.ctrl[self._pitch_ctrl_idx] = np.radians(applied_pitch)

        return yaw_cmd, pitch_cmd, yaw_dead, pitch_dead

    def step(self):
        """1ステップ物理を進める。"""
        mujoco.mj_step(self.model, self.data)

    def get_angles_deg(self):
        """現在のyaw/pitch角度を度で返す。"""
        yaw_deg = np.degrees(self.data.qpos[self._yaw_qpos_adr])
        pitch_deg = np.degrees(self.data.qpos[self._pitch_qpos_adr])
        return yaw_deg, pitch_deg

    def settle(self, n_steps: int = 500):
        """指令を変えずにn_stepsだけ進め、静止状態に収束させる。"""
        for _ in range(n_steps):
            self.step()


def apply_detection_noise(target_deg: float, sigma_deg: float, rng: np.random.Generator) -> float:
    """
    [v2実装、非推奨] 角度空間で直接ノイズを乗せる旧実装。

    v3では track_target_deg(..., use_optics=True, pixel_noise_sigma_px=...) を
    使うことで、pixel空間(実機のYOLO bbox検出ばらつきに相当する単位)で
    ノイズを注入し、光学変換(optics.py)を経由して角度ノイズに変換する方式に
    切り替えた。角度空間で一律にノイズを乗せる本関数は、画面端で非線形に
    増幅されるという実機の性質を再現できないため、新規実装では
    optics経由の方式を使うこと。後方互換のために残してある。
    """
    return target_deg + rng.normal(0.0, sigma_deg)


def apply_detection_dropout(rng: np.random.Generator, dropout_prob: float) -> bool:
    """検出ドロップアウト(bbox消失)を確率的に発生させる。test2用。
    Trueならこのフレームは検出なし(=前回targetを保持する側で処理する)。
    これは光学変換とは無関係(bbox自体が存在しない事象)のため、v3でも変更なし。"""
    return rng.random() < dropout_prob

