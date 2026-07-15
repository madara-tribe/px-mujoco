"""
tracking_mode_fsm.py — TrackingModeFSM

挙動11(watch/detect 2-mode遷移)の骨格実装。①のスコープ。

実機対応:
  inference.h の `enum class Mode { WATCH, DETECT_TRACK };`
  inference.cpp の `enterWatchMode()` / `enterDetectTrackMode()`

設計方針:
  - PxPanTiltEnv・AxisPdControllerからは独立させる。
    FSMは「状態(state)の管理」のみを行い、PDのreset()呼び出しや
    data.ctrlの操作はEnv側がFSMの状態/戻り値を見て行う
    (責務分離。04_future_problem_prediction.md の2-Aで指摘された
    「mode遷移時のリセット順序」を、FSM単体でテスト可能にするため)。
  - 状態は "watch" | "detect" の文字列で表現する(実機のenum class Modeに対応)。
  - 挙動5(初回DETECTフレームのD項スキップ)は、enter_detect_track()内で
    first_detect_frame フラグを立てることで連動させる。フラグの消費
    (dt=0をPDへ渡す)はEnv側の責務とする。
  - 挙動8(ロストフレームによるWATCH復帰)・挙動12(WATCHのfps間引き)は
    このクラスに後続ステップで追加していく(①では骨格のみ)。

①のスコープ外(後続ステップで追加予定):
  - on_detection_result(): 挙動8の no_det_frames カウント・自動遷移
  - should_run_control():  挙動12の fps間引き判定

③での変更点(挙動8: ロストフレームによるWATCH復帰):
  - lost_max_frames をコンストラクタ引数として追加(実機inference.hのデフォルト15、
    run.sh実運用値は30。control_params.yamlのfuture_params.lost_max_framesに対応)。
  - on_detection_result(detected: bool) を新設。実機inference.cppの
    メインループ分岐(検出あり:no_det_frames_=0、検出なし:カウントアップし
    閾値到達でenterWatchMode())を直訳する。
  - このメソッドはDETECT_TRACK状態でのみ意味を持つ(実機もdetectモード中の
    ループでのみこの分岐を通る)。WATCH状態で呼ばれても何もしない
    (実機はWATCH中このコールバック自体が呼ばれないため、シム側も
    無害化することで呼び出し側の条件分岐を簡潔にする)。

④での設計判断(挙動12: WATCHモードのfps間引き):
  - 実機コードを読み直すと、WATCH中のfps間引き(watch_fps=2.0)は
    「YOLO推論ループ自体をスキップし、フレームを読み捨てる」処理であり、
    perception層(YOLO推論の実行頻度)の話である。MuJoCo側に対応する
    概念は存在しない(mj_stepの粒度でフレーム読み捨てをシミュレートしても
    actuator/jointの動作確認には寄与しない)。
  - 01_behavior_source_reference.md の検証区分方針(confidence閾値・
    面積フィルタは実機区分)と同じ理由で、fps値そのもの(2Hz)の
    間引きロジックはFSMに実装しない。
  - MuJoCo側が担うべきなのは、その結果として観測される事実
    「WATCH中はactuatorへ新規指令が一切送られない」のみ。
    これは should_run_control() のような新規メソッドを追加せず、
    既存の is_watch() を Env.track_target_deg() 側で参照させることで
    表現する(FSMへの変更は不要。Env側のみ変更)。
"""

from enum import Enum


class Mode(str, Enum):
    WATCH = "watch"
    DETECT_TRACK = "detect"


class TrackingModeFSM:
    """
    inference.h の Mode enum + enterWatchMode()/enterDetectTrackMode() の直訳。

    このクラス単体はactuator/PDには一切触れない。状態のみを保持する。
    Env側は以下のように使う想定:

        fsm = TrackingModeFSM()
        fsm.enter_detect_track()
        if fsm.first_detect_frame:
            dt = 0.0
            fsm.consume_first_detect_frame()
        ...
    """

    def __init__(self, lost_max_frames: int = 30):
        """
        lost_max_frames: 検出なしフレームが何回連続でWATCHへ復帰するか。
                          実機inference.hのデフォルトは15だが、run.sh実運用値の
                          30をこのクラスの既定値とする(control_params.yaml
                          future_params.lost_max_frames = 30 に合わせる)。
        """
        # 実機のデフォルト初期状態は Mode::WATCH (inference.h: `Mode mode_{Mode::WATCH};`)
        self.mode: Mode = Mode.WATCH
        self.no_det_frames: int = 0
        self.first_detect_frame: bool = False
        self.lost_max_frames = lost_max_frames

    def reset(self):
        """
        Env.reset()から呼ばれる想定。実機に「起動時に呼ばれるreset」に相当する
        処理は存在しないが(起動時は静的初期化のみ)、シム側でepisodeを
        繰り返す都合上、明示的なreset()を用意する。
        WATCH状態に戻す。
        """
        self.mode = Mode.WATCH
        self.no_det_frames = 0
        self.first_detect_frame = False

    def enter_watch(self):
        """
        inference.cpp の enterWatchMode() 直訳:

            void OnnxInferenceNode::enterWatchMode() {
              mode_ = Mode::WATCH;
              no_det_frames_ = 0;
              wakeup_requested_.store(false);
              publishWakeup(false);
            }

        wakeup_requested_/publishWakeup()はROS2通信層のため対象外
        (01_behavior_source_reference.md の検証区分方針により実機側)。
        mode_ と no_det_frames_ のリセットのみをMuJoCo側の対象とする。
        """
        self.mode = Mode.WATCH
        self.no_det_frames = 0

    def enter_detect_track(self):
        """
        inference.cpp の enterDetectTrackMode() 直訳:

            void OnnxInferenceNode::enterDetectTrackMode() {
              mode_ = Mode::DETECT_TRACK;
              no_det_frames_ = 0;
              first_detect_frame_ = true;
              yaw_axis_.reset(0.0);
              pitch_axis_.reset(0.0);
              publishWakeup(true);
            }

        yaw_axis_.reset(0.0)/pitch_axis_.reset(0.0) はPDController側の責務のため
        ここでは呼ばない。Env側が本メソッド呼び出し後、
        yaw_pd.reset(0.0) / pitch_pd.reset(0.0) を明示的に呼ぶこと。
        publishWakeup()はROS2通信層のため対象外。
        """
        self.mode = Mode.DETECT_TRACK
        self.no_det_frames = 0
        self.first_detect_frame = True

    def consume_first_detect_frame(self) -> bool:
        """
        first_detect_frame フラグを読み取り、falseにリセットして返す。
        「1回だけdt=0を使う」という一回性の消費を明示的にするためのメソッド。

        戻り値: 消費前のfirst_detect_frame値
                (True なら今回のフレームはdt=0を使うべき)
        """
        was_first = self.first_detect_frame
        self.first_detect_frame = False
        return was_first

    def is_watch(self) -> bool:
        return self.mode == Mode.WATCH

    def is_detect(self) -> bool:
        return self.mode == Mode.DETECT_TRACK

    def on_detection_result(self, detected: bool) -> bool:
        """
        挙動8: ロストフレームによるWATCH復帰。

        inference.cpp のメインループ分岐の直訳:

            if (box.area() >= kMinBoxArea) {
              no_det_frames_ = 0;
              ...
            } else {
              no_det_frames_++;
              if (no_det_frames_ >= lost_max_frames_)
                enterWatchMode();
            }

        box.area() >= kMinBoxArea (挙動10, 実機側判定)の結果を
        detected という単純なbool値として引数で受け取る
        (01_behavior_source_reference.md の検証区分方針により、
        面積フィルタ自体の判定ロジックは実機側の対象。MuJoCoは
        「検出成功/失敗」というイベントの結果だけを受け取ればよい)。

        WATCH状態で呼ばれた場合は何もしない(実機のメインループは
        WATCH中この分岐自体を通らないため)。

        戻り値: このメソッド呼び出しの結果、enterWatchMode()相当の
                自動遷移が発生した場合はTrue、それ以外はFalse。
                (呼び出し側がactuator側の後処理を挟みたい場合のフック)
        """
        if not self.is_detect():
            return False

        if detected:
            self.no_det_frames = 0
            return False

        self.no_det_frames += 1
        if self.no_det_frames >= self.lost_max_frames:
            self.enter_watch()
            return True
        return False
