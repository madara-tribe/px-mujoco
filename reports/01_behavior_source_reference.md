# Pattern-B 実機コード 挙動一覧レポート

**対象コード**: `SensorFireld-PX4.3-day-pattern-B`
**作成日**: 2026-07-04
**更新日**: 2026-07-15 — MuJoCo/実機 検証区分を追加
**位置づけ**: MuJoCoシミュレーションで段階的に再現していく挙動と、その実機コード上の該当箇所を対応させたリファレンス。シミュレーション実装が進むごとに追記していく前提の一次資料。

---

## 検証区分の方針(2026-07-15追記)

MuJoCoは「ロボットの正常動作を事前に確認するシミュレータ」として位置づけ、役割を以下の4点に限定する。

- actuatorが期待通りに動作するか
- jointが期待通りに動作するか
- 制御ロジックによって想定した動きになるか
- モード遷移(state transition)が正しく行われるか

異常系・ハードウェア固有現象(センサーデバウンス、通信タイムアウト、起動シーケンス等)まで再現するとモデルが過度に複雑化するため、これらは**実機側で検証する対象**として明確に切り分ける。

servoが動くのはdetectモードのみであるため、MuJoCo対象はdetectモードでの制御・動作確認と、watch⇄detectのstate transition(「動いている状態から止まる→また動き出す」パターン)に集約される。

---

## 目次(検証区分つき)

| # | 挙動 | ファイル | 検証区分 | 状態 |
|---|---|---|---|---|
| 1 | PD制御(積分項なし) | axis_pd_controller.hpp | 🔵 MuJoCo | 実装済み |
| 2 | dt正規化 | axis_pd_controller.hpp | 🔵 MuJoCo | 実装済み |
| 3 | レート制限(振動抑制) | axis_pd_controller.hpp | 🔵 MuJoCo | 実装済み |
| 4 | 角度飽和(クランプ) | axis_pd_controller.hpp | 🔵 MuJoCo | 実装済み |
| 5 | 初回DETECTフレームのD項スキップ | inference.cpp | 🔵 MuJoCo | スタブ止まり |
| 6 | 軸独立ターゲット設計(yaw/pitch分離) | inference.cpp | 🔵 MuJoCo | 実装済み |
| 7 | デッドゾーン | inference.h / inference.cpp | 🔵 MuJoCo | 実装済み(未使用) |
| 8 | ロストフレームによるWATCH復帰 | inference.h / inference.cpp | 🔵 MuJoCo | 未実装 |
| 9 | confidence閾値 | inference.cpp | 🟠 実機 | 対象外 |
| 10 | bbox最小面積フィルタ | inference.h / inference.cpp | 🟠 実機 | 対象外 |
| 11 | watch/detect 2-mode遷移 | inference.h / inference.cpp | 🔵 MuJoCo | 未実装 |
| 12 | WATCHモードのfps間引き | inference.cpp | 🔵 MuJoCo | 未実装 |
| 13 | ACKタイムアウト | inference.h / inference.cpp | 🟠 実機 | 対象外 |
| 14 | サーボ到達待ち(ブロッキング) | px3_servo_node.h / px3_servo_node.cpp | 🔵 MuJoCo | 実装済み |
| 15 | サーボ角度の整数量子化 | px3_servo_node.cpp | 🟠 実機 | 対象外 |
| 16 | サーボパルス幅クランプ | px3_servo_node.cpp | 🟠 実機 | 対象外 |
| 17 | PIRデバウンス(watch_node側) | watch_node.h / watch_node.cpp | 🟠 実機 | 対象外 |
| 18 | PIRエッジ検知(px3側) | px3_servo_node.cpp | 🟠 実機 | 対象外 |
| 19 | PIR再トリガー機構 | px3_servo_node.cpp / watch_node.cpp | 🟠 実機 | 対象外 |
| 20 | wakeup二重送信防止 | watch_node.cpp | 🟠 実機 | 対象外 |
| 21 | シリアル読み取りタイムアウト | px3_servo_node.cpp | 🟠 実機 | 対象外 |
| 22 | Arduino起動待機 | px3_servo_node.cpp | 🟠 実機 | 対象外 |
| 23 | PIR再tracking未解決の既知制約 | fixed_report/05 | 🟠 実機 | 対象外 |

---

## MuJoCo対象挙動の実装状況サマリ(2026-07-15、④完了時点で更新)

`scripts/v3/` までに実装済みの挙動はそのまま維持し、変更しない。以降の実装は `scripts/v4/` 以降に配置する。

| # | 挙動 | 配置先 | 備考 |
|---|---|---|---|
| 1〜4, 6, 7, 14 | PD制御・dt正規化・レート制限・角度飽和・軸独立・デッドゾーン・サーボ遅延 | `scripts/v3/`(既存) | 変更不要。test1〜4でPASS確認済み |
| 11 | watch/detect 2-mode遷移 | `scripts/v4/`(TrackingModeFSM, ①) | 完了。test5a/b/cでPASS確認済み |
| 5 | 初回DETECTフレームのD項スキップ | `scripts/v4/`(②) | 完了。track_target_deg()内部でdt自動上書き。test5dでPASS確認済み |
| 8 | ロストフレームによるWATCH復帰 | `scripts/v4/`(③) | 完了。on_detection_result()。test5e/f/gでPASS確認済み |
| 12 | WATCHモードのfps間引き | `scripts/v4/`(④) | 完了(設計判断: fps値自体はperception層のため実装せず、\"WATCH中は新規指令を送らない\"という結果のみtrack_target_deg()の早期リターンとして実装)。test5h/iでPASS確認済み |

MuJoCo対象の全挙動(1〜8,11,12,14)の実装がv4で完了。挙動9,10,13,15〜23は検証区分の方針により実機側の対象。

---

## 1. PD制御(積分項なし)

🔵 **検証区分: MuJoCo**

**ファイル**: `axis_pd_controller.hpp`

```cpp
struct AxisPdController {
  double Kp = 0.0;
  double Kd = 0.0;
  double max_step_deg = 5.0;
  ...
  double pos_deg = 0.0;
  double prev_err_deg = 0.0;

  double updateFromErrorRad(double err_rad, double dt = 1.0) {
    const double err_deg = err_rad * 180.0 / PX_PI;
    const double d_term = (dt > 0.0) ? Kd * (err_deg - prev_err_deg) / dt : 0.0;
    double delta_deg = Kp * err_deg + d_term;
    ...
  }
};
```

I項(積分項)を保持するメンバ変数が存在しない。`Kp * err_deg + d_term`のみで指令を計算するP+D制御。

---

## 2. dt正規化

🔵 **検証区分: MuJoCo**

**ファイル**: `axis_pd_controller.hpp`

```cpp
  double updateFromErrorRad(double err_rad, double dt = 1.0) {
    const double err_deg = err_rad * 180.0 / PX_PI;

    // dt == 0.0 はDETECT遷移直後の初回フレームを示す。
    // reset(0.0) で prev_err_deg=0 の状態でKdを適用すると
    // 実態と乖離した微分スパイクが発生するため、初回はP項のみ使用する。
    const double d_term = (dt > 0.0) ? Kd * (err_deg - prev_err_deg) / dt : 0.0;
```

D項を`dt`で割ることでフレーム間隔の揺らぎを正規化している。`dt=0`の場合はD項自体をスキップする特別処理あり。

---

## 3. レート制限(振動抑制)

🔵 **検証区分: MuJoCo**

**ファイル**: `axis_pd_controller.hpp`

```cpp
    double delta_deg = Kp * err_deg + d_term;
    delta_deg = std::clamp(delta_deg, -max_step_deg, max_step_deg);

    pos_deg += delta_deg;
```

1ステップあたりの角度変化量を`max_step_deg`（実運用値: 3.0度、`inference.cpp`の`max_step_deg`パラメータ由来）でクランプする。過大な指令値による振動(ペンデュラム現象)を抑制する仕組み。

---

## 4. 角度飽和(クランプ)

🔵 **検証区分: MuJoCo**

**ファイル**: `axis_pd_controller.hpp`

```cpp
    pos_deg += delta_deg;
    prev_err_deg = err_deg;
    const double servo = center_deg + pos_deg;
    return std::clamp(servo, min_deg, max_deg);
```

最終出力角度を`min_deg`/`max_deg`でクランプする。run.sh実運用値ではyaw=[30,150]、pitch=[50,130]。

---

## 5. 初回DETECTフレームのD項スキップ

🔵 **検証区分: MuJoCo**

**ファイル**: `inference.cpp`

```cpp
    if (box.area() >= kMinBoxArea) {
      no_det_frames_ = 0;
      // 初回フレームは dt が不定のため微分項をスキップする
      const double effective_dt = first_detect_frame_ ? 0.0 : dt;
      first_detect_frame_ = false;

      computeAxisDeg(Axis::YAW, box, effective_dt);
      computeAxisDeg(Axis::PITCH, box, effective_dt);
```

`enterDetectTrackMode()`遷移直後は`first_detect_frame_ = true`となり、dt=0が`updateFromErrorRad`に渡される(上記2と連動)。

```cpp
void OnnxInferenceNode::enterDetectTrackMode() {
  mode_ = Mode::DETECT_TRACK;
  no_det_frames_ = 0;
  first_detect_frame_ = true;
  yaw_axis_.reset(0.0);
  pitch_axis_.reset(0.0);
  publishWakeup(true);
```

---

## 6. 軸独立ターゲット設計(yaw/pitch分離)

🔵 **検証区分: MuJoCo**

**ファイル**: `inference.cpp`

```cpp
void OnnxInferenceNode::computeAxisDeg(Axis axis, const cv::Rect2d& box, double dt) {
  const cv::Point2f p_meas = vision::bbox::bboxCenter(box);

  if (axis == Axis::YAW) {
    // 目標は画像中央X、Yは現在位置を維持（横方向のみ補正）
    const cv::Point2f p_tgt(static_cast<float>(active_width_ / 2.0), p_meas.y);
    const double yaw_rad = control::yawErrFromPixels(p_meas, p_tgt, calib_.K, calib_.D);
    servo_x_deg_ = yaw_axis_.updateFromErrorRad(yaw_rad, dt);
  } else {
    // 目標は画像高さ*kTargetYRatio、Xは現在位置を維持（縦方向のみ補正）
    const cv::Point2f p_tgt(p_meas.x, static_cast<float>(active_height_ * kTargetYRatio));
    const double pitch_rad = control::pitchErrFromPixels(p_meas, p_tgt, calib_.K, calib_.D);
    servo_y_deg_ = pitch_axis_.updateFromErrorRad(pitch_rad, dt);
  }
}
```

yawはbboxのx座標のみ、pitchはbboxのy座標のみを使って誤差計算する。互いに干渉しない完全独立制御。

---

## 7. デッドゾーン

🔵 **検証区分: MuJoCo**

**ファイル**: `inference.h`

```cpp
  static constexpr double kYawDeadZoneDeg = 1.5;
  static constexpr double kPitchDeadZoneDeg = 1.5;
```

**ファイル**: `inference.cpp`

```cpp
      // 両軸ともdead zone内ならサーボコマンドを発行しない
      const bool yaw_in_dead = std::abs(yaw_axis_.prev_err_deg) < kYawDeadZoneDeg;
      const bool pitch_in_dead = std::abs(pitch_axis_.prev_err_deg) < kPitchDeadZoneDeg;
      if (!yaw_in_dead || !pitch_in_dead) {
        publishState(static_cast<float>(servo_x_deg_), static_cast<float>(servo_y_deg_));
      }
```

両軸の誤差が1.5度未満なら、サーボ指令自体を送信しない(publishStateを呼ばない)。

---

## 8. ロストフレームによるWATCH復帰

🔵 **検証区分: MuJoCo**

**ファイル**: `inference.h`

```cpp
  int lost_max_frames_ = 15;
```

**ファイル**: `inference.cpp` (パラメータ宣言)

```cpp
  lost_max_frames_ = declare_parameter<int>("lost_max_frames", 15);
```

run.shでは`lost_max_frames:=30`で実運用値をオーバーライド。

```cpp
    } else {
      no_det_frames_++;
      if (no_det_frames_ >= lost_max_frames_)
        enterWatchMode();
    }
```

検出なしフレームが連続で閾値に達するとWATCHモードへ遷移。

---

## 9. confidence閾値

🟠 **検証区分: 実機**

**ファイル**: `inference.cpp`

```cpp
  const double conf_threshold = declare_parameter<double>("conf_threshold", 0.25);
  ...
  yolo_ = std::make_unique<YoloDetect>(pkg_path + std::string(ONNX_YOLO_PATH),
                                       static_cast<float>(conf_threshold));
```

YOLO推論エンジン(`YoloDetect`)のコンストラクタに閾値を渡し、低信頼度の検出を除外する。`fixed_report/04_conf_threshold_report.md`によれば、false positiveを追い続けてDETECTモードに滞留する問題(tracking不発)への対策として後日追加された。

---

## 10. bbox最小面積フィルタ

🟠 **検証区分: 実機**

**ファイル**: `inference.h`

```cpp
  static constexpr double kMinBoxArea = 1.0;
```

**ファイル**: `inference.cpp`

```cpp
    cv::Rect2d box = vision::bbox::findBiggestBBox(results);

    if (box.area() >= kMinBoxArea) {
      no_det_frames_ = 0;
      ...
```

面積が閾値未満のbboxは検出なしとみなす。

---

## 11. watch/detect 2-mode遷移

🔵 **検証区分: MuJoCo**

**ファイル**: `inference.h`

```cpp
  enum class Mode { WATCH, DETECT_TRACK };
  ...
  Mode mode_{Mode::WATCH};
```

**ファイル**: `inference.cpp`

```cpp
void OnnxInferenceNode::enterWatchMode() {
  mode_ = Mode::WATCH;
  no_det_frames_ = 0;
  wakeup_requested_.store(false);
  publishWakeup(false);
}

void OnnxInferenceNode::enterDetectTrackMode() {
  mode_ = Mode::DETECT_TRACK;
  no_det_frames_ = 0;
  first_detect_frame_ = true;
  yaw_axis_.reset(0.0);
  pitch_axis_.reset(0.0);
  publishWakeup(true);
}
```

2状態のみ(watch/detect_track)。search/lockのような中間状態は存在しない。

---

## 12. WATCHモードのfps間引き

🔵 **検証区分: MuJoCo**

**ファイル**: `inference.cpp`

```cpp
    if (mode_ == Mode::WATCH) {
      const auto t0 = std::chrono::steady_clock::now();
      const bool ext_wakeup = wakeup_requested_.exchange(false);
      if (ext_wakeup && px3_ready_.load()) {
        enterDetectTrackMode();
        continue;
      }

      // 低 fps を維持してフレームを読み捨て（カメラバッファを空にするため必要）
      const double fps = (watch_fps_ > 0.0 ? watch_fps_ : 2.0);
      const auto period = std::chrono::duration<double>(1.0 / fps);
      const auto dt = std::chrono::steady_clock::now() - t0;
      if (dt < period)
        std::this_thread::sleep_for(period - dt);

      ++frame_id;
      continue;
    }
```

run.sh実運用値`watch_fps=2.0`。WATCH中はフレームを読み捨てるのみでYOLO推論は走らない。

---

## 13. ACKタイムアウト

🟠 **検証区分: 実機**

**ファイル**: `inference.h`

```cpp
  // SERVO_ARRIVAL_WAIT_MS (30ms, px3_servo_node.h) + ROS通信マージン (20ms)
  static constexpr int kAckTimeoutMs = 50;
```

**ファイル**: `inference.cpp`

```cpp
bool OnnxInferenceNode::wait_for_ack_ms(int timeout_ms) {
  std::unique_lock<std::mutex> lk(ack_mtx_);
  return ack_cv_.wait_for(lk, std::chrono::milliseconds(timeout_ms), [this] { return ack_ready_; });
}

void OnnxInferenceNode::publishState(float yaw_deg, float pitch_deg) {
  ...
  pub_abs_->publish(m);
  if (!wait_for_ack_ms(kAckTimeoutMs))
    RCLCPP_WARN(get_logger(), "px3 ACK timeout; continuing");
}
```

px3への指令送信後、50ms以内にACKが来なければ警告を出すが処理は継続する(ブロックしない)。

---

## 14. サーボ到達待ち(ブロッキング)

🔵 **検証区分: MuJoCo**

**ファイル**: `px3_servo_node.h`

```cpp
// MG996R が目標角度に物理的に到達するまでの待機時間（ms）
inline constexpr int SERVO_ARRIVAL_WAIT_MS = 30;
```

**ファイル**: `px3_servo_node.cpp`

```cpp
void Px3ServoNode::on_angle(const custom_msgs::msg::AbsResult::SharedPtr msg) {
  if (board_) {
    board_->servo_write(static_cast<uint8_t>(SERVO_X_PIN), static_cast<int>(msg->x_angle));
    board_->servo_write(static_cast<uint8_t>(SERVO_Y_PIN), static_cast<int>(msg->y_angle));
    std::this_thread::sleep_for(std::chrono::milliseconds(SERVO_ARRIVAL_WAIT_MS));
  }
  std_msgs::msg::Bool ack;
  ack.data = true;
  ack_pub_->publish(ack);
}
```

サーボ書き込み後、30ms間スレッドをブロックしてから ACKを送信する。これは13のACKタイムアウト(50ms)と対をなす(30ms + ROS通信マージン20ms = 50ms)。

---

## 15. サーボ角度の整数量子化

🟠 **検証区分: 実機**

**ファイル**: `px3_servo_node.cpp`

```cpp
void Px3ServoNode::on_angle(const custom_msgs::msg::AbsResult::SharedPtr msg) {
  if (board_) {
    board_->servo_write(static_cast<uint8_t>(SERVO_X_PIN), static_cast<int>(msg->x_angle));
    board_->servo_write(static_cast<uint8_t>(SERVO_Y_PIN), static_cast<int>(msg->y_angle));
```

`msg->x_angle`(float)が`static_cast<int>`で切り捨てられる。PD制御は小数点角度を出力するが、実際にサーボへ送られるのは整数度のみ。**MuJoCo未対応の新規発見挙動**。

---

## 16. サーボパルス幅クランプ

🟠 **検証区分: 実機**

**ファイル**: `px3_servo_node.cpp`

```cpp
void TelemetrixSerial::servo_write(uint8_t pin, int angle_deg) {
  angle_deg = std::clamp(angle_deg, 0, 180);
  std::vector<uint8_t> cmd = {CMD_SERVO_WRITE, pin, static_cast<uint8_t>(angle_deg)};
  send_command(cmd);
}
```

Telemetrix層でも独立して0〜180度にクランプする(`axis_pd_controller`のmin/maxクランプとは別の、ハードウェア保護用の二重クランプ)。

サーボ初期化時のパルス幅設定:

```cpp
  const uint16_t x_pulse_min = 544;
  const uint16_t x_pulse_max = 2400;
  const uint16_t y_pulse_min = 544;
  const uint16_t y_pulse_max = 2400;
```

---

## 17. PIRデバウンス(watch_node側)

🟠 **検証区分: 実機**

**ファイル**: `watch_node.h`

```cpp
  int pir_debounce_ms_{200}; // 連続 PIR 検知に対するデバウンス (ms)
```

**ファイル**: `watch_node.cpp`

```cpp
  // ガード3: 前回 wakeup から pir_debounce_ms_ 以内は無視
  const auto now = std::chrono::steady_clock::now();
  const int64_t elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_wakeup_t_).count();
  if (elapsed_ms < static_cast<int64_t>(pir_debounce_ms_)) {
    RCLCPP_DEBUG(...);
    return;
  }
```

前回wakeup送信から200ms以内のPIR検知は無視する。**MuJoCo未対応の新規発見挙動**。

---

## 18. PIRエッジ検知(px3側)

🟠 **検証区分: 実機**

**ファイル**: `px3_servo_node.cpp`

```cpp
void Px3ServoNode::on_pir_change(uint8_t /*pin*/, uint8_t value) {
  const bool new_state = (value != 0);
  const bool old_state = pir_detected_.exchange(new_state);
  ...
  if (new_state == old_state)
    return; // 変化なし → 無視（デバウンス兼用）

  std_msgs::msg::Bool msg;
  msg.data = new_state;
  pir_state_pub_->publish(msg);
}
```

値の変化(エッジ)のみをpublishする。同じ値が連続しても再送しない。**17と合わせて二重のデバウンス構造になっている**。

---

## 19. PIR再トリガー機構

🟠 **検証区分: 実機**

**ファイル**: `px3_servo_node.cpp`

```cpp
void TelemetrixSerial::retrigger_pir_reporting() {
  send_set_pin_mode(pir_pin_);
}
```

```cpp
void Px3ServoNode::on_watch_resumed(const std_msgs::msg::Bool::SharedPtr msg) {
  if (!msg->data && init_ok_) {
    pir_detected_.store(false);
    if (board_) {
      board_->retrigger_pir_reporting();
    }
    RCLCPP_INFO(get_logger(), "px3: WATCH resumed → pir_detected_ reset + PIR re-triggered");
  }
}
```

WATCH復帰時にArduinoへSET_PIN_MODEを再送し、PIRがHIGH保持中でもエッジを再検出できるようにする。`fixed_report/04_pir_reset_fix_report.md`由来の修正。

---

## 20. wakeup二重送信防止

🟠 **検証区分: 実機**

**ファイル**: `watch_node.h`

```cpp
  std::atomic<bool> wakeup_sent_{false}; // 二重送信防止
```

**ファイル**: `watch_node.cpp`

```cpp
  // ガード2: wakeup 送信済みなら重複送信しない（px2 から false が来るまでブロック）
  if (wakeup_sent_.load()) {
    RCLCPP_DEBUG(get_logger(), "watch_node: PIR detected but wakeup already sent");
    return;
  }
  ...
  last_wakeup_t_ = now;
  wakeup_sent_.store(true);
  publish_wakeup(true);
```

px2からの`/px_wakeup=false`フィードバックが来るまで、次のwakeup送信をブロックする。

---

## 21. シリアル読み取りタイムアウト

🟠 **検証区分: 実機**

**ファイル**: `px3_servo_node.cpp`

```cpp
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10; // 1s read タイムアウト（read_loop の polling 間隔）
```

シリアルポートのread()が1秒でタイムアウトし、read_loopのポーリング間隔を規定する。**MuJoCo未対応の新規発見挙動**。

---

## 22. Arduino起動待機

🟠 **検証区分: 実機**

**ファイル**: `px3_servo_node.cpp`

```cpp
    board_ = std::make_unique<TelemetrixSerial>(port, baud);
    // Arduino リセット完了まで待機（Telemetrix初期化に必要）
    std::this_thread::sleep_for(std::chrono::seconds(4));
```

シリアルポートを開いた後、Arduinoのリセット完了まで4秒間ブロック待機する。起動シーケンスのみに関わる挙動で、稼働中の制御ループには影響しない。

---

## 23. PIR再tracking未解決の既知制約

🟠 **検証区分: 実機**

**ファイル**: `fixed_report/05_PIRtracking_issue_report.md`

> DETECT_TRACKモード中にtargetがカメラのフレームから一度退出し、短時間（観測上おおよそ2〜7秒程度）で再侵入した場合に、trackingが再開されない現象が発生した。
>
> 結論：ハードウェア（PIRモジュール）特性に起因する既知の制約として受容し、Pattern-Bの調査はここで終了。Night/Pattern-Cへ移行する。

コード修正では解決されず、既知の制約として受容された。シミュレーションで再現しても、実機側の対策(ハードウェア変更やPattern-C移行)がなければ解消しない。

---

## 未実装・非対応(参考)

| 挙動 | 状態 |
|---|---|
| search mode(見失い後のスキャン探索) | 未実装。見失うとWATCHに戻るのみ |
| 昼夜カメラ切替(RGB/IR) | Pattern-Bでは非搭載。Pattern-Cへ移行の記述あり |
| バックラッシュ・関節摩擦の明示的モデル | 実機コード上に対応する記述なし(物理現象として存在するが、ソフトウェアでは補償していない) |
