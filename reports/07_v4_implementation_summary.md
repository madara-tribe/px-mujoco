# v4 実装サマリレポート — TrackingModeFSM (挙動5・8・11・12)

**作成日**: 2026-07-15
**対象**: `px_sim_v4`（`scripts/v4/`）
**位置づけ**: v3完了後、01_behavior_source_reference.mdの検証区分方針に基づきMuJoCo対象と確定した挙動5・8・11・12を、①〜④の4段階に分けて実装した際の設計判断・実装内容・テスト結果を集約した記録。06のQ&Aレポートは本レポートの内容を踏まえたQ&A集であり、本レポートは実装そのものの経緯を扱う。

---

## 前提：検証区分の方針（01レポートより）

MuJoCoは「正常系の制御・動作確認」に役割を限定する。

- actuatorが期待通りに動作するか
- jointが期待通りに動作するか
- 制御ロジックによって想定した動きになるか
- モード遷移(state transition)が正しく行われるか

この方針の下、23個の挙動のうちMuJoCo対象は**1,2,3,4,5,6,7,8,11,12,14の11個**と確定した。うち1,2,3,4,6,7,14はv3までに実装済み。残る**5,8,11,12**がv4のスコープ。

---

## 全体設計

```
TrackingModeFSM (新規class, scripts/v4/tracking_mode_fsm.py)
├── state: Mode.WATCH | Mode.DETECT_TRACK
├── no_det_frames: int
├── first_detect_frame: bool
├── lost_max_frames: int (既定30, run.sh実運用値)
├── reset()
├── enter_watch()            # 挙動11
├── enter_detect_track()     # 挙動11 + 挙動5のフラグセット
├── consume_first_detect_frame()  # 挙動5の一回性消費
├── on_detection_result()    # 挙動8
├── is_watch() / is_detect()
```

**責務分離の方針**: FSMは状態管理のみを行い、PDController(`AxisPdController`)のreset呼び出しやactuator(`data.ctrl`)の操作には一切触れない。これらはEnv側(`PxPanTiltEnv`)がFSMの状態/戻り値を見て実行する。04_future_problem_prediction.mdの2-A（mode遷移時のリセット順序の取り違え）を、FSM単体でテスト可能な形にすることで防ぐ狙い。

**実装順序の理由**: 依存関係の下流から実装した。①(11)が他3つ全ての土台となるFSM骨格、②(5)は①のenter_detect_trackと連動、③(8)は①のFSM骨格の上に自動遷移を追加、④(12)は状態(①)が確定してから「watch中は指令を送らない」という帰結を実装、という順。

---

## ①: 挙動11（watch/detect 2-mode遷移）

**実装**:
- `TrackingModeFSM`クラスを新規実装。`Mode.WATCH`/`Mode.DETECT_TRACK`のみを持つ
- `enter_watch()`/`enter_detect_track()`は実機`inference.cpp`の`enterWatchMode()`/`enterDetectTrackMode()`を直訳
- `PxPanTiltEnv`側の`self.mode`/`self.no_det_frames`スタブを`self.fsm`（FSMインスタンス）に置換
- `Env.enter_detect_track()`/`Env.enter_watch()`を新設し、「FSM遷移→PDリセット」の順序を1メソッド内に固定

**テスト**: test5-a（FSM単体の遷移）、test5-b（Env統合時のリセット順序）、test5-c（watch復帰時のPD状態保持）

---

## ②: 挙動5（初回DETECTフレームのD項スキップ）

**実装**:
- `track_target_deg()`冒頭で`self.fsm.consume_first_detect_frame()`を呼び、`True`が返れば引数`dt`を強制的に`0.0`に上書き
- 一回性の消費のため、2回目以降の呼び出しは通常通り引数`dt`が使われる
- FSMを使わない既存呼び出し(test1〜4)は`first_detect_frame`が常に`False`のため無影響

**発見したバグ（テスト側）**: test5-d初回実行でFAIL。原因は実装ではなくテスト設計。2回目の`track_target_deg()`呼び出し前に`env.step()`を挟んでおらず、jointが物理的に動いていない状態で誤差の変化を期待していた。`step()`を挟んで実際にjointを動かしてから2回目を呼ぶよう修正し解消。

**テスト**: test5-d（dt自動上書き・一回性消費）

---

## ③: 挙動8（ロストフレームによるWATCH復帰）

**実装**:
- `TrackingModeFSM`のコンストラクタに`lost_max_frames`引数を追加（既定30。`control_params.yaml`の`future_params.lost_max_frames`から読む）
- `on_detection_result(detected: bool) -> bool`を新設。実機`inference.cpp`のメインループ分岐（検出成功で`no_det_frames`リセット、失敗でカウントアップし閾値到達で`enterWatchMode()`）を直訳
- WATCH状態で呼ばれた場合は無害化（実機がこの分岐を通らないことの再現）

**テスト**: test5-e（`lost_max_frames`回連続失敗での自動遷移）、test5-f（途中の検出成功によるカウントリセット）、test5-g（WATCH中の無害化）

---

## ④: 挙動12（WATCHモードのfps間引き）

**設計判断（実装前の再検討）**: 実機コードを読み直した結果、WATCH中のfps間引き(`watch_fps=2.0`)は「YOLO推論ループ自体をスキップし、フレームを読み捨てる」処理であり、perception層（YOLO推論の実行頻度）の話であると判明。01レポートの検証区分方針（confidence閾値・面積フィルタは実機区分）と同じ理由で、fps値そのもの(2Hz)の間引きロジックはFSMに実装しないと判断。MuJoCo側が担うべきは、その結果として観測される「WATCH中はactuatorへ新規指令が一切送られない」という事実のみとした。

**実装**:
- `should_run_control()`のような新規メソッドは追加せず、既存の`is_watch()`を`Env.track_target_deg()`側で参照する形にした
- `Env`に`_fsm_enabled`フラグを追加。FSM系メソッド(`enter_detect_track`/`enter_watch`)を一度も使わない呼び出し(test1〜4)で早期リターンが誤発火しないようにするガード
- `track_target_deg()`冒頭で`self.fsm.is_watch() and self._fsm_enabled`の場合、`(None, None, False, False)`を返して早期リターン。actuator・PD状態はいずれも変更しない

**発見したバグ（テスト側、2件）**: test5-iで2回FAILし、いずれもテスト設計の問題だった。

1. **フェーズ1の準備不足**: WATCH突入直前にactuator目標値(`ctrl`)がまだqposに完全収束していなかったため、WATCH突入後もqposが動き続けた。これはMuJoCoのposition actuatorが直前の`ctrl`へ物理的に収束し続ける正常動作であり、実装のバグではない。`env.settle(500steps)`を挟んで収束させてから遷移するテストに修正。
2. **許容誤差が非現実的**: `1e-6`度という厳密すぎる閾値を使っていたが、04レポートの教訓2（I項なしPD制御の定常偏差）により実際は`0.0001`度オーダーの残留誤差が生じる。`0.01`度に緩和。

**テスト**: test5-h（WATCH中の早期リターン確認）、test5-i（動く→止まる→また動く、統合シナリオ）

---

## 最終テスト結果

**test5（`scripts/v4/run_level4_fsm_tests.py`）— 9/9 PASS**

| test | 検証内容 | 対応挙動 |
|---|---|---|
| test5-a | FSM単体の状態遷移 | 11 |
| test5-b | Env統合時のリセット順序 | 11 |
| test5-c | watch復帰時のPD状態保持 | 11 |
| test5-d | dt自動上書き・一回性消費 | 5 |
| test5-e | lost_max_frames回連続失敗での自動遷移 | 8 |
| test5-f | 検出成功によるカウントリセット | 8 |
| test5-g | WATCH中の`on_detection_result()`無害化 | 8 |
| test5-h | WATCH中の早期リターン確認 | 12 |
| test5-i | 動く→止まる→また動く統合シナリオ | 5+8+11+12統合 |

**test1〜4（既存、回帰確認）— 4/4 PASS、v3と数値完全一致**
FSM統合が既存挙動（PD制御・光学変換・サーボ遅延等）に一切影響していないことを、①〜④の各ステップで都度確認済み。

---

## MuJoCo対象挙動 実装完了状況

MuJoCo対象と確定した11個の挙動（1,2,3,4,5,6,7,8,11,12,14）は、v3+v4で全て実装完了。挙動9,10,13,15〜23の12個は検証区分の方針により実機（ROS2/Arduino）側の対象として、MuJoCoでの実装対象からは意図的に除外している。
