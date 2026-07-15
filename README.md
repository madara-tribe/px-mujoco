# PX Sim v4 — TrackingModeFSM統合版 (Pattern-B 分割XML統合モデル + Level 1〜4 + 光学変換)

Pattern-Bの実機コード(`SensorFireld-PX4.3-day-pattern-B`)を解析し、
そのパラメータ・挙動に対応させたMuJoCoシミュレーション。

**v4での変更点**: 01_behavior_source_reference.mdの検証区分方針(MuJoCoは
「正常系のactuator/joint/制御ロジック/state transition確認」に限定)に
基づき、MuJoCo対象と確定した挙動のうち残っていた挙動5・8・11・12
(watch/detectモード遷移とその周辺)を`TrackingModeFSM`として実装した。
v3までのスクリプトは`scripts/v3/`に凍結し、変更していない。

v3での変更点(参考): カメラの光学変換ロジック(pixel誤差→角度)を
`optics.py`として独立実装し、実機の校正データ(`calib_result.yaml`)を
そのまま使用する。

## フォルダ構成

```
px_sim_v4/
├── models/
│   ├── parts/
│   │   ├── servo_x.xml       # PANサーボ単体(MG996R, 固定側)
│   │   ├── servo_y.xml       # TILTサーボ単体(MG996R)
│   │   ├── bracket.xml       # コの字ジンバルブラケット
│   │   └── camera.xml        # RGBカメラ(実測70x30x30mm/50g)
│   └── pattern_b_integrated.xml   # <include>で上記を統合(v3から変更なし)
├── scripts/
│   ├── v3/                            # v3までのスクリプト(凍結、変更なし)
│   │   ├── optics.py
│   │   ├── px_env.py
│   │   ├── run_level1_static.py
│   │   ├── run_level2_sweep.py
│   │   └── run_level3_tracking_tests.py  # Level 3: PD追従検証 test1-4
│   └── v4/                            # v4新規スクリプト(本レポート対象)
│       ├── optics.py                     # v3からコピー、変更なし
│       ├── px_env.py                     # TrackingModeFSM統合版
│       ├── tracking_mode_fsm.py          # 新規: 挙動5,8,11,12
│       └── run_level4_fsm_tests.py       # Level 4: FSM検証 test5(a〜i)
├── data/params/
│   ├── control_params.yaml   # run.sh実運用値(center/min/max, lost_max_frames等)
│   └── camera_calib.yaml     # カメラ校正値(calib_result.yaml由来)
├── reports/
│   ├── 01_behavior_source_reference.md    # 挙動一覧+実機コード該当箇所+検証区分(v4更新)
│   ├── 02_xml_dimension_conversion.md     # 実測寸法→XML変換の記録(v3)
│   ├── 03_isaac_sim_setup_memo.md         # Isaac Sim環境構築メモ(v3, 将来用)
│   ├── 04_future_problem_prediction.md    # 案1-3の挙動追加で発生/しない問題の予測(v3)
│   ├── 05_optics_reproduction_feasibility.md  # 光学変換の再現度検証(v3)
│   ├── 06_v4_TrackingModeFSM_QA_report.md # v4実装後のQ&A集(v4新規)
│   └── 07_v4_implementation_summary.md    # v4実装(①〜④)のサマリ(v4新規)
├── outputs/                  # 実行結果(png)
└── requirements.txt
```

## セットアップ・実行

```bash
pip install -r requirements.txt
cd px_sim_v4

# v3(既存挙動、回帰確認用)
python3 scripts/v3/run_level1_static.py
python3 scripts/v3/run_level2_sweep.py
python3 scripts/v3/run_level3_tracking_tests.py

# v4(TrackingModeFSM、新規)
python3 scripts/v4/run_level4_fsm_tests.py
```


## モデル設計の要点

### 分割XML + include方式
`servo_x.xml` / `servo_y.xml` / `bracket.xml` / `camera.xml` は
純粋な剛体形状(`<body>` + `<geom>`)のみを持ち、jointを持たない。
可動軸(`yaw`, `pitch`)は統合ファイル `pattern_b_integrated.xml` 側で
一元管理する。これは実機コードで `axis_pd_controller.hpp` が
`yaw_axis_` / `pitch_axis_` を独立構造体として持つ設計と対応させている。

### 単位の統一(重要な既知の落とし穴)
`compiler angle="degree"` は **joint の `range`/`ref` には効くが、
actuator の `ctrlrange` には効かない場合がある**。この不一致により、
初回実装では目標90degに対し実際171deg・195degに収束する不具合が
発生した(actuatorの力計算 `kp*(ctrl-qpos)` で単位が揃っていなかったため)。

対策として本モデルは **`compiler angle="radian"` に統一し、
joint range/ref・actuator ctrlrangeすべてをradianの生値で明記**している。
度単位の値はXMLコメントに併記した。今後XMLを編集する際は、
角度に関する数値は必ずradianで統一すること。

## run.sh 準拠パラメータ

| 軸 | center | min | max |
|---|---|---|---|
| yaw (PAN)   | 90deg | 30deg | 150deg |
| pitch (TILT) | 90deg | 50deg | 130deg |

`data/params/control_params.yaml` の `future_params` には、
Level3以降で使用予定の追加パラメータ(dead zone, lost_max_frames,
conf_threshold, ack_timeout_ms, PD gains等)も実機コードの値のまま
集約してある。

## Level 1: サーボの静止

center_deg(yaw=90, pitch=90)に初期化し、外力(重力)下でその姿勢を
維持できるか確認する。

結果: yawは誤差ゼロ(PAN軸は垂直回転軸のため静的重力トルクを受けない)。
pitchは約0.15degの定常偏差が生じる(position actuatorがP制御のみで
積分項を持たないため、片持ちカメラの重力トルクを完全には打ち消せない)。
これはPattern-Bの `axis_pd_controller.hpp` が積分項(I)を持たない設計と
整合する既知の挙動であり、`data/params/pattern_b_parts.yaml`
(旧px_simパッケージ)の `known_issues` にも記録済み。

## Level 2: center -> max -> center -> min -> center スイープ

run.sh準拠の可動範囲内で、yaw/pitchそれぞれをこの順に往復させる。
PD制御は未使用で、position actuatorへの直接角度指令のみ。
往復時に発生するオーバーシュート(0.2〜0.3deg程度)は正常な物理応答で、
joint range逸脱(out_of_range_samples)はゼロであることを確認済み。

## Level 3: PD制御によるtarget追従検証 (test1〜4)

`scripts/run_level3_tracking_tests.py` で実行。1回のMuJoCo実行で
test1〜4を順に行い、PASS/FAIL判定つきレポートを出す。

```bash
python3 scripts/run_level3_tracking_tests.py
```

### 実装したPD制御 (px_env.py の AxisPdController)

`axis_pd_controller.hpp` を直訳したクラス。積分項(I)は実機同様
意図的に持たない。dt正規化(dt=0でD項スキップ)、レート制限
(max_step_deg)、角度飽和(min/max clamp)、デッドゾーン判定を実装。

### test1〜4の内容

| test | 検証内容 | 判定方式 |
|---|---|---|
| test1 | 速度スイープ付き定速追従(10〜200deg/s) | baseline計測(健全性チェックのみ、NaN/発散がないか) |
| test2 | 検出ノイズ(σ=2deg) + ドロップアウト(15%)下の追従 | test1 baseline比2倍以内 |
| test3 | サーボ書き込み遅延30ms(SERVO_ARRIVAL_WAIT_MS)下の追従 | test1 baseline比2倍以内 |
| test4 | targetがmin/max境界付近を高速振動する状況 | PDコマンド出力のstep deltaが実機仕様値(3.0deg/frame)以内 |

### 実装過程で見つかった重要な設計ミスと修正(教訓)

**発見1: PD更新頻度と物理タイムステップの取り違え**
初期実装ではPD制御を物理タイムステップ(2ms/500Hz)ごとに呼んでいたが、
実機の`axis_pd_controller`はinference.cppのDETECT_TRACKループ内、
つまりYOLO推論フレーム(run.sh準拠 active_fps=30Hz)ごとにしか呼ばれない。
この不一致により、test1のbaselineとtest2(30Hzでノイズを注入)の実行条件が
異なり、Kd項の分母dtが極小になることでノイズが桁違いに増幅される
異常な発振が発生した。全testのPD更新を30Hz(detection_fps)に統一して解消。

**発見2: 実機PDゲイン自体の追従性能限界**
修正後、実用速度域(10-60deg/s)でも定常追従誤差が10-20deg程度生じる
ことが判明した。これはバグではなく、実機ゲイン(Kp=0.3, Kd=0.2)が
30Hz駆動という条件下での追従性能そのものの限界を示す実測値である。
そのためtest1は絶対閾値のPASS/FAILから「baseline計測」に位置づけを
変更し、test2-4はtest1との相対比較(悪化率)で判定する設計にした。

**発見3: 検証対象の選定ミス(物理応答 vs 制御コマンド)**
test4は当初「物理qposの1ステップ変化」を監視していたが、position
actuatorの物理応答(kp・慣性)がボトルネックとなり、PD層のmax_step_deg
を意図的に壊しても物理側で自然に抑制され、レート制限バグを検出
できないことが判明した(env.max_step_deg=50に緩めても物理step deltaは
最大4.1deg/stepに留まった)。「PDコマンド出力(yaw_cmd)自体の差分」を
監視する設計に変更し、異常検出力を確認した(正常時PASS、
max_step_deg破壊時FAIL、をそれぞれ確認済み)。

これらはすべて `reports/04_future_problem_prediction.md` で予測した
「教訓1型(実装ミス)」「教訓2型(構造起因、新規設計が必要)」の
両方が実際に発生した実例であり、特に発見2は評価基準そのものを
実機に頼れず新規設計する必要があった(3-Aで予測した通り)。

### v3で追加した光学変換 (optics.py)

`track_target_deg()`に`use_optics`引数を追加した(既定False)。
`use_optics=True`にすると、「目標角度 - 現在角度」の真の誤差を
`optics.py`の`simulate_detection()`経由でpixel空間に変換し、実機と
同じ`cv2.undistortPoints`で角度に戻した「観測誤差」をPD制御に渡す。

test1・test3・test4は`use_optics=True`を使用。test2はノイズ注入方式を
角度空間直接注入から`pixel_noise_sigma_px`経由のpixel空間注入に変更した
(ドロップアウトの「前回値保持」ロジックは光学変換と無関係なため維持)。

詳細な再現度の検証(実機の校正データでの歪み補正の影響が画面端でも
最大0.17deg程度と判明したこと、往復変換の系統誤差、test4への影響が
ゼロであることの確認等)は `reports/05_optics_reproduction_feasibility.md`
を参照。

## Pattern-Bコード解析で確認した挙動一覧(実装は今後段階的に追加)

コード(`inference.cpp`, `inference.h`, `axis_pd_controller.hpp`,
`px3_servo_node.h`, `watch_node.h`, `fixed_report/*.md`)を解析し、
以下の挙動を確認した。Level1/2では未実装だが、今後
`px_env.py` の `PxPanTiltEnv` にメソッドとして追加していく。

| 挙動 | 実機コードでの実体 |
|---|---|
| PD制御(I項なし) | `AxisPdController` (Kp, Kd)。積分項は存在しない |
| dt正規化 | `updateFromErrorRad(err_rad, dt)`。dt=0時はD項スキップ |
| レート制限(振動抑制) | `max_step_deg` によるclamp |
| 角度飽和 | `std::clamp(servo, min_deg, max_deg)` |
| デッドゾーン | yaw/pitchとも1.5deg以内なら指令を出さない |
| ロストフレームでWATCH復帰 | `no_det_frames_ >= lost_max_frames_`(実運用30) |
| confidence閾値 | YOLO false positive除去用(後日追加された修正) |
| ACKタイムアウト | px3への指令到達確認、50ms |
| watch/detect 2-mode遷移 | `Mode::WATCH` / `Mode::DETECT_TRACK`の2状態のみ |
| 初回DETECTフレームのD項スキップ | `first_detect_frame_`、dt不定による微分スパイク回避 |
| 軸独立ターゲット設計 | yawは横方向のみ、pitchは縦方向のみ補正 |
| PIR再tracking未解決の既知制約 | 2〜7秒の短時間再侵入で追従が再開しない(Pattern-Bでは解決断念、Pattern-Cへ移行) |
| search mode | **未実装**(見失うとWATCHに戻るのみ、スキャン探索なし) |
| 昼夜カメラ切替 | **Pattern-Bでは非搭載**(Pattern-Cへ移行の記述あり) |

## Level 4: TrackingModeFSM検証 (test5, v4新規)

`scripts/v4/run_level4_fsm_tests.py` で実行。

```bash
python3 scripts/v4/run_level4_fsm_tests.py
```

### 実装した挙動 (tracking_mode_fsm.py の TrackingModeFSM)

01_behavior_source_reference.mdの検証区分方針に基づき、MuJoCo対象と
確定した挙動のうち残っていた4つを実装した。

| 挙動 | 内容 | 実機対応 |
|---|---|---|
| 11 | watch/detect 2-mode遷移 | `enterWatchMode()` / `enterDetectTrackMode()` |
| 5 | 初回DETECTフレームのD項スキップ | `first_detect_frame_` |
| 8 | ロストフレームによるWATCH復帰 | `no_det_frames_ >= lost_max_frames_` |
| 12 | WATCHモードのfps間引き(の帰結) | WATCH中はactuatorへ新規指令を送らない |

**設計方針**: `TrackingModeFSM`はPxPanTiltEnv・AxisPdControllerから独立させ、
状態管理のみを行う。PDのreset()呼び出しやactuator操作はEnv側がFSMの
状態/戻り値を見て行う(責務分離)。挙動12について、実機のfps間引き
(`watch_fps=2.0`)はYOLO推論頻度というperception層の処理であり、
MuJoCo側の対象外と判断した。MuJoCoが担うのはその帰結である
「WATCH中は新規指令を送らない」という事実のみで、これは
`track_target_deg()`の早期リターンとして実装した。

### test5(a〜i)の内容

| test | 検証内容 | 対応挙動 |
|---|---|---|
| test5-a | FSM単体の状態遷移(WATCH<->DETECT_TRACK, 一回性消費) | 11 |
| test5-b | Env統合時のリセット順序(FSM遷移とPDリセットが揃うこと) | 11 |
| test5-c | watch復帰時、FSM状態のみ変化しPD状態は保持されること | 11 |
| test5-d | track_target_deg()内部でのdt自動上書き | 5 |
| test5-e | lost_max_frames回連続失敗での自動WATCH遷移 | 8 |
| test5-f | 途中の検出成功によるno_det_framesリセット | 8 |
| test5-g | WATCH中のon_detection_result()無害化 | 8 |
| test5-h | WATCH中はtrack_target_deg()が状態を変更しないこと | 12 |
| test5-i | detect(動く)->watch(静止)->detect復帰(再び動く)の統合シナリオ | 5+8+11+12統合 |

結果: test5(9/9)・test1〜4(4/4、v3と数値完全一致)ともに全てPASS。
実装過程で発見したテスト側の設計ミス(actuator収束待ちの不足、
非現実的な許容誤差設定)を含め、詳細は
`reports/07_v4_implementation_summary.md` を参照。

### MuJoCo対象挙動 実装完了状況

01_behavior_source_reference.mdの検証区分方針でMuJoCo対象と確定した
11個の挙動(1,2,3,4,5,6,7,8,11,12,14)は、v3+v4で全て実装完了した。
挙動9,10,13,15〜23の12個は方針により実機(ROS2/Arduino)側の対象として、
MuJoCoでの実装対象からは意図的に除外している。

## 実行環境について

Google Colab CPUランタイムでの実行を想定。GPU(MJX)は不要と判断した。
理由: PXは2軸剛体のみで、今後追加する挙動(PD制御・mode遷移・遅延・
ノイズ等)はすべて逐次的な状態遷移ロジックであり、GPU並列化の恩恵を
ほぼ受けない。RL方策学習等、大規模並列が必要になる段階で
MJX移行を検討すればよい。モデル資産(XML)はそのまま流用可能。

