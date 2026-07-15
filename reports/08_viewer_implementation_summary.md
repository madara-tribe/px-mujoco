# v3/v4 Viewer分離実装サマリ

## 方針

既存の `scripts/v3/` と `scripts/v4/` は変更せず、Viewer用コードを別ディレクトリへ分離した。
物理モデルXML、制御パラメータ、PD制御、FSMの元実装も変更していない。

## 追加ファイル

```text
scripts/common/runtime_viewer.py
scripts/v3_viewer/optics.py
scripts/v3_viewer/px_env.py
scripts/v3_viewer/run_level1_static.py
scripts/v3_viewer/run_level2_sweep.py
scripts/v3_viewer/run_level3_tracking_tests.py
scripts/v4_viewer/optics.py
scripts/v4_viewer/px_env.py
scripts/v4_viewer/tracking_mode_fsm.py
scripts/v4_viewer/run_level4_fsm_tests.py
scripts/v4_viewer/run_level4_fsm_visual.py
VIEWER_USAGE.md
```

`optics.py`、`px_env.py`、`tracking_mode_fsm.py`、`run_level4_fsm_tests.py` は対応する元ファイルから複製した。
Viewer専用ランナーだけが `runtime_viewer.py` を使用する。

## runtime_viewer.pyの責務

- MuJoCo passive viewer起動
- 30～60 FPSに間引いた `viewer.sync()`
- 実時間再生と再生速度変更
- マウス操作と自動360度カメラ周回
- target位置の表示専用マーカー
- joint角度、actuator command、FSM状態のオーバーレイ
- Viewerを使わない `--headless` 実行

Viewer処理は `PxPanTiltEnv.step()` の元実装へ追加していない。
`EnvViewerBridge` がViewer版の実行時だけ既存envへ表示同期を接続する。

## v3 Viewer

- Level 1: center位置での静止保持を3D表示
- Level 2: center → max → center → min → centerを3D表示
- Level 3: 元のtest1～4クラスを読み込み、元の判定条件のまま3D表示
- Viewer版の2D出力先は `outputs/viewer/`

## v4 Viewer

- `run_level4_fsm_tests.py`: 元テストの複製。自動判定用途
- `run_level4_fsm_visual.py`: WATCH → DETECT → detection lost → WATCH → DETECT復帰を目視しやすい時間幅で再生
- WATCH中のcommand blocking、lost frame数、first detect frameを画面へ表示

## 確認結果

以下をViewerなしのheadlessモードで実行した。

- v3 Level 1: 実行完了
- v3 Level 2: 実行完了、range逸脱なし
- v3 Level 3: 4/4 PASS
- v4 FSMテスト: 9/9 PASS
- v4可視化シナリオ: lost-frame自動WATCH復帰 PASS、WATCH command blocking PASS
- 新規Pythonファイル: `py_compile` PASS
- `runtime_viewer.py`: fake viewer handleを使用したtarget marker・overlay・sync確認 PASS
- 元の `scripts/v3/` と `scripts/v4/`: original archiveとの`diff -qr`で差分なし

実行環境がディスプレイを持たないため、GUIウィンドウ自体の起動確認は行っていない。
MuJoCo 3.10.0のviewer APIに対する構文・headless動作・表示処理の単体確認を実施した。
