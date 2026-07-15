# 光学変換ロジック(optics.py)実装レポート — px_sim_v3

**作成日**: 2026-07-12
**対象**: `scripts/optics.py`, `data/params/camera_calib.yaml`, `px_env.py`/`run_level3_tracking_tests.py`への統合

---

## 実装方針

実機コード`axis_pd_controller.hpp`の`yawErrFromPixels`/`pitchErrFromPixels`を
Pythonに移植した独立モジュール`optics.py`を新設し、`px_env.py`から呼び出す形にした。
光学計算のロジックは`px_env.py`本体には書かず、役割を分離している。

`cv2.undistortPoints`(Python版OpenCV)は`cv::undistortPoints`(C++版)と同一実装のため、
`calib_result.yaml`の値をそのまま使えば理論上ほぼ完全に一致する。

## 実機カメラの校正品質(検証結果)

`calib_result.yaml`(20枚のチェッカーボード画像から校正、rms再投影誤差0.27px)の
歪み係数を使い、歪み補正込みの厳密な角度計算とピンホール近似(歪み無視)の差を実測した。

| pixel_offset_x | 歪み補正あり[deg] | ピンホール近似[deg] | 差[deg] |
|---|---|---|---|
| 0 | 0.000 | 0.000 | 0.000 |
| 100 | 4.404 | 4.403 | 0.000 |
| 300 | 12.970 | 13.008 | -0.037 |
| 500 | 20.895 | 21.058 | -0.162 |
| 600 | 24.629 | 24.798 | -0.169 |

画面端(600px、水平画角52.5度のほぼ限界)でも最大0.17度程度しか乖離しない。
これはこのカメラの歪み係数が絶対値として小さい(広角魚眼ではなく標準画角に近い)ため。

## optics.pyの構成

| 関数 | 役割 |
|---|---|
| `CameraCalibration.from_yaml()` | camera_calib.yamlを読み込みK行列・歪み係数を保持 |
| `pixel_error_to_angle_rad()` | 実機の`yawErrFromPixels`/`pitchErrFromPixels`の直訳。`cv2.undistortPoints`使用 |
| `angle_deg_offset_to_pixel_offset()` | 逆変換(角度→pixel)。ピンホール近似(歪み無視)を使用 |
| `simulate_detection()` | 角度→pixel→[ノイズ]→角度、の往復変換。test2のノイズ注入で使用 |

### 往復変換(simulate_detection)の系統誤差

逆変換にピンホール近似を使っているため、往復させると若干の系統誤差が乗る。

| 真の角度オフセット[deg] | 往復後の観測角度[deg] | 差[deg] |
|---|---|---|
| 0 | 0.000 | 0.000 |
| 10 | 9.977 | -0.023 |
| 20 | 19.832 | -0.168 |
| 24 | 23.837 | -0.163 |

最大でも0.17度程度で、PD制御のdead_zone_deg(1.5度)より一桁小さい。実用上無視できる。

## px_env.py / run_level3_tracking_tests.pyへの統合

`track_target_deg()`に`use_optics`引数を追加(既定False、後方互換を維持)。
`use_optics=True`で「真の角度誤差 → optics.simulate_detection() → 観測誤差」を経由してから
PD制御に渡す。

| test | 変更内容 |
|---|---|
| test1 | `use_optics=True`を追加。baseline計測に光学変換の系統誤差(最大0.17度)が乗るが、baseline自体の値(30dps時17.4度前後)に対して無視できる規模 |
| test2 | ノイズ注入方式を角度空間直接注入から、`pixel_noise_sigma_px`経由のpixel空間注入に変更。ドロップアウト時の「前回値保持」ロジックは維持(光学変換と無関係な事象のため) |
| test3 | `use_optics=True`を追加。遅延との相互作用を確認済み(PASS) |
| test4 | `use_optics=True`を追加。事前検証で、`delta_deg`の`max_step_deg`クランプが光学変換の往復誤差(0.17度程度)を完全に吸収し、PDコマンド出力への影響がゼロであることを確認(クランプ限界の検証対象を汚染しない) |

## 検証結果

- v2からv3への移行で、`use_optics=False`時の挙動(既存test1〜4のbaseline等)が変化しないことを確認済み(数値一致)。
- `use_optics=True`でも静止targetへの収束結果はほぼ変化なし(差0.0004度程度)。
- test2で意図的にpixel_noise_sigma_pxを800まで上げても既定の判定閾値(baseline x2.0)ではPASSし続けることを確認。これはPD制御のレート制限(max_step_deg)がノイズに対する天然のローパスフィルタとして機能しているため。
- 閾値をbaseline x1.1まで厳しくすると正しくFAILすることを確認済み(異常検出力が機能している)。

## 今後の課題

- `angle_deg_offset_to_pixel_offset`の逆変換はピンホール近似であり、厳密な歪み込み逆変換(`cv2.projectPoints`相当)ではない。往復誤差は実用上無視できる範囲(0.17度以下)だが、より高精度が必要になった場合はここを厳密化する余地がある。
- pitch方向の目標点設定(`kTargetYRatio`、実機は`active_height * kTargetYRatio`)は本実装では単純化して中央固定にしている。この値の実機由来の具体的な数値は未確認。
