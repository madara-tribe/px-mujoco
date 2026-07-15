"""
optics.py — カメラ光学変換ロジック(独立モジュール)

実機コード axis_pd_controller.hpp の yawErrFromPixels / pitchErrFromPixels を
Pythonに移植したもの。cv2.undistortPoints は cv::undistortPoints(C++)と
同一アルゴリズムのため、calib_result.yaml(= data/params/camera_calib.yaml)の
値をそのまま使えば理論上ほぼ完全に一致する
(reports/05_optics_reproduction_feasibility.md で実測検証済み: 画面端でも差0.2deg未満)。

このモジュールは px_env.py から呼び出される想定で、px_env.py 自体には
光学計算のロジックを書かない(役割を分離する)。

提供する変換:
  pixel_error_to_angle_rad() : 実機と同じ方向(pixel誤差 -> 角度)
  angle_deg_to_pixel_offset() : 逆変換(角度 -> pixel位置)。
    シミュレーション側は「目標のサーボ角度」で軌道を組むテストが多いため、
    一度pixel空間に変換してからノイズ等を注入し、再度角度に戻す
    往復変換(angle -> pixel -> [noise] -> angle)に使う。
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass
class CameraCalibration:
    fx: float
    fy: float
    cx: float
    cy: float
    dist_coeffs: np.ndarray  # [k1, k2, p1, p2, k3]
    image_width: int
    image_height: int

    @property
    def K(self) -> np.ndarray:
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ])

    @classmethod
    def from_yaml(cls, path: str) -> "CameraCalibration":
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        cm = d["camera_matrix"]
        dc = d["dist_coeffs"]
        return cls(
            fx=cm["fx"], fy=cm["fy"], cx=cm["cx"], cy=cm["cy"],
            dist_coeffs=np.array([dc["k1"], dc["k2"], dc["p1"], dc["p2"], dc["k3"]]),
            image_width=d["image_width"], image_height=d["image_height"],
        )


def pixel_error_to_angle_rad(calib: CameraCalibration,
                              p_meas: tuple[float, float],
                              p_tgt: tuple[float, float],
                              axis: str) -> float:
    """
    axis_pd_controller.hpp の yawErrFromPixels / pitchErrFromPixels の直訳。

    p_meas: 検出されたbbox中心のpixel座標 (x, y)
    p_tgt : 目標pixel座標 (x, y)。yawなら(画像中央X, p_meas.y)、
            pitchなら(p_meas.x, 画像高さ*kTargetYRatio)を呼び出し側で用意する。
    axis  : "yaw" または "pitch"。dx(横方向)かdy(縦方向)かを選ぶ。

    戻り値: 角度誤差(ラジアン)。正 = yawならtargetが右、pitchならtargetが下。
    """
    src = np.array([[list(p_meas)], [list(p_tgt)]], dtype=np.float32)
    und = cv2.undistortPoints(src, calib.K, calib.dist_coeffs)
    # und[i, 0] は正規化座標 (歪み補正 + 焦点距離1に正規化済み)
    if axis == "yaw":
        d = und[1, 0, 0] - und[0, 0, 0]  # dx
    elif axis == "pitch":
        d = und[1, 0, 1] - und[0, 0, 1]  # dy
    else:
        raise ValueError(f"axis must be 'yaw' or 'pitch', got {axis}")
    return float(np.arctan2(d, 1.0))


def angle_deg_offset_to_pixel_offset(calib: CameraCalibration,
                                      angle_deg_offset: float, axis: str) -> float:
    """
    角度誤差(度, center基準の相対値) -> pixel誤差 の近似逆変換。

    厳密な逆変換は undistortPoints の完全な逆写像(歪み込み)が必要だが、
    calib_result.yaml の歪み係数による影響は最大0.2deg程度であることが
    実測済みのため、逆変換にはピンホール近似(歪み無視)を用いる。
    これは「テスト軌道を角度空間で組んだ後、pixel空間でノイズを注入し、
    正変換(pixel_error_to_angle_rad、歪み補正込み)で角度に戻す」という
    往復変換の起点として使うものであり、往路の近似誤差は復路の厳密変換で
    実質的に吸収される。

    axis: "yaw" (fx使用) または "pitch" (fy使用)
    """
    f = calib.fx if axis == "yaw" else calib.fy
    angle_rad = np.radians(angle_deg_offset)
    return float(f * np.tan(angle_rad))


def simulate_detection(calib: CameraCalibration,
                        true_yaw_offset_deg: float, true_pitch_offset_deg: float,
                        pixel_noise_sigma_px: float = 0.0,
                        rng: np.random.Generator | None = None) -> tuple[float, float]:
    """
    「真の角度オフセット(center基準)」から、実機のbbox検出〜光学変換
    パイプラインを模して「観測される角度オフセット」を求める往復変換。

    経路: 角度(deg) -> pixel(近似) -> [ノイズ注入] -> 角度(deg, 厳密な undistort 経由)

    ノイズはpixel空間(実機のYOLO bbox検出のばらつきに相当する単位)で注入する。
    これにより、画面端に近いほど同じpixelノイズが大きな角度ノイズに変換される
    という非線形性(実機と同じ性質)が再現される。

    pixel_noise_sigma_px=0.0 なら、往復変換の近似誤差(画面端で最大0.2deg程度)
    のみが乗る。ノイズなしの追従テスト(test1, test4)でこの関数を使う場合、
    この往復変換自体がわずかな系統誤差の発生源になりうる点に注意。
    """
    img_cx, img_cy = calib.image_width / 2.0, calib.image_height / 2.0

    px_offset_x = angle_deg_offset_to_pixel_offset(calib, true_yaw_offset_deg, "yaw")
    px_offset_y = angle_deg_offset_to_pixel_offset(calib, true_pitch_offset_deg, "pitch")

    if pixel_noise_sigma_px > 0.0 and rng is not None:
        px_offset_x += rng.normal(0.0, pixel_noise_sigma_px)
        px_offset_y += rng.normal(0.0, pixel_noise_sigma_px)

    p_meas = (img_cx + px_offset_x, img_cy + px_offset_y)
    p_center = (img_cx, img_cy)

    yaw_rad = pixel_error_to_angle_rad(calib, p_meas, p_center, "yaw")
    pitch_rad = pixel_error_to_angle_rad(calib, p_meas, p_center, "pitch")

    # 符号に注意: pixel_error_to_angle_rad(p_meas, p_tgt) は p_tgt方向への誤差。
    # ここではp_meas(観測位置)からp_center(画像中央=目標)への誤差を求めたいので、
    # meas/tgtを入れ替えて符号を反転する。
    return -np.degrees(yaw_rad), -np.degrees(pitch_rad)
