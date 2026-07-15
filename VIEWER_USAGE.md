# PX MuJoCo model: 3D / 360-degree viewing

Run commands from the `px_sim_v4` directory.

## 1. Install

```bash
python -m pip install -r requirements.txt
```

## 2. Fastest manual 3D view

Linux / Windows:

```bash
python -m mujoco.viewer --mjcf=models/pattern_b_integrated.xml
```

macOS:

```bash
mjpython -m mujoco.viewer --mjcf=models/pattern_b_integrated.xml
```

Use the MuJoCo viewer mouse controls to orbit, pan and zoom around the model.

## 3. Automatic 360-degree orbit

Linux / Windows:

```bash
python viewer/view_model_360.py --auto
```

macOS:

```bash
mjpython viewer/view_model_360.py --auto
```

Controls:

- `Space`: pause/resume automatic camera rotation
- `R`: reset the camera azimuth
- Close the window to exit

Useful adjustments:

```bash
mjpython viewer/view_model_360.py --auto \
  --speed 20 \
  --elevation -25 \
  --distance 0.30 \
  --yaw 90 \
  --pitch 90
```

The script rotates the **viewer camera** around the model. It does not rotate the physical base, so the simulated model and gravity remain unchanged.
