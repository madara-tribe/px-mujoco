# MuJoCo Simulation for PX4.3

MuJoCo simulation for the Pattern-B system, based on the behavior and parameters of the real implementation in `SensorFireld-PX4.3-day-pattern-B`.

This package contains:

- An integrated MuJoCo model built from separate XML parts
- Level 1 to Level 4 validation scripts
- Camera pixel-to-angle conversion
- A `TrackingModeFSM` implementation for WATCH and DETECT/TRACK mode transitions


<img width="700" height="500" alt="Image" src="https://github.com/user-attachments/assets/ca76cc81-e689-464f-a7bb-1d2d80d880a8" />


## What Changed in v4

Version 4 follows the validation policy defined in `01_behavior_source_reference.md`.

MuJoCo is used only to validate:

- Normal actuator behavior
- Joint movement
- Control logic
- State transitions

Abnormal-value behavior, hardware communication, and other real-device-specific cases are intended to be tested on the physical system.

The remaining MuJoCo-targeted behaviors—items 5, 8, 11, and 12—were implemented in v4 as `TrackingModeFSM`. These behaviors cover WATCH/DETECT mode transitions and related control logic.

All scripts from v3 are preserved in `scripts/v3/` and were not modified.

## What Changed in v3

Version 3 introduced camera optical conversion as a separate module named `optics.py`.

The module converts pixel error into angular error and uses the calibration values from the real camera calibration file, `calib_result.yaml`.


## Setup

Install the required Python packages:

```bash
pip install -r requirements.txt
```

Move to the project directory:

```bash
cd px_sim_v4
```

## Run the v3 Validation Scripts

Use these scripts to validate the existing model and confirm that earlier behavior still works correctly.

```bash
python3 scripts/v3/run_level1_static.py
python3 scripts/v3/run_level2_sweep.py
python3 scripts/v3/run_level3_tracking_tests.py
```

## Run the v4 FSM Validation

Use the following command to validate the new `TrackingModeFSM` behavior:

```bash
python3 scripts/v4/run_level4_fsm_tests.py
```

## Validation Scope

The MuJoCo simulation is intended to validate:

- PAN and TILT joint movement
- Actuator response
- Joint limits
- PD tracking behavior
- Pixel-to-angle conversion
- WATCH and DETECT/TRACK state transitions
- Lost-detection handling
- Controller reset and command-blocking behavior

The following areas should be validated on the physical system instead:

- Serial communication
- Real servo response and timing
- Camera and detector performance
- PIR sensor behavior
- Hardware-specific failures
- Abnormal sensor values
- Electrical or mechanical faults
