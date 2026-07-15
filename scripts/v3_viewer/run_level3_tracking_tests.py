"""Run the original Level 3 test logic with a live MuJoCo 3D viewer.

The source under scripts/v3/ remains unchanged.  This file imports its test
classes, attaches viewer synchronization to a copied PxPanTiltEnv, and stores
viewer-run plots under outputs/viewer/.
"""

from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from types import ModuleType

VIEWER_DIR = Path(__file__).resolve().parent
ROOT = VIEWER_DIR.parent.parent
sys.path.insert(0, str(VIEWER_DIR))
sys.path.insert(0, str(ROOT / "scripts" / "common"))

from px_env import PxPanTiltEnv
from runtime_viewer import (
    EnvViewerBridge,
    RuntimeViewer,
    add_viewer_arguments,
    viewer_config_from_args,
)

MODEL_PATH = ROOT / "models" / "pattern_b_integrated.xml"
PARAMS_PATH = ROOT / "data" / "params" / "control_params.yaml"
ORIGINAL_TEST_PATH = ROOT / "scripts" / "v3" / "run_level3_tracking_tests.py"
VIEWER_OUTPUT_DIR = ROOT / "outputs" / "viewer"


def load_original_tests() -> ModuleType:
    spec = importlib.util.spec_from_file_location("px_v3_original_tracking_tests", ORIGINAL_TEST_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load original test module: {ORIGINAL_TEST_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    module.OUTPUT_DIR = VIEWER_OUTPUT_DIR
    return module


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Level 3 PD tests with live 3D visualization")
    parser.add_argument(
        "--test",
        choices=("all", "test1", "test2", "test3", "test4"),
        default="all",
        help="Run all tests or a selected test.",
    )
    parser.add_argument(
        "--test1-speed",
        type=float,
        default=None,
        help="When running test1 only, visualize one speed instead of the full sweep.",
    )
    add_viewer_arguments(parser)
    return parser.parse_args()


def compute_baseline_30dps(original: ModuleType) -> float:
    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    baseline_test = original.Test1ConstantVelocitySweep()
    baseline_test.SPEEDS_DEG_S = [30]
    result, data = baseline_test.run(env)
    if not result.passed:
        raise RuntimeError("Could not establish a valid 30deg/s baseline for the selected test")
    return float(data[30]["latter_half_mean"])


def report_results(results: list) -> None:
    print("\n" + "=" * 70)
    print(f"SUMMARY: {sum(r.passed for r in results)}/{len(results)} tests passed")
    for result in results:
        print(f"  [{'PASS' if result.passed else 'FAIL'}] {result.name}")
    print("=" * 70)


def main() -> list:
    args = parse_args()
    original = load_original_tests()
    env = PxPanTiltEnv(str(MODEL_PATH), str(PARAMS_PATH))
    config = viewer_config_from_args(args)

    print("=" * 70)
    print("Level 3 Viewer: original PD tracking tests")
    print("Original scripts/v3 files are unchanged")
    print("=" * 70)

    results = []
    all_data = {}

    with RuntimeViewer(env.model, env.data, config) as viewer:
        bridge = EnvViewerBridge(env, viewer).attach()

        if args.test in ("all", "test1"):
            test1 = original.Test1ConstantVelocitySweep()
            if args.test == "test1" and args.test1_speed is not None:
                test1.SPEEDS_DEG_S = [args.test1_speed]
            bridge.set_state(phase="Level 3 / test1", behavior="constant velocity baseline")
            result1, data1 = test1.run(env)
            print("\n" + result1.report())
            results.append(result1)
            all_data["test1"] = data1
        else:
            data1 = None

        if args.test == "all":
            if 30 not in data1:
                raise RuntimeError("The all-tests baseline does not contain 30deg/s")
            baseline_30dps = float(data1[30]["latter_half_mean"])
        elif args.test in ("test2", "test3"):
            baseline_30dps = compute_baseline_30dps(original)
        else:
            baseline_30dps = 0.0

        if args.test in ("all", "test2"):
            bridge.set_state(phase="Level 3 / test2", behavior="pixel noise + dropout")
            test2 = original.Test2NoisyDropoutTracking()
            result2, data2 = test2.run(env, baseline_latter_half_mean=baseline_30dps)
            print("\n" + result2.report())
            results.append(result2)
            all_data["test2"] = data2

        if args.test in ("all", "test3"):
            bridge.set_state(phase="Level 3 / test3", behavior="30 ms servo delay")
            test3 = original.Test3ServoDelayTracking()
            result3, data3 = test3.run(env, baseline_latter_half_mean=baseline_30dps)
            print("\n" + result3.report())
            results.append(result3)
            all_data["test3"] = data3

        if args.test in ("all", "test4"):
            bridge.set_state(phase="Level 3 / test4", behavior="boundary clamp and step limit")
            test4 = original.Test4ClampBoundaryTracking()
            result4, data4 = test4.run(env)
            print("\n" + result4.report())
            results.append(result4)
            all_data["test4"] = data4

        bridge.detach()
        viewer.wait_until_closed({"phase": "Level 3 complete"})

    report_results(results)

    if args.test == "all":
        out_path = original.plot_results(
            all_data["test1"],
            all_data["test2"],
            all_data["test3"],
            all_data["test4"],
        )
        print(f"\nplot saved: {out_path}")

    return results


if __name__ == "__main__":
    main()
