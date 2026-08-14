# FlexArm Estimator Integration Design

## Goal

Replace the legacy `flex_model_0003.FlexPredictor.predict_raw(...)` integration
with the new stateful `flexarm.FlexArmEstimator` API while preserving the
project's existing sensor acquisition, elbow prediction, semantic joint
mapping, safety controller, and robot backends.

The shoulder estimator consumes Sleeve packet fields CH3, CH4, and CH5. The
existing rule-based elbow predictor continues to consume CH2.

## Current Architecture

The current Phase 4 runtime is:

```text
SleeveSource
  -> SensorSynchronizer
  -> RuleBasedPredictor(CH2) ---------------------------> elbow angle
  -> FlexModelPredictor(CH2/CH3/CH4, legacy model API) -> shoulder action/angle
  -> ArmMotionPredictor
  -> ArmMapper
  -> SafeArmController
  -> FakeRobotArm or DyMotorArm
```

`FlexModelPredictor` dynamically imports a configured class and calls
`predict_raw(flex=[...], calibration_baseline=[...], calibration_scale=[...],
trial_rest=[...])`. Calibration is performed by a separate tool using the old
model package's `collect_flex_samples` and `quick_calibrate_flex` functions.

The new estimator instead loads a directory of model artifacts once, maintains
a causal feature window and temporal state, and is called once per frame:

```python
estimator = FlexArmEstimator.from_pretrained(model_dir)
result = estimator.update(
    flex1=flex1,
    flex2=flex2,
    flex3=flex3,
    timestamp_ns=timestamp_ns,
)
```

It also performs a new per-wear calibration from an `n x 3` matrix of resting
raw samples.

## Selected Approach

Keep the current control project's boundaries and replace only the model
adapter and its configuration. Install `flexarm-estimator` as a Python
dependency, keep trained artifacts outside the repository, and point the Phase
4 configuration at the artifact directory.

This avoids copying binary model files into the control repository and avoids
fragile `sys.path` references to the adjacent `arm_data_collector` checkout.
The external model package owns feature extraction, motion gating,
classification, regression, action debounce, confidence thresholds, angular
velocity limiting, and EMA filtering. The control repository owns sensor
selection, calibration orchestration, conversion to joint semantics, fault
handling, and robot safety.

## Target Architecture

```text
SleeveSource
  -> SensorSynchronizer
  -> RuleBasedPredictor(CH2) ----------------------------> elbow absolute angle
  -> FlexModelPredictor adapter
       -> FlexArmEstimator.update(CH3, CH4, CH5, time_ns) -> shoulder state/angle
  -> ArmMotionPredictor
  -> ArmMapper
  -> SafeArmController
  -> FakeRobotArm or DyMotorArm
```

The following components remain unchanged in responsibility:

- `SleeveSource` parses complete sensor packets and exposes `SleeveFrame`.
- `SensorSynchronizer` produces `SensorSample` values on the Sleeve monotonic
  clock.
- `RuleBasedPredictor` maps CH2 to the absolute semantic elbow angle.
- `ArmMapper` maps semantic joint targets without motor/CAN knowledge.
- `SafeArmController` remains the only path to robot motion.
- `FakeRobotArm` and `DyMotorArm` keep their current lifecycle and safety
  behavior.

## Configuration

Replace the old Phase 4 model-module configuration with an estimator artifact
configuration:

```yaml
predictor:
  backend: flexarm_estimator
  model_dir: ../arm_data_collector/flexarm_estimator/models
  sleeve_channels: [3, 4, 5]
  calibration_file: calibrations/flexarm_live_calibration.json
  calibration_seconds: 3.0
  angle:
    min_deg: 0
    max_deg: 180

phase4_validation:
  max_consecutive_prediction_errors: 3
```

Paths are resolved relative to the Phase 4 configuration file, so an alternate
configuration can be deployed without depending on the process working
directory. `model_dir` must exist and contain the estimator metadata,
calibration, classifier, and three angle-regressor artifacts. The runtime
calibration file remains separate from the training artifact directory.

The configured Sleeve channels must be exactly `[3, 4, 5]`, matching the new
model's trained packet-field order. CH2 remains configured independently in
`configs/phase3.yaml` for the elbow predictor.

The obsolete `model_module`, `model_class`, `min_action_confidence`,
`action_stability`, and old baseline/scale/trial-rest parsing are removed. The
new estimator already owns confidence gating and temporal stabilization.

## Estimator Adapter

`FlexModelPredictor` remains the project-facing class name to minimize changes
to control composition, but it becomes an adapter around one long-lived
`FlexArmEstimator` instance.

Construction loads the estimator with `FlexArmEstimator.from_pretrained`.
Tests may inject an estimator double so unit tests do not require model binary
files. A clear `RuntimeError` wraps missing package or invalid model-artifact
errors before any robot is enabled.

For each `SensorSample`, the adapter:

1. Converts configured one-based channels CH3/CH4/CH5 to tuple indexes 2/3/4.
2. Validates that all three inputs exist and are finite.
3. Converts `sample.timestamp` seconds to `int(sample.timestamp * 1e9)`.
4. Calls `estimator.update` with named `flex1`, `flex2`, `flex3`, and
   `timestamp_ns` arguments.
5. Validates and converts the estimator result to a `MotionIntent`.

`Forward` maps to positive shoulder flexion, `Backward` maps to negative
shoulder flexion, and `Lateral` maps to positive shoulder abduction. Angles are
absolute human-semantic angles in radians and remain subject to the existing
robot configuration and `SafeArmController` checks.

## Rest, Unknown, and Target Holding

The new estimator can legitimately emit `Unknown` while its causal window is
warming or when confidence is insufficient, and `Rest` when the wearer is
stationary. These are successful predictions, not inference errors.

`MotionIntent.action` continues to hold only one of the three active
`ArmAction` values and is `None` for `Rest` or `Unknown`. Add fields that retain
the model-facing state and diagnostics:

- `model_action: str | None`
- `angle_confidence: float | None`
- `moving: bool | None`

The adapter retains the most recent valid active shoulder targets. During
`Rest` or `Unknown`, it emits those retained shoulder targets while preserving
the current result's diagnostic state. Before the first active result, the
shoulder fields remain `None`, which makes the mapper hold the robot startup
positions. The independent CH2 elbow result continues to update in both cases.

This prevents a stationary or warming estimator from snapping a shoulder joint
back to its startup position while still allowing elbow control.

## Calibration Lifecycle

Live calibration is the default because the estimator requires a fresh
baseline after the garment is put on.

The control tool starts the Sleeve source, asks the wearer to let the arm hang
naturally, discards already-buffered observations by waiting for fresh frame
timestamps, and collects approximately `calibration_seconds` of CH3/CH4/CH5
rows. It rejects non-finite rows and requires at least two valid samples. It
then calls `estimator.calibrate(rows)`, atomically saves the returned
calibration to `calibration_file`, and calls `estimator.reset()` before normal
prediction.

`--reuse-calibration` explicitly skips live collection. The adapter loads and
validates the configured saved calibration and creates the estimator with that
calibration applied to the already-loaded model artifacts. Reuse is never the
implicit default.

Model and sensor readiness complete before the robot is connected. For a real
robot, connection remains read-only until all existing readiness and
calibration checks succeed. Servo On and position commands remain gated by
`--execute`.

## Runtime Error Handling

The following are normal estimator states and do not increment the prediction
error counter:

- window warm-up reported as `Unknown`;
- low-confidence/debounced state reported as `Unknown`;
- stationary state reported as `Rest`.

The following are rejected predictions:

- missing CH3, CH4, or CH5;
- non-finite sensor inputs;
- an unsupported model action label;
- a non-finite or out-of-range active angle;
- invalid confidence or moving values;
- an exception raised by model inference.

On a rejected prediction, the runtime holds the last safe complete target and
increments the existing consecutive prediction-error counter. Reaching
`max_consecutive_prediction_errors` faults the run and enters the existing
shutdown path. Sensor stale and hard-timeout behavior is unchanged. Any
calibration, package import, artifact load, or configuration error occurs
before Servo On.

## Command-Line Behavior

`tools/run_model_control.py` gains:

- `--reuse-calibration`: explicitly use the configured saved calibration;
- `--calibration-seconds`: override the configured live collection duration;
- `--calibration-output`: override the configured runtime calibration path.

Without `--reuse-calibration`, the tool prompts for and performs live
calibration. Existing `--sleeve`, `--robot`, `--execute`, timeout, diagnostics,
and duration behavior remains intact. Fake Sleeve runs use deterministic
calibration collection suitable for automated and manual dry runs.

`tools/test_flex_model.py` uses the same adapter and exposes the same model
directory and calibration policy without importing or connecting any robot.
The obsolete separate old-model calibration implementation is replaced by
shared calibration helpers used by both tools.

## Testing Strategy

All production changes follow test-driven development.

Configuration tests verify:

- model and calibration paths are resolved relative to the Phase 4 config;
- `[3, 4, 5]` is accepted and other channel orders are rejected;
- missing model directories, non-positive calibration duration, and malformed
  values fail with clear messages.

Adapter tests verify:

- a model instance is reused across frames;
- exact named arguments receive CH3/CH4/CH5 and nanosecond timestamps;
- Forward, Backward, and Lateral retain the current semantic mapping;
- Rest and Unknown preserve the last active shoulder targets;
- Rest and Unknown before the first active result leave shoulders unset;
- confidence, angle confidence, moving, and model action diagnostics are
  preserved;
- invalid inputs and active outputs are rejected.

Calibration tests verify:

- fresh CH3/CH4/CH5 rows are collected without reordering;
- timeout and invalid rows are skipped;
- fewer than two valid rows fail;
- live calibration is saved and resets estimator temporal state;
- explicit calibration reuse loads and applies the saved calibration.

An integration test combines the real rule-based elbow predictor, the new
shoulder adapter with a fake estimator, `ArmMotionPredictor`, `ArmMapper`,
`SafeArmController`, and `FakeRobotArm`. It verifies that all three joint
targets still pass through the same safety path.

The full `pytest` suite is the automated acceptance gate. Verification must not
open a serial port, load the DyMotor bridge, enable a real servo, or issue real
robot commands. Hardware acceptance remains a manual sequence documented in
the README: FakeRobot, real Sleeve with FakeRobot, DyMotor read-only dry run,
then explicit `--execute` under physical supervision.

## Dependencies and Documentation

Runtime requirements add the dependencies declared by `flexarm-estimator`:
NumPy, Joblib, and the exact scikit-learn version required by the artifacts.
The README documents installing the wheel, configuring the external model
directory, live calibration, explicit calibration reuse, model state logging,
and the staged hardware validation sequence.

No trained model binaries, runtime calibration files, recordings, IDE files,
or adjacent-project source trees are committed to this repository.

