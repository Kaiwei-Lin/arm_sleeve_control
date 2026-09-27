# Model control golden fixture

`model_control_golden.json` was captured before replacing the 741-line
`tools/run_model_control.py`, from the user working tree at the recorded HEAD.
Its normalized source SHA-256 and provenance are stored in the JSON.

The capture executed the original readiness prediction block verbatim:
shoulder prediction → elbow prediction → dataclass replacement → fresh rotation
pair → rotation update → radians replacement. It used the existing predictor,
elbow EMA and rotation estimator implementations, identity calibration pairs,
scripted estimator results and the input rows stored in the fixture. Clock and
inference timer were fixed; no model was trained or robot connected. Both
existing mappers were then applied to each captured intent.

The regression test runs those same inputs through the new IntentPipeline.
Do not regenerate expected values from the new pipeline when changing it:
that would remove the independent pre-refactor baseline. Floating-point values
are compared with tolerance; all discrete metadata and missing values match.

## Local estimator migration baseline

`dual_imu_migration_golden.json` was captured from the installed
`flexarm-estimator==0.3.0` before replacing its imports. The fixture records that
implementation's SHA-256, the control pipeline's configured axes/thresholds,
rest/forward calibration inputs, calibration outputs, and nine chest/arm pairs.
It covers Rest, Forward, Backward, Lateral, Transition target holding, sensor
mounting, world movement and reset to rest. Outputs include all legacy estimate
fields and complete MotionIntent values with the inference timer fixed. The
local-estimator test compares those fields with 1e-12 tolerance; new optional
continuous/YXY fields are covered by the source project's migrated tests.
Do not regenerate this baseline from the local implementation.
