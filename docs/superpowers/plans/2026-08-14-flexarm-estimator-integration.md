# FlexArm Estimator Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the legacy Flex shoulder model call with `FlexArmEstimator`, using CH3/CH4/CH5, per-wear calibration, and safe Rest/Unknown target holding while preserving CH2 elbow control and the existing robot safety path.

**Architecture:** Keep acquisition, synchronization, elbow prediction, mapping, safety, and robot backends intact. Migrate Phase 4 configuration to external model artifacts, place calibration orchestration in a focused helper module, and make `FlexModelPredictor` a stateful adapter around one `FlexArmEstimator` instance.

**Tech Stack:** Python 3.10+, PyYAML, NumPy, Joblib, scikit-learn 1.7.2, pytest, external `flexarm-estimator` wheel.

## Global Constraints

- Shoulder model inputs are exactly Sleeve CH3/CH4/CH5; elbow input remains CH2.
- `Rest` and `Unknown` are valid states, not prediction errors.
- Rest/Unknown hold the last active shoulder target; before one exists, shoulders hold startup positions.
- Live calibration is the default; reuse requires `--reuse-calibration`.
- Real motion remains gated by `--execute` and all current safety checks.
- Do not commit model binaries, runtime calibration, `.idea`, recordings, or adjacent-project sources.
- Follow red-green-refactor for every production behavior.

---

### Task 1: Migrate Phase 4 Configuration

**Files:**
- Modify: `sleeve_arm/config.py`
- Modify: `configs/phase4.yaml`
- Test: `tests/test_phase4.py`

**Interfaces:**
- Produces: `FlexModelConfig(model_dir: Path, sleeve_channels: tuple[int, int, int], calibration_file: Path, calibration_seconds: float, angle_min_deg: float, angle_max_deg: float)`.
- Produces: backend name `flexarm_estimator`.
- Resolves model and calibration paths relative to the selected Phase 4 YAML file.

- [ ] **Step 1: Write failing configuration tests**

```python
def test_phase4_loads_flexarm_paths_relative_to_config(tmp_path) -> None:
    model_dir = tmp_path / "models"
    model_dir.mkdir()
    config = write_phase4_config(tmp_path)
    loaded = load_phase4_config(config)
    assert loaded.flex_model is not None
    assert loaded.flex_model.model_dir == model_dir.resolve()
    assert loaded.flex_model.calibration_file == (tmp_path / "runtime/calibration.json").resolve()
    assert loaded.flex_model.sleeve_channels == (3, 4, 5)
    assert loaded.flex_model.calibration_seconds == 3.0


def test_phase4_rejects_wrong_flexarm_channels(tmp_path) -> None:
    config = write_phase4_config(tmp_path, sleeve_channels="[2, 3, 4]")
    with pytest.raises(ValueError, match=r"exactly \[3, 4, 5\]"):
        load_phase4_config(config)


def test_phase4_rejects_non_positive_calibration_duration(tmp_path) -> None:
    config = write_phase4_config(tmp_path, calibration_seconds="0")
    with pytest.raises(ValueError, match="calibration_seconds must be positive"):
        load_phase4_config(config)
```

`write_phase4_config` creates a real `models/` directory and writes a complete
literal YAML with backend `flexarm_estimator`, `model_dir: models`, channels
`[3, 4, 5]`, `calibration_file: runtime/calibration.json`, an angle range, and
the existing error threshold.

- [ ] **Step 2: Verify RED**

Run: `python -m pytest tests/test_phase4.py -k "phase4_loads_flexarm_paths or wrong_flexarm_channels or non_positive_calibration" -v`

Expected: FAIL because the loader still requires legacy module and calibration fields.

- [ ] **Step 3: Implement minimal configuration migration**

Replace legacy `FlexModelConfig` fields with the interface above. Remove JSON
triple loading. Require an existing model directory, exact channels `(3, 4,
5)`, positive finite calibration seconds, and an ordered finite angle range.
Update `configs/phase4.yaml` to use
`../../arm_data_collector/flexarm_estimator/models` and
`../calibrations/flexarm_live_calibration.json`, both relative to `configs/`.

- [ ] **Step 4: Verify GREEN**

Run: `python -m pytest tests/test_phase4.py -k "phase4_loads_flexarm_paths or wrong_flexarm_channels or non_positive_calibration" -v`

Expected: selected tests PASS.

- [ ] **Step 5: Commit**

```powershell
git add sleeve_arm/config.py configs/phase4.yaml tests/test_phase4.py
git commit -m "refactor: migrate flexarm model configuration"
```

---

### Task 2: Represent Estimator Diagnostics

**Files:**
- Modify: `sleeve_arm/domain/motion.py`
- Test: `tests/test_phase4.py`

**Interfaces:**
- Adds optional `MotionIntent.model_action: str | None`.
- Adds optional `MotionIntent.angle_confidence: float | None`.
- Adds optional `MotionIntent.moving: bool | None`.
- Existing constructor call sites stay compatible through defaults.

- [ ] **Step 1: Write failing validation tests**

```python
def test_motion_intent_preserves_flexarm_diagnostics() -> None:
    intent = MotionIntent(timestamp=1.0, model_action="Rest", angle_confidence=0.75, moving=False)
    assert (intent.model_action, intent.angle_confidence, intent.moving) == ("Rest", 0.75, False)


@pytest.mark.parametrize("value", (-0.1, 1.1, math.nan))
def test_motion_intent_rejects_invalid_angle_confidence(value: float) -> None:
    with pytest.raises(ValueError, match="angle_confidence"):
        MotionIntent(timestamp=1.0, angle_confidence=value)


def test_motion_intent_rejects_non_boolean_moving() -> None:
    with pytest.raises(ValueError, match="moving"):
        MotionIntent(timestamp=1.0, moving=1)
```

- [ ] **Step 2: Verify RED**

Run: `python -m pytest tests/test_phase4.py -k motion_intent -v`

Expected: FAIL because the fields do not exist.

- [ ] **Step 3: Add fields and validation**

Accept only model actions `Forward`, `Backward`, `Lateral`, `Rest`, `Unknown`;
validate angle confidence in `[0, 1]` and require `moving` to be a real boolean.

- [ ] **Step 4: Verify GREEN and commit**

Run: `python -m pytest tests/test_phase4.py -k motion_intent -v`

```powershell
git add sleeve_arm/domain/motion.py tests/test_phase4.py
git commit -m "feat: represent flexarm prediction diagnostics"
```

---

### Task 3: Add Shared Live Calibration Helpers

**Files:**
- Create: `sleeve_arm/predictor/calibration.py`
- Modify: `tools/calibrate_flex_model.py`
- Test: `tests/test_phase4.py`

**Interfaces:**
- Produces: `collect_calibration_samples(source, channels, duration_s, *, monotonic=time.monotonic) -> np.ndarray`.
- Produces: `calibrate_estimator(estimator, rows, output: Path) -> Any`.
- Consumes: non-blocking `source.latest() -> SleeveFrame | None`.

- [ ] **Step 1: Write failing collection tests**

```python
def test_collect_calibration_samples_uses_fresh_ch3_ch4_ch5() -> None:
    source = SequenceSleeveSource([
        SleeveFrame(1.0, (10, 20, 30, 40, 50)),
        SleeveFrame(2.0, (11, 21, 31, 41, 51)),
    ])
    rows = collect_calibration_samples(source, (3, 4, 5), 1.0, monotonic=calibration_clock())
    assert rows.tolist() == [[30.0, 40.0, 50.0], [31.0, 41.0, 51.0]]


def test_collect_calibration_samples_requires_two_valid_rows() -> None:
    source = SequenceSleeveSource([SleeveFrame(1.0, (10, 20, 30, 40, 50))])
    with pytest.raises(RuntimeError, match="only 1 valid samples"):
        collect_calibration_samples(source, (3, 4, 5), 1.0, monotonic=short_clock())
```

Also test duplicate timestamps, missing channels, and invalid rows. Unexpected
source exceptions propagate.

- [ ] **Step 2: Verify RED**

Run: `python -m pytest tests/test_phase4.py -k collect_calibration_samples -v`

Expected: import or assertion failure because the helper does not exist.

- [ ] **Step 3: Implement helpers and update calibration tool**

Validate duration and one-based channels, deduplicate frame timestamps, skip
non-finite rows, and require two valid samples. Implement calibration as:

```python
def calibrate_estimator(estimator, rows: np.ndarray, output: Path):
    calibration = estimator.calibrate(rows)
    calibration.save(output)
    estimator.reset()
    return calibration
```

Update `tools/calibrate_flex_model.py` to use the new config and helpers, with
no legacy dynamic model import.

- [ ] **Step 4: Verify GREEN and commit**

Run: `python -m pytest tests/test_phase4.py tests/test_sources.py -v`

```powershell
git add sleeve_arm/predictor/calibration.py tools/calibrate_flex_model.py tests/test_phase4.py
git commit -m "feat: add flexarm live calibration helpers"
```

---

### Task 4: Replace the Legacy Model Adapter

**Files:**
- Modify: `sleeve_arm/predictor/flex_model.py`
- Modify: `sleeve_arm/predictor/__init__.py`
- Test: `tests/test_phase4.py`

**Interfaces:**
- Consumes `FlexModelConfig` and `FlexArmEstimator.from_pretrained(model_dir)`.
- Produces `FlexModelPredictor.predict(sample) -> MotionIntent`.
- Produces `calibrate(rows, output)` and `reuse_calibration(path)`.

- [ ] **Step 1: Write failing exact-call and action tests**

```python
@dataclass
class EstimatorResult:
    action: str
    angle_deg: float
    action_confidence: float
    angle_confidence: float
    moving: bool


def test_flexarm_adapter_calls_exact_channels_and_timestamp() -> None:
    estimator = FakeEstimator([EstimatorResult("Forward", 90.0, 0.8, 0.7, True)])
    predictor = FlexModelPredictor(model_config(), estimator=estimator)
    intent = predictor.predict(sample(timestamp=1.0))
    assert estimator.calls == [{"flex1": 30.0, "flex2": 40.0, "flex3": 50.0, "timestamp_ns": 1_000_000_000}]
    assert intent.shoulder_flexion_rad == pytest.approx(math.pi / 2)
```

Use literal sample channels that distinguish CH2 from CH3/CH4/CH5. Retain
separate Forward, Backward, and Lateral semantic assertions.

- [ ] **Step 2: Verify RED**

Run: `python -m pytest tests/test_phase4.py -k "flexarm_adapter or action_mapping" -v`

Expected: FAIL because the legacy API is called.

- [ ] **Step 3: Implement model loading and active results**

Import `flexarm` inside `_load_model` so injected unit tests do not require the
wheel. Load once with `FlexArmEstimator.from_pretrained(config.model_dir)`, call
`update` with named scalars and nanosecond timestamp, validate the five output
fields, and preserve the current active action-to-joint mapping. Remove legacy
probability vector and duplicate stabilization logic.

- [ ] **Step 4: Verify active actions GREEN**

Run: `python -m pytest tests/test_phase4.py -k "flexarm_adapter or action_mapping" -v`

- [ ] **Step 5: Write failing Rest/Unknown tests**

```python
def test_rest_holds_last_active_shoulder_target() -> None:
    predictor = FlexModelPredictor(model_config(), estimator=FakeEstimator([
        EstimatorResult("Lateral", 30.0, 0.8, 0.7, True),
        EstimatorResult("Rest", 0.0, 1.0, 1.0, False),
    ]))
    active = predictor.predict(sample(timestamp=1.0))
    resting = predictor.predict(sample(timestamp=1.1))
    assert resting.action is None
    assert resting.model_action == "Rest"
    assert resting.shoulder_flexion_rad == active.shoulder_flexion_rad
    assert resting.shoulder_abduction_rad == active.shoulder_abduction_rad


@pytest.mark.parametrize("state", ("Rest", "Unknown"))
def test_non_active_before_first_action_leaves_shoulders_unset(state: str) -> None:
    intent = FlexModelPredictor(model_config(), estimator=FakeEstimator([
        EstimatorResult(state, 0.0, 0.0, 0.0, False)
    ])).predict(sample())
    assert intent.shoulder_flexion_rad is None
    assert intent.shoulder_abduction_rad is None
```

- [ ] **Step 6: Verify RED, implement holding, then verify GREEN**

Run RED: `python -m pytest tests/test_phase4.py -k "holds_last_active or before_first_action" -v`

Track only last active shoulder targets. For Rest/Unknown, return retained
targets without changing them and preserve all diagnostics. `reset()` clears
estimator and retained state.

Run GREEN: `python -m pytest tests/test_phase4.py -v`

- [ ] **Step 7: Implement and test calibration reuse**

First test that explicit reuse loads the configured file and creates a fresh
estimator with `artifacts.default_calibration` replaced. Implement with
`flexarm.calibration.FlexCalibration.load`, `dataclasses.replace`, and the
loaded estimator's class constructor; wrap incompatible external APIs in a
clear `RuntimeError`.

- [ ] **Step 8: Commit**

```powershell
git add sleeve_arm/predictor/flex_model.py sleeve_arm/predictor/__init__.py tests/test_phase4.py
git commit -m "feat: integrate stateful flexarm estimator"
```

---

### Task 5: Integrate Calibration into Runtime Tools

**Files:**
- Modify: `tools/run_model_control.py`
- Modify: `tools/test_flex_model.py`
- Test: `tests/test_phase4.py`

**Interfaces:**
- Consumes adapter calibration methods from Task 4.
- Produces `prepare_flexarm_predictor(source, config, *, predictor=None, reuse_calibration=False, calibration_seconds=None, calibration_output=None, input_fn=input, print_fn=print, monotonic=time.monotonic) -> FlexModelPredictor`.

- [ ] **Step 1: Write failing preparation tests**

```python
def test_prepare_predictor_live_calibrates_by_default(tmp_path) -> None:
    predictor = RecordingPredictor()
    prepared = prepare_flexarm_predictor(
        SequenceSleeveSource(calibration_frames()), runtime_config(tmp_path),
        predictor=predictor, reuse_calibration=False, calibration_seconds=0.5,
        calibration_output=tmp_path / "live.json", input_fn=lambda _: "",
        monotonic=calibration_clock(),
    )
    assert prepared is predictor
    assert predictor.calibration_rows.tolist() == expected_calibration_rows()


def test_prepare_predictor_reuses_only_when_explicit(tmp_path) -> None:
    predictor = RecordingPredictor()
    path = tmp_path / "saved.json"
    prepare_flexarm_predictor(SequenceSleeveSource([]), runtime_config(tmp_path, calibration_file=path), predictor=predictor, reuse_calibration=True)
    assert predictor.reused_path == path
    assert predictor.calibration_rows is None
```

- [ ] **Step 2: Verify RED**

Run: `python -m pytest tests/test_phase4.py -k prepare_predictor -v`

Expected: FAIL because the helper and CLI behavior do not exist.

- [ ] **Step 3: Implement runtime preparation**

Add `--reuse-calibration`, `--calibration-seconds`, and
`--calibration-output`. Start and validate Sleeve/model readiness before robot
construction and connection. Preserve all existing `--execute` gates. Log
model action, both confidences, moving, CH3/CH4/CH5, angle, inference time, and
existing safety diagnostics. Do not count Rest/Unknown as errors.

Update `tools/test_flex_model.py` to share the same adapter and calibration
policy. Offline replay must use recorded CH3/CH4/CH5 and monotonic timestamps.

- [ ] **Step 4: Verify GREEN**

Run: `python -m pytest tests/test_phase4.py -v`

- [ ] **Step 5: Run no-hardware smoke test when dependencies are available**

Run: `python tools/run_model_control.py --sleeve fake --robot fake --duration 0.2 --reuse-calibration`

Expected: exit 0 without serial or DyMotor access. If the wheel is not
installed, record that dependency gap and rely on the fake-estimator integration
test locally; do not install without authorization.

- [ ] **Step 6: Commit**

```powershell
git add tools/run_model_control.py tools/test_flex_model.py tests/test_phase4.py
git commit -m "feat: add flexarm runtime calibration flow"
```

---

### Task 6: Dependencies, Documentation, and Full Verification

**Files:**
- Modify: `requirements-dev.txt`
- Modify: `README.md`
- Modify: `.gitignore` only if calibration output is not ignored
- Test: `tests/test_phase4.py`

**Interfaces:**
- Documents wheel installation, artifact configuration, calibration policy, diagnostic states, and staged hardware validation.

- [ ] **Step 1: Declare runtime dependencies**

Add `numpy>=1.23`, `joblib>=1.2`, and `scikit-learn==1.7.2`. Document wheel
installation with a relative command, without adding a machine-specific path
to requirements.

- [ ] **Step 2: Update README**

Replace legacy Phase 4 guidance with the new data flow, configuration keys,
live calibration, explicit reuse, Rest/Unknown behavior, FakeRobot test,
DyMotor dry run, and supervised `--execute` sequence.

- [ ] **Step 3: Run focused and full verification**

Run:

```powershell
python -m pytest tests/test_phase4.py -v
python -m pytest -v
git diff --check
git status --short
```

Expected: zero test failures and no staged/untracked task artifacts other than
the pre-existing `.idea/` directory.

- [ ] **Step 4: Commit and verify again**

```powershell
git add requirements-dev.txt README.md .gitignore tests/test_phase4.py
git commit -m "docs: document flexarm estimator runtime"
python -m pytest -v
git status --short
```
