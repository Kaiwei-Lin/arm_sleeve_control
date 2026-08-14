# IMU770 Upper-Arm Twist Validation Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build one independently runnable, read-only Python demo that acquires two real IMU770 streams and compares world-referenced upper-arm `+X` twist with the submitted upper-arm-versus-forearm estimator.

**Architecture:** `tools/demo_upper_arm_twist_imu770.py` contains the protocol parser, serial workers, one-use nearest-time synchronizer, quaternion math, both estimators, calibration, CSV logging, and CLI orchestration behind explicit internal interfaces. `tests/test_imu770_upper_arm_twist_demo.py` imports those interfaces and verifies deterministic behavior without adding a simulated runtime mode.

**Tech Stack:** Python 3.10+, NumPy 1.23+, pyserial 3.5+, pytest 7+

## Global Constraints

- Interpret IMU770 field `0x41` as Sensor-to-World `[w, x, y, z]` values scaled by `1e-6`.
- Use the upper-arm IMU local `+X` axis from shoulder to elbow.
- Open two IMU770 ports read-only at 460800 baud, 8N1 by default; never send device commands.
- Reject synchronized pairs whose host-monotonic timestamp gap exceeds 20 ms by default.
- Deliver no `--simulate` mode and no imports from `sleeve_arm` or `arm_collector`.
- Preserve unrelated staged `.idea` files and do not include them in feature commits.

---

## File Structure

- Create `tools/demo_upper_arm_twist_imu770.py`: all standalone runtime code and importable test seams.
- Create `tests/test_imu770_upper_arm_twist_demo.py`: focused parser, math, synchronization, calibration, CSV, CLI, and lifecycle tests.
- Modify `README.md`: hardware assumptions, invocation, calibration, validation actions, output interpretation, and limitations.

### Task 1: Decode IMU770 frames into standalone samples

**Files:**
- Create: `tools/demo_upper_arm_twist_imu770.py`
- Create: `tests/test_imu770_upper_arm_twist_demo.py`

**Interfaces:**
- Produces: immutable `Imu770Sample(host_timestamp_ns: int, tid: int, device_timestamp_us: int | None, dataready_timestamp_us: int | None, accel_mps2: tuple[float, float, float] | None, gyro_dps: tuple[float, float, float] | None, euler_deg: tuple[float, float, float] | None, quaternion_wxyz: tuple[float, float, float, float] | None)`.
- Produces: `Imu770FrameParser.feed(data: bytes, *, host_timestamp_ns: int | None = None) -> list[Imu770Sample]`.
- Produces parser counters `frame_count`, `valid_frame_count`, `checksum_error_count`, `parser_error_count`, and `tid_drop_count`.

- [ ] **Step 1: Write failing parser tests**

Add a `make_imu770_frame()` helper that constructs `59 53 + <tid:uint16> + <payload_length:uint8> + TLVs + CK1 CK2`, where CK1/CK2 are accumulated over bytes starting at TID. Assert that fragmented input yields nothing until complete, concatenated frames yield both samples, `0x41=(1_000_000,0,0,0)` becomes `(1.0,0.0,0.0,0.0)`, timestamps decode as unsigned integers, unknown TLVs are skipped, and accel/gyro may be absent when a valid quaternion is present.

```python
def test_parser_decodes_quaternion_only_frame() -> None:
    parser = demo.Imu770FrameParser()
    raw = make_imu770_frame(7, [(0x41, (1_000_000, 0, 0, 0)), (0x51, 123456)])
    samples = parser.feed(raw, host_timestamp_ns=999)
    assert samples[0].quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert samples[0].device_timestamp_us == 123456
    assert samples[0].host_timestamp_ns == 999
```

- [ ] **Step 2: Run parser tests and confirm RED**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -v`

Expected: collection fails because `tools.demo_upper_arm_twist_imu770` does not exist.

- [ ] **Step 3: Implement the sample and stream parser**

Implement header resynchronization, payload-length framing, Fletcher-style CK1/CK2 verification, TLV length validation for IDs `0x10`, `0x20`, `0x40`, `0x41`, `0x51`, and `0x52`, `int32 * 1e-6` scaling for vector fields, and `uint32` timestamps. A valid frame is published when `0x41` exists, even if acceleration or gyro fields are absent. Count malformed candidates without terminating the stream and count positive TID gaps with wrap from 60000 to 1.

- [ ] **Step 4: Run parser tests and confirm GREEN**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -v`

Expected: all Task 1 tests pass.

- [ ] **Step 5: Commit parser slice**

```powershell
git add tools/demo_upper_arm_twist_imu770.py tests/test_imu770_upper_arm_twist_demo.py
git commit -m "feat: decode standalone IMU770 quaternion frames"
```

### Task 2: Implement calibrated twist math and both estimators

**Files:**
- Modify: `tools/demo_upper_arm_twist_imu770.py`
- Modify: `tests/test_imu770_upper_arm_twist_demo.py`

**Interfaces:**
- Consumes: `Imu770Sample.quaternion_wxyz` from Task 1.
- Produces: `normalize_quaternion`, `quaternion_inverse`, `quaternion_multiply`, and `average_quaternions` functions using NumPy arrays in `[w,x,y,z]` order.
- Produces: immutable `TwistResult(raw_deg: float, unwrapped_deg: float, filtered_deg: float)`.
- Produces: `TwistEstimator(ema_alpha: float)` with `calibrate(quaternions: Sequence[Sequence[float]]) -> None` and `update(quaternion: Sequence[float]) -> TwistResult`.
- Produces: `RelativeTwistEstimator(ema_alpha: float)` with `calibrate(pairs: Sequence[tuple[Sequence[float], Sequence[float]]]) -> None` and `update(upper: Sequence[float], forearm: Sequence[float]) -> TwistResult`.

- [ ] **Step 1: Write failing quaternion and estimator tests**

Construct axis-angle quaternions in the tests. Verify rejection of zero/non-finite quaternions, sign-aligned averaging of `q` and `-q`, exact `+30°` and `-45°` twists about `+X`, rejection of pure `Y` swing as twist, unwrapping from `+179°` to `+181°`, and EMA behavior with a known alpha. Verify that the world estimator remains at the injected upper twist when only the forearm flexion quaternion changes, while the relative estimator receives the changed forearm orientation.

```python
def test_world_estimator_is_independent_of_forearm_flexion() -> None:
    estimator = demo.TwistEstimator(ema_alpha=1.0)
    estimator.calibrate([(1.0, 0.0, 0.0, 0.0)])
    result = estimator.update(axis_angle((1, 0, 0), 35.0))
    assert result.filtered_deg == pytest.approx(35.0)
```

- [ ] **Step 2: Run estimator tests and confirm RED**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -k "quaternion or twist or estimator" -v`

Expected: failures for missing math and estimator interfaces.

- [ ] **Step 3: Implement quaternion math and estimators**

Use `q_delta = inverse(q_zero) * q_current` for the world estimator. Use `q_relative = inverse(q_forearm) * q_upper`, followed by `inverse(q_relative_zero) * q_relative`, for the relative estimator. Extract `+X` twist by projecting quaternion vector part onto `(1,0,0)`, normalize the twist quaternion, calculate `2*atan2(x,w)`, wrap to `[-180,180)`, unwrap relative to the prior raw value, and filter the continuous value. Validate `0 < ema_alpha <= 1` and require calibration before update.

- [ ] **Step 4: Run estimator tests and confirm GREEN**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -k "quaternion or twist or estimator" -v`

Expected: all selected tests pass.

- [ ] **Step 5: Commit estimator slice**

```powershell
git add tools/demo_upper_arm_twist_imu770.py tests/test_imu770_upper_arm_twist_demo.py
git commit -m "feat: compare world and relative IMU twist"
```

### Task 3: Add one-use synchronization and dual serial workers

**Files:**
- Modify: `tools/demo_upper_arm_twist_imu770.py`
- Modify: `tests/test_imu770_upper_arm_twist_demo.py`

**Interfaces:**
- Consumes: `Imu770FrameParser` and `Imu770Sample` from Task 1.
- Produces: `SampleSynchronizer(max_gap_ns: int, max_queue: int = 256)` with `add_upper`, `add_forearm`, and `pop_pair() -> tuple[Imu770Sample, Imu770Sample, int] | None`.
- Produces: `Imu770SerialReader(port: str, baudrate: int, timeout_s: float, on_sample: Callable[[Imu770Sample], None], serial_factory: Callable[..., object] | None = None)` with `start()`, `close()`, `error`, and `stats`.

- [ ] **Step 1: Write failing synchronizer and serial lifecycle tests**

Create samples with controlled nanosecond timestamps. Assert nearest pairing, one-use consumption, stale-sample rejection beyond 20 ms, bounded queues, and no duplicate pair emission. Provide a fake serial object and assert 460800/8N1 opening arguments, parsed callback delivery, prompt close, and surfaced reader exceptions. Assert the writer method is never called.

```python
def test_synchronizer_consumes_each_sample_once() -> None:
    sync = demo.SampleSynchronizer(max_gap_ns=20_000_000)
    sync.add_upper(sample_at(100_000_000))
    sync.add_forearm(sample_at(104_000_000))
    assert sync.pop_pair()[2] == 4_000_000
    assert sync.pop_pair() is None
```

- [ ] **Step 2: Run acquisition tests and confirm RED**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -k "synchronizer or serial" -v`

Expected: failures for missing synchronization and serial-reader interfaces.

- [ ] **Step 3: Implement synchronization and reader lifecycle**

Protect queues and reader state with locks. Pair the available timestamps with minimum absolute gap, discard the older head when no acceptable pair can be formed, and never reuse a sample. The serial worker lazily imports pyserial, opens with explicit 8N1 constants, reads up to 4096 bytes with a finite timeout, feeds the parser, calls `on_sample`, records rates/errors, and closes/join its non-daemon thread without sending bytes.

- [ ] **Step 4: Run acquisition tests and confirm GREEN**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -k "synchronizer or serial" -v`

Expected: all selected tests pass.

- [ ] **Step 5: Commit acquisition slice**

```powershell
git add tools/demo_upper_arm_twist_imu770.py tests/test_imu770_upper_arm_twist_demo.py
git commit -m "feat: synchronize dual IMU770 serial streams"
```

### Task 4: Build calibration, live comparison, CSV, and CLI

**Files:**
- Modify: `tools/demo_upper_arm_twist_imu770.py`
- Modify: `tests/test_imu770_upper_arm_twist_demo.py`

**Interfaces:**
- Consumes: synchronized sample triples and both estimators from Tasks 2 and 3.
- Produces: `build_argument_parser() -> argparse.ArgumentParser`.
- Produces: `CsvRecorder(path: Path | None)` context manager with `write(upper, forearm, gap_ns, world_result, relative_result) -> None`.
- Produces: `run(args: argparse.Namespace, *, serial_factory: Callable[..., object] | None = None, input_fn: Callable[[str], str] = input) -> int` and `main() -> int`.

- [ ] **Step 1: Write failing CLI, calibration, and CSV tests**

Assert required distinct upper/forearm ports; positive baud, timeout, calibration duration, print rate and sync gap; `0 < ema_alpha <= 1`; defaults of 460800 baud and 20 ms. Feed synchronized real-frame bytes through two fake serial ports, make `input_fn` return immediately, and assert calibration collects multiple pairs, the CSV header has stable raw input and derived fields, rows include both filtered angles and the gap, and both ports close when acquisition ends or one reader fails.

```python
def test_cli_rejects_same_port_for_both_sensors() -> None:
    parser = demo.build_argument_parser()
    with pytest.raises(SystemExit):
        parser.parse_args(["--upper-port", "COM5", "--forearm-port", "COM5"])
```

- [ ] **Step 2: Run orchestration tests and confirm RED**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -k "cli or csv or calibration or run" -v`

Expected: failures for missing CLI and orchestration behavior.

- [ ] **Step 3: Implement CLI and real-hardware loop**

Wait up to a bounded startup deadline for valid quaternion data from both ports. Prompt for the aligned zero pose, collect synchronized pairs for `--calibration-seconds`, calibrate both estimators, then drain new pairs and print at `--print-hz`. Emit world, relative, difference, synchronization gap, per-port frame rates, and parser-error counters. Write the full CSV schema, flush once per second, detect reader errors, and guarantee both readers and the recorder close in `finally`. Return 0 on Ctrl+C and nonzero with a concise message on startup, serial, calibration, or runtime failure.

- [ ] **Step 4: Run all Demo tests and confirm GREEN**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -v`

Expected: all Demo tests pass.

- [ ] **Step 5: Commit executable Demo**

```powershell
git add tools/demo_upper_arm_twist_imu770.py tests/test_imu770_upper_arm_twist_demo.py
git commit -m "feat: add live dual IMU770 twist validation demo"
```

### Task 5: Document and verify the deliverable

**Files:**
- Modify: `README.md`
- Test: `tests/test_imu770_upper_arm_twist_demo.py`

**Interfaces:**
- Consumes: the final CLI from Task 4.
- Produces: operator instructions and recorded verification evidence.

- [ ] **Step 1: Add README usage and physical test instructions**

Document the confirmed `[w,x,y,z]` Sensor-to-World semantics, both local `+X` mounting directions, aligned/zero-calibrated pose, command example, CSV columns, the four physical trials, expected contrast between the two angles, and the limitation that body-motion rejection needs a torso reference. State explicitly that the program never writes device configuration.

- [ ] **Step 2: Verify CLI help and syntax without opening hardware**

Run: `python tools/demo_upper_arm_twist_imu770.py --help`

Expected: exit 0; help lists both required ports and contains no simulation option.

Run: `python -m py_compile tools/demo_upper_arm_twist_imu770.py`

Expected: exit 0.

- [ ] **Step 3: Run focused and full automated tests**

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest tests/test_imu770_upper_arm_twist_demo.py -v`

Expected: all Demo tests pass.

Run: `$env:PYTEST_DISABLE_PLUGIN_AUTOLOAD='1'; python -m pytest -q`

Expected: no new failures. If the repository's previously known three unrelated baseline failures remain, report them separately with exact test names rather than attributing them to this feature.

- [ ] **Step 4: Inspect final diff and working tree isolation**

Run: `git diff --check`

Expected: no whitespace errors in feature files.

Run: `git status --short`

Expected: only the user's pre-existing staged `.idea` files remain outside the committed feature files.

- [ ] **Step 5: Commit documentation**

```powershell
git add README.md
git commit -m "docs: explain IMU770 twist validation workflow"
```

## Hardware Acceptance

Automated verification ends before opening real COM ports. The operator then runs the documented command with the two IMU770 devices and performs the four physical trials. Acceptance evidence is the saved CSV plus the observation that `upper_world_twist` responds to upper-arm `+X` rotation but remains substantially steadier than `upper_vs_forearm_twist` during elbow-only flexion and extension. Exact accuracy tolerances must be established with an external angular reference if required.
