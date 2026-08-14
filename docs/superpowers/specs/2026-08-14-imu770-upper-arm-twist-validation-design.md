# IMU770 Upper-Arm Twist Validation Demo Design

## Goal

Create one independently runnable Python demo that reads two real IMU770 devices and tests whether an upper-arm twist estimator remains valid while the elbow flexes and extends. The upper-arm IMU local `+X` axis is aligned from shoulder to elbow. IMU770 quaternion field `0x41` is interpreted as `[w, x, y, z]` and Sensor-to-World.

The demo compares the supplied upper-arm-versus-forearm estimator with an upper-arm-versus-calibrated-world estimator. It records enough raw and derived data to expose elbow-motion cross-talk instead of declaring physical accuracy without an external ground-truth instrument.

## Scope

The deliverable consists of:

- `tools/demo_upper_arm_twist_imu770.py`: one self-contained, real-hardware demo.
- Focused automated tests for binary parsing, quaternion math, synchronization, calibration, angle unwrapping, and filtering.
- A short README section with wiring assumptions, launch commands, calibration steps, and the physical validation procedure.

The executable depends only on Python, NumPy, and pyserial. It will not import `sleeve_arm` or the unavailable `arm_collector` package and will not change IMU configuration. It will not contain a simulation mode.

## Command-Line Interface

The primary command is:

```text
python tools/demo_upper_arm_twist_imu770.py --upper-port COM5 --forearm-port COM6 --csv upper_arm_twist_test.csv
```

Both devices default to 460800 baud, eight data bits, no parity, one stop bit, and a finite read timeout. Options allow the user to adjust the baud rate, timeout, calibration duration, console refresh rate, maximum synchronization gap, EMA coefficient, and CSV path. The quaternion order and direction are fixed to the confirmed IMU770 semantics rather than exposed as unnecessary runtime choices.

After both streams produce valid quaternions, the program prompts the user to hold the agreed zero pose and press Enter. It then averages a short window of synchronized samples and starts live output. Ctrl+C stops acquisition cleanly.

## Architecture

The implementation remains in one executable file for portability, with these internal boundaries:

1. An IMU770 stream parser buffers arbitrary serial chunks, locates `59 53` frames, validates the frame checksum and TLV lengths, and decodes acceleration, angular velocity, Euler angles, timestamps, and quaternion field `0x41`.
2. Two serial reader workers own the upper-arm and forearm ports. Each publishes timestamped decoded samples plus acquisition and parser statistics. They only read and never send commands to either device.
3. A timestamp synchronizer consumes each sample at most once and pairs upper-arm and forearm samples with the smallest available host-monotonic-time gap. Pairs beyond the configured gap, defaulting to 20 ms, are rejected.
4. Quaternion utilities handle normalization, inversion, multiplication, sign-aligned averaging, swing-twist decomposition around local `+X`, angle wrapping/unwrapping, and EMA filtering.
5. Two estimator instances consume each synchronized pair and publish comparable results to the console and CSV recorder.

The parser and math remain independently testable even though they are packaged in the same executable.

## Angle Estimators

For every valid input, the quaternion is normalized. Zero-length and non-finite quaternions are rejected.

The world-referenced estimator uses:

```text
q_delta_world = inverse(q_upper_zero) * q_upper_current
```

It decomposes `q_delta_world` into swing and twist about upper-arm local `+X`. Because the forearm quaternion is not part of this calculation, elbow flexion alone should not change this result. It remains world-referenced, so body or sensor-heading changes are not automatically removed.

The supplied relative estimator uses:

```text
q_relative = inverse(q_forearm_current) * q_upper_current
q_delta_relative = inverse(q_relative_zero) * q_relative
```

Here `q_relative_zero` is the sign-aligned average of the relative quaternions from the calibration pairs. The estimator applies the same swing-twist decomposition around `+X`. This preserves the submitted algorithm for direct comparison, while making its potential cancellation of common upper/forearm motion and sensitivity to elbow-relative motion measurable.

Each estimator reports a raw angle in `[-180, 180)`, an unwrapped continuous angle, and an EMA-filtered continuous angle. Calibration uses sign-aligned quaternion averaging over synchronized samples rather than a single frame.

## Live Output and CSV

The console periodically shows:

- Filtered world-referenced and relative twist angles.
- Their difference.
- Pair synchronization gap.
- Valid frame rates for each IMU.
- Rejected, malformed, and checksum-failed frame counts.

CSV rows contain host timestamps, device timestamps and sequence IDs, both normalized quaternions, raw/unwrapped/filtered values from both estimators, their difference, and the synchronization gap. The CSV is flushed periodically and closed on normal or interrupted shutdown.

## Physical Validation Procedure

After zero calibration, the operator performs four real-hardware trials:

1. Hold the zero pose and observe drift.
2. Keep the elbow angle fixed and rotate the upper arm around its `+X` axis in known directions.
3. Avoid intentional upper-arm axial rotation and repeatedly flex and extend the elbow.
4. Hold approximately the same upper-arm axial rotation while repeating the test at different elbow angles.

The expected diagnostic result is that the world-referenced output follows upper-arm axial rotation and remains substantially steadier during elbow-only motion than the relative output. The two IMUs alone do not provide traceable angular ground truth, so numerical accuracy claims require an external encoder, optical tracker, or measured mechanical fixture. A torso IMU would also be required later if body motion must be removed from the world-referenced result.

## Error Handling and Shutdown

- A missing pyserial dependency produces an installation hint.
- Failure to open either port identifies the failing port and closes the other port.
- A stream that does not produce valid quaternion frames within a bounded startup period reports which sensor is missing data.
- Invalid checksums, malformed TLVs, non-finite quaternions, sequence gaps, and unsynchronized pairs are counted and skipped without terminating acquisition.
- Reader failure or physical disconnection stops the run with a nonzero exit status and final statistics.
- Ctrl+C stops both reader workers, closes both serial handles and the CSV file, and exits successfully.

## Testing

Automated tests use constructed byte frames only to verify deterministic parser and math behavior; the delivered command has no simulated runtime mode. Tests cover:

- Fragmented, concatenated, malformed, and checksum-invalid IMU770 frames.
- Correct `0x41` `[w, x, y, z]` scaling and field extraction.
- Quaternion normalization, inverse, product, averaging, and `+X` swing-twist decomposition.
- Positive and negative angles, crossing `±180` degrees, and EMA filtering.
- Timestamp pair acceptance and rejection around the configured limit.
- Multi-frame calibration and rejection of invalid samples.
- CSV column stability and command-line validation.

Automated tests cannot replace the four real-hardware trials, which are the acceptance test for the physical scheme.
