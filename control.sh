#!/usr/bin/env bash
set -euo pipefail

# bash control.sh --print-only: live sensor estimates without robot hardware.
# Defaults: right-arm IMUs + calibrated Bend6 hand; Sleeve is disabled.
# Use --glove none for IMUs only, or --sleeve real to add Sleeve elbow input.
# Other arguments are forwarded, e.g. --duration 30 or --sensor-config <file>.
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
mode=--execute
show_help=false
extra_args=()
for arg in "$@"; do
    case "$arg" in
        --print-only) mode=--print-only ;;
        --help|-h) show_help=true; extra_args+=("$arg") ;;
        *) extra_args+=("$arg") ;;
    esac
done
mode_args=("$mode")
if [[ "$mode" == --execute ]]; then
    mode_args+=(--prepare-aurora-fsm)
fi

# robot_aurora.yaml uses official GR3 velocity limits and matching 100 Hz steps.
# --duration below bounds the whole session, not each joint movement.
robot_python="$(python -c 'import sys; print(sys.executable)')"
runner=("$robot_python" -u)
if [[ "$mode" == --execute && "$show_help" == false ]]; then
    sdk_user_site="$(python -c 'import site; print(site.getusersitepackages())')"
    runner=(sudo env -u LD_LIBRARY_PATH "PYTHONPATH=$sdk_user_site" "$robot_python" -u)
fi

exec "${runner[@]}" \
 tools/run_model_control.py \
--robot aurora \
--aurora-profile configs/robot_aurora.yaml \
--arm-side right --source-side right \
--shoulder-predictor dual_imu \
--sleeve none \
--imus real \
--glove real \
--sensor-config configs/bend6_calibration.measured.sensors.yaml \
--phase3-config configs/phase3.yaml \
--duration 3600 \
--imu-debug \
"${mode_args[@]}" "${extra_args[@]}"
