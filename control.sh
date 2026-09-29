#!/usr/bin/env bash
set -euo pipefail

# bash control.sh --print-only: live sensor estimates without robot hardware.
# Dual-IMU Aurora control and print-only skip Sleeve by default.
# Add --sleeve real to enable Sleeve-based elbow input.
# Other arguments are forwarded, e.g. --duration 30 or --sleeve fake --imus fake.
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
--imus real \
--sensor-config configs/sensors.yaml \
--phase3-config configs/phase3.yaml \
--duration 3600 \
--imu-debug \
"$mode" "${extra_args[@]}"
