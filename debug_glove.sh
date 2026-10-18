#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
robot_python="$(python -c 'import sys; print(sys.executable)')"
runner=("$robot_python" -u)
show_help=false
for arg in "$@"; do
    case "$arg" in --help|-h) show_help=true ;; esac
done
if [[ "$show_help" == false ]]; then
    sdk_user_site="$(python -c 'import site; print(site.getusersitepackages())')"
    runner=(sudo env -u LD_LIBRARY_PATH "PYTHONPATH=$sdk_user_site" "$robot_python" -u)
fi

# Lift the selected arm for hand clearance. Use --arm-forward-deg 0 for hand only.
exec "${runner[@]}" tools/debug_glove.py \
    --duration 3000 --execute --arm-forward-deg 45 --arm-move-seconds 5 "$@"
