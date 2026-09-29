robot_python="$(python -c 'import sys; print(sys.executable)')"
sdk_user_site="$(python -c 'import site; print(site.getusersitepackages())')"

sudo env -u LD_LIBRARY_PATH "PYTHONPATH=$sdk_user_site" "$robot_python" \
 tools/aurora_arm_debug.py \
--profile configs/aurora_196_shoulder.yaml \
--side right \
--reference neutral \
--action 1 \
--angle-deg 30 \
--execute
