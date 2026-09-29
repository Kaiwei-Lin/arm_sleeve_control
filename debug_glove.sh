robot_python="$(python -c 'import sys; print(sys.executable)')"
sdk_user_site="$(python -c 'import site; print(site.getusersitepackages())')"

sudo env -u LD_LIBRARY_PATH "PYTHONPATH=$sdk_user_site" "$robot_python" \
 tools/debug_glove.py --duration 3000 \
--execute
