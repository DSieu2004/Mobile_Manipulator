#!/usr/bin/env bash
set -eo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$#" -eq 0 ]]; then
    exec "$project_root/.venv/bin/python" "$project_root/scripts/draw_panel.py"
fi
source /opt/ros/humble/setup.bash
source "$project_root/install/setup.bash"
set -u
export LD_LIBRARY_PATH="$project_root/third_party/ros/opt/ros/humble/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"

headless=false
for argument in "$@"; do
    if [[ "$argument" == "--headless" ]]; then
        headless=true
    fi
done

if [[ "$headless" == false ]]; then
    # Select the RTX 4050 through PRIME rather than Mesa llvmpipe or the AMD iGPU.
    export LIBGL_ALWAYS_SOFTWARE=0
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export MUJOCO_GL=glfw
    if ! command -v glxinfo >/dev/null 2>&1; then
        echo "Missing glxinfo; install mesa-utils to verify the MuJoCo renderer." >&2
        exit 1
    fi
    renderer="$(glxinfo -B 2>/dev/null | sed -n 's/^OpenGL renderer string: //p' | head -n 1)"
    if [[ "$renderer" != *"NVIDIA GeForce RTX 4050"* ]]; then
        echo "Expected RTX 4050, but OpenGL selected: ${renderer:-unavailable}. Check NVIDIA PRIME/driver." >&2
        exit 1
    fi
    echo "MuJoCo OpenGL renderer: $renderer"
fi

mpc_log="$project_root/log/ocs2_mpc_latest.log"
mkdir -p "$project_root/log"
ros2 launch husky_ur5e_description ocs2_mujoco.launch.py >"$mpc_log" 2>&1 &
mpc_pid=$!
cleanup() {
    kill "$mpc_pid" 2>/dev/null || true
    wait "$mpc_pid" 2>/dev/null || true
}
trap cleanup EXIT

if ! "$project_root/.venv/bin/python" "$project_root/scripts/run_ocs2_mujoco.py" "$@"; then
    tail -n 30 "$mpc_log"
    exit 1
fi
