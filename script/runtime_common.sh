#!/usr/bin/env bash

set -Euo pipefail

declare -a CODEXOPEN_MANAGED_PIDS=()
declare -a CODEXOPEN_MANAGED_NAMES=()
declare -a CODEXOPEN_MANAGED_REQUIRED=()
CODEXOPEN_MASTER_PID=""

codexopen_resolve_layout()
{
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[1]}")" && pwd)"
    if [[ -d "${script_dir}/../bin/config" ]]; then
        CODEXOPEN_INSTALL_ROOT="${CODEXOPEN_INSTALL_ROOT:-$(cd "${script_dir}/.." && pwd)}"
        CODEXOPEN_PROJECT_ROOT="${CODEXOPEN_PROJECT_ROOT:-$(cd "${CODEXOPEN_INSTALL_ROOT}/.." && pwd)}"
    else
        CODEXOPEN_PROJECT_ROOT="${CODEXOPEN_PROJECT_ROOT:-$(cd "${script_dir}/.." && pwd)}"
        CODEXOPEN_INSTALL_ROOT="${CODEXOPEN_INSTALL_ROOT:-${CODEXOPEN_PROJECT_ROOT}/install}"
    fi
    export CODEXOPEN_PROJECT_ROOT CODEXOPEN_INSTALL_ROOT
}

codexopen_source_ros1()
{
    local ros_setup="${CODEXOPEN_ROS1_SETUP:-/opt/ros/noetic/setup.bash}"
    if [[ ! -f "${ros_setup}" ]]; then
        echo "[ERROR] ROS1 setup not found: ${ros_setup}" >&2
        return 1
    fi
    # shellcheck disable=SC1090
    source "${ros_setup}"
}

codexopen_prepare_runtime()
{
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
    export ROS_HOME="${ROS_HOME:-${CODEXOPEN_LOG_DIR}/ros_home}"
    export ROS_LOG_DIR="${ROS_LOG_DIR:-${CODEXOPEN_LOG_DIR}/ros_log}"
    mkdir -p "${CODEXOPEN_LOG_DIR}" "${ROS_HOME}" "${ROS_LOG_DIR}"
}

codexopen_require_file()
{
    if [[ ! -f "$1" ]]; then
        echo "[ERROR] Required file not found: $1" >&2
        return 1
    fi
}

codexopen_require_executable()
{
    if [[ ! -x "$1" ]]; then
        echo "[ERROR] Executable not found: $1" >&2
        echo "        Build with BUILD_ROS1=ON, then run ninja install." >&2
        return 1
    fi
}

codexopen_start_master()
{
    if timeout 2 rosparam list >/dev/null 2>&1; then
        echo "[INFO] Reusing ROS master at ${ROS_MASTER_URI:-http://127.0.0.1:11311}"
        return 0
    fi
    roscore >"${CODEXOPEN_LOG_DIR}/roscore.log" 2>&1 &
    CODEXOPEN_MASTER_PID=$!
    for _ in $(seq 1 50); do
        if timeout 1 rosparam list >/dev/null 2>&1; then
            echo "[INFO] ROS master started (PID=${CODEXOPEN_MASTER_PID})"
            return 0
        fi
        if ! kill -0 "${CODEXOPEN_MASTER_PID}" 2>/dev/null; then
            echo "[ERROR] roscore exited during startup" >&2
            return 1
        fi
        sleep 0.2
    done
    echo "[ERROR] ROS master did not become ready" >&2
    return 1
}

codexopen_start_process()
{
    local name="$1"
    shift
    "$@" >"${CODEXOPEN_LOG_DIR}/${name}.log" 2>&1 &
    local pid=$!
    CODEXOPEN_MANAGED_NAMES+=("${name}")
    CODEXOPEN_MANAGED_PIDS+=("${pid}")
    CODEXOPEN_MANAGED_REQUIRED+=(1)
    sleep 0.2
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[ERROR] ${name} exited during startup" >&2
        tail -n 40 "${CODEXOPEN_LOG_DIR}/${name}.log" >&2 || true
        return 1
    fi
    echo "[INFO] ${name} started (PID=${pid})"
}

codexopen_start_optional_process()
{
    local name="$1"
    shift
    "$@" >"${CODEXOPEN_LOG_DIR}/${name}.log" 2>&1 &
    local pid=$!
    CODEXOPEN_MANAGED_NAMES+=("${name}")
    CODEXOPEN_MANAGED_PIDS+=("${pid}")
    CODEXOPEN_MANAGED_REQUIRED+=(0)
    sleep 0.2
    if ! kill -0 "${pid}" 2>/dev/null; then
        echo "[WARN] Optional process ${name} exited during startup" >&2
        tail -n 20 "${CODEXOPEN_LOG_DIR}/${name}.log" >&2 || true
        return 0
    fi
    echo "[INFO] ${name} started (PID=${pid}, optional)"
}

codexopen_cleanup()
{
    local exit_code=$?
    trap - EXIT INT TERM
    set +e
    for ((index=${#CODEXOPEN_MANAGED_PIDS[@]}-1; index>=0; --index)); do
        kill -INT "${CODEXOPEN_MANAGED_PIDS[index]}" 2>/dev/null
    done
    for _ in $(seq 1 20); do
        local alive=0
        for pid in "${CODEXOPEN_MANAGED_PIDS[@]}"; do
            if kill -0 "${pid}" 2>/dev/null; then
                alive=1
            fi
        done
        [[ "${alive}" -eq 0 ]] && break
        sleep 0.1
    done
    for pid in "${CODEXOPEN_MANAGED_PIDS[@]}"; do
        kill -TERM "${pid}" 2>/dev/null
    done
    if [[ -n "${CODEXOPEN_MASTER_PID}" ]]; then
        kill -INT "${CODEXOPEN_MASTER_PID}" 2>/dev/null
    fi
    wait 2>/dev/null
    echo "[INFO] Managed processes stopped"
    exit "${exit_code}"
}

codexopen_monitor_processes()
{
    while true; do
        for index in "${!CODEXOPEN_MANAGED_PIDS[@]}"; do
            [[ "${CODEXOPEN_MANAGED_REQUIRED[index]}" -eq 0 ]] && continue
            if ! kill -0 "${CODEXOPEN_MANAGED_PIDS[index]}" 2>/dev/null; then
                echo "[ERROR] ${CODEXOPEN_MANAGED_NAMES[index]} stopped unexpectedly" >&2
                tail -n 40 "${CODEXOPEN_LOG_DIR}/${CODEXOPEN_MANAGED_NAMES[index]}.log" >&2 || true
                return 1
            fi
        done
        sleep 1
    done
}
