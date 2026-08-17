#!/usr/bin/env bash

set -Euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime_common.sh
source "${SCRIPT_DIR}/runtime_common.sh"
codexopen_resolve_layout
codexopen_source_ros1

FAST_LIO_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/localization/fast_lio/robot_dog_fast_lio_ros1"
EXPORTER_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/tools/manual_loop_session_exporter/manual_loop_session_exporter_ros1"
STATUS_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/monitor/robot_status_monitor/robot_status_monitor_ros1"
FAST_LIO_CONFIG="${ROBOT_DOG_FAST_LIO_CONFIG:-${CODEXOPEN_INSTALL_ROOT}/bin/config/RobotDogFastLio/RobotDogFastLio.yaml}"
FAST_LIO_INTERFACE="${ROBOT_DOG_FAST_LIO_INTERFACE:-${CODEXOPEN_INSTALL_ROOT}/bin/config/RobotDogFastLio/Interface.yaml}"
EXPORTER_CONFIG="${ROBOT_DOG_EXPORTER_CONFIG:-${CODEXOPEN_INSTALL_ROOT}/bin/config/ManualLoopSessionExporter/ManualLoopSessionExporter.yaml}"
EXPORTER_INTERFACE="${ROBOT_DOG_EXPORTER_INTERFACE:-${CODEXOPEN_INSTALL_ROOT}/bin/config/ManualLoopSessionExporter/Interface.yaml}"
STATUS_CONFIG="${ROBOT_DOG_STATUS_CONFIG:-${CODEXOPEN_INSTALL_ROOT}/bin/config/RobotStatusMonitor/RobotStatusMonitor.ini}"
STATUS_INTERFACE="${ROBOT_DOG_STATUS_INTERFACE:-${CODEXOPEN_INSTALL_ROOT}/bin/config/RobotStatusMonitor/Interface.ini}"
STATUS_MODE="${ROBOT_DOG_STATUS_MODE:-gui}"
RVIZ_ENABLED="${ROBOT_DOG_RVIZ:-true}"
SESSION_ROOT="${ROBOT_DOG_SESSION_ROOT:-/tmp/codexopen_manual_loop_session_$(date '+%Y%m%d_%H%M%S')}"

for executable in "${FAST_LIO_BIN}" "${EXPORTER_BIN}" "${STATUS_BIN}"; do
    codexopen_require_executable "${executable}"
done
for config in "${FAST_LIO_CONFIG}" "${FAST_LIO_INTERFACE}" \
              "${EXPORTER_CONFIG}" "${EXPORTER_INTERFACE}" \
              "${STATUS_CONFIG}" "${STATUS_INTERFACE}"; do
    codexopen_require_file "${config}"
done

if [[ "${1:-}" == "--check" ]]; then
    echo "[OK] robot_dog_rebuild executables and configurations are complete"
    exit 0
fi

CODEXOPEN_LOG_DIR="${ROBOT_DOG_LOG_DIR:-${CODEXOPEN_INSTALL_ROOT}/log/robot_dog_rebuild/$(date '+%Y%m%d_%H%M%S')}"
export CODEXOPEN_LOG_DIR
codexopen_prepare_runtime
trap codexopen_cleanup EXIT INT TERM
if ! codexopen_start_master; then
    exit 1
fi

codexopen_start_process fast_lio \
    "${FAST_LIO_BIN}" "${FAST_LIO_CONFIG}" "${FAST_LIO_INTERFACE}"
codexopen_start_process session_exporter \
    "${EXPORTER_BIN}" "${EXPORTER_CONFIG}" "${EXPORTER_INTERFACE}" "${SESSION_ROOT}"
codexopen_start_process status_monitor \
    "${STATUS_BIN}" "${STATUS_CONFIG}" "${STATUS_INTERFACE}" "${STATUS_MODE}"

if [[ "${RVIZ_ENABLED}" == "true" ]]; then
    codexopen_start_optional_process rviz rviz -d \
        "${CODEXOPEN_INSTALL_ROOT}/bin/config/RobotDogFastLio/RobotDogFastLio.rviz"
fi

echo "[READY] robot_dog_rebuild is running"
echo "[INFO] Session: ${SESSION_ROOT}"
echo "[INFO] Logs:    ${CODEXOPEN_LOG_DIR}"
codexopen_monitor_processes
