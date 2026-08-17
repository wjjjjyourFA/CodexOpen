#!/usr/bin/env bash

set -Euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime_common.sh
source "${SCRIPT_DIR}/runtime_common.sh"
codexopen_resolve_layout
codexopen_source_ros1

LOCALIZATION_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/localization/prior_map_localization/localization_robot_ros1"
LOCALIZATION_CONFIG_DIR="${ROBOT_DOG_LOCALIZATION_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/PriorMapLocalization}"
LOCALIZATION_RUNTIME="${ROBOT_DOG_LOCALIZATION_RUNTIME:-${LOCALIZATION_CONFIG_DIR}/PriorMapLocalization.yaml}"
LOCALIZATION_INTERFACE="${ROBOT_DOG_LOCALIZATION_INTERFACE:-${LOCALIZATION_CONFIG_DIR}/Interface.yaml}"
LOCALIZATION_MAP="${ROBOT_DOG_LOCALIZATION_MAP:-${LOCALIZATION_CONFIG_DIR}/data/global_map_manual_opt.pcd}"
LOCALIZATION_RVIZ_CONFIG="${ROBOT_DOG_LOCALIZATION_RVIZ_CONFIG:-${LOCALIZATION_CONFIG_DIR}/rviz/PriorMapLocalization.rviz}"
RVIZ_ENABLED="${ROBOT_DOG_LOCALIZATION_RVIZ:-true}"
RVIZ_LEAD_TIME="${ROBOT_DOG_LOCALIZATION_RVIZ_LEAD_TIME:-1}"

codexopen_require_executable "${LOCALIZATION_BIN}"
codexopen_require_file "${LOCALIZATION_RUNTIME}"
codexopen_require_file "${LOCALIZATION_INTERFACE}"
codexopen_require_file "${LOCALIZATION_MAP}"
if [[ "${RVIZ_ENABLED}" == "true" ]]; then
    codexopen_require_file "${LOCALIZATION_RVIZ_CONFIG}"
fi

if [[ "${1:-}" == "--check" ]]; then
    echo "[OK] robot_dog_localization executable, configuration, map and RViz resources are complete"
    exit 0
fi

CODEXOPEN_LOG_DIR="${ROBOT_DOG_LOCALIZATION_LOG_DIR:-${CODEXOPEN_INSTALL_ROOT}/log/robot_dog_localization/$(date '+%Y%m%d_%H%M%S')}"
export CODEXOPEN_LOG_DIR
codexopen_prepare_runtime
trap codexopen_cleanup EXIT INT TERM

if ! codexopen_start_master; then
    exit 1
fi

# The localization adapter publishes the prior map with a latched publisher.
# Starting RViz first preserves the familiar initialization workflow, but a
# later subscriber can still receive the most recently published map.
# RViz is optional: runtime_common does not treat its exit as a flow failure.
if [[ "${RVIZ_ENABLED}" == "true" ]]; then
    codexopen_start_optional_process localization_rviz \
        rviz -d "${LOCALIZATION_RVIZ_CONFIG}"
    sleep "${RVIZ_LEAD_TIME}"
fi

codexopen_start_process prior_map_localization \
    "${LOCALIZATION_BIN}" \
    "${LOCALIZATION_RUNTIME}" \
    "${LOCALIZATION_INTERFACE}" \
    "${LOCALIZATION_MAP}" \
    "${CODEXOPEN_LOG_DIR}/localization"

echo "[READY] robot_dog_localization is running"
echo "[INFO] Use RViz '2D Pose Estimate' to provide the initial pose"
echo "[INFO] Closing RViz will not stop the localization process"
echo "[INFO] Logs: ${CODEXOPEN_LOG_DIR}"
codexopen_monitor_processes
