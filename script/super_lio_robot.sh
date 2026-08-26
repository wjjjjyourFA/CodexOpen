#!/usr/bin/env bash

set -Euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime_common.sh
source "${SCRIPT_DIR}/runtime_common.sh"
codexopen_resolve_layout
codexopen_source_ros1

usage()
{
    cat <<'USAGE'
Usage: super_lio_robot.sh [--mode MODE] [--check]

Modes:
  livox-mid360          Livox Mid-360 input
  quadruped-pcd         Quadruped normal mapping/PCD-compatible output
  quadruped-relocation  Quadruped prior-map relocation output

Aliases livox_360, pcd_normal_dog_output and relocation_dog_output are accepted.
USAGE
}

MODE="${SUPER_LIO_ROBOT_MODE:-livox-mid360}"
CHECK_ONLY=false
while (($# > 0)); do
    case "$1" in
        --mode)
            if (($# < 2)); then
                echo "[ERROR] --mode requires a value" >&2
                exit 2
            fi
            MODE="$2"
            shift 2
            ;;
        --check)
            CHECK_ONLY=true
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

SUPER_LIO_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/mapping/super_lio_robot/super_lio_robot_ros1"
SUPER_LIO_CONFIG_DIR="${SUPER_LIO_ROBOT_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/SuperLioRobot}"
SUPER_LIO_OUTPUT_ROOT="${SUPER_LIO_ROBOT_OUTPUT_ROOT:-${CODEXOPEN_PROJECT_ROOT}/tmp}"

case "${MODE}" in
    livox-mid360|livox_360|mid360)
        MODE="livox-mid360"
        RUNTIME_CONFIG="${SUPER_LIO_CONFIG_DIR}/SuperLioRobotLivoxMid360.yaml"
        INTERFACE_CONFIG="${SUPER_LIO_CONFIG_DIR}/InterfaceLivoxMid360.yaml"
        RVIZ_CONFIG="${SUPER_LIO_CONFIG_DIR}/rviz/SuperLioRobot.rviz"
        OPERATION="mapping"
        MAP_ROOT="${SUPER_LIO_OUTPUT_ROOT}"
        ;;
    quadruped-pcd|pcd_normal_dog_output|pcd)
        MODE="quadruped-pcd"
        RUNTIME_CONFIG="${SUPER_LIO_CONFIG_DIR}/SuperLioRobotQuadrupedPcd.yaml"
        INTERFACE_CONFIG="${SUPER_LIO_CONFIG_DIR}/InterfaceQuadrupedPcd.yaml"
        RVIZ_CONFIG="${SUPER_LIO_CONFIG_DIR}/rviz/SuperLioRobot.rviz"
        OPERATION="mapping"
        MAP_ROOT="${SUPER_LIO_OUTPUT_ROOT}"
        ;;
    quadruped-relocation|relocation_dog_output|relocation)
        MODE="quadruped-relocation"
        RUNTIME_CONFIG="${SUPER_LIO_CONFIG_DIR}/SuperLioRobotQuadrupedRelocation.yaml"
        INTERFACE_CONFIG="${SUPER_LIO_CONFIG_DIR}/InterfaceQuadrupedRelocation.yaml"
        RVIZ_CONFIG="${SUPER_LIO_CONFIG_DIR}/rviz/SuperLioRobotRelocation.rviz"
        OPERATION="relocation"
        MAP_ROOT="${SUPER_LIO_CONFIG_DIR}"
        RELOCATION_MAP="${SUPER_LIO_CONFIG_DIR}/data/global_map_manual_opt.pcd"
        ;;
    *)
        echo "[ERROR] Unsupported mode: ${MODE}" >&2
        usage >&2
        exit 2
        ;;
esac

RVIZ_ENABLED="${SUPER_LIO_ROBOT_RVIZ:-true}"
codexopen_require_executable "${SUPER_LIO_BIN}"
codexopen_require_file "${RUNTIME_CONFIG}"
codexopen_require_file "${INTERFACE_CONFIG}"
if [[ "${RVIZ_ENABLED}" == "true" ]]; then
    codexopen_require_file "${RVIZ_CONFIG}"
fi
if [[ "${OPERATION}" == "relocation" ]]; then
    codexopen_require_file "${RELOCATION_MAP}"
fi

if [[ "${CHECK_ONLY}" == "true" ]]; then
    echo "[OK] super_lio_robot mode '${MODE}' executable and resources are complete"
    exit 0
fi

CODEXOPEN_LOG_DIR="${SUPER_LIO_ROBOT_LOG_DIR:-${CODEXOPEN_INSTALL_ROOT}/log/super_lio_robot/$(date '+%Y%m%d_%H%M%S')}"
export CODEXOPEN_LOG_DIR
codexopen_prepare_runtime
if [[ "${OPERATION}" == "mapping" ]]; then
    mkdir -p "${MAP_ROOT}/map"
fi
trap codexopen_cleanup EXIT INT TERM

if ! codexopen_start_master; then
    exit 1
fi

if [[ "${RVIZ_ENABLED}" == "true" ]]; then
    codexopen_start_optional_process super_lio_robot_rviz \
        rviz -d "${RVIZ_CONFIG}"
fi

codexopen_start_process super_lio_robot \
    "${SUPER_LIO_BIN}" \
    "${RUNTIME_CONFIG}" \
    "${INTERFACE_CONFIG}" \
    "${OPERATION}" \
    "${MAP_ROOT}"

echo "[READY] super_lio_robot is running in mode '${MODE}'"
if [[ "${OPERATION}" == "relocation" ]]; then
    echo "[INFO] Use RViz '2D Pose Estimate' or press Enter for YAML init_pose"
fi
echo "[INFO] Logs: ${CODEXOPEN_LOG_DIR}"
codexopen_monitor_processes
