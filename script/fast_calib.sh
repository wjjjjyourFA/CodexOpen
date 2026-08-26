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
Usage: fast_calib.sh [options]

Modes:
  --mode single            Run one-scene camera-LiDAR calibration (default)
  --mode multi             Fuse the last three center-record scenes
  --mode distance-filter   Export a bag cloud and select filter bounds

Common options:
  --bag PATH               Override Interface.yaml bag_path
  --image PATH             Override Interface.yaml image_path (single only)
  --output DIR             Override Interface.yaml output_path
  --topic TOPIC            Override Interface.yaml lidar_topic
  --rviz / --no-rviz       Enable or disable RViz
  --check                  Validate installed executables and configuration

Distance-filter option:
  --no-pick                Export PCD only; do not open Open3D
USAGE
}

MODE="${FAST_CALIB_MODE:-single}"
BAG_PATH=""
IMAGE_PATH=""
OUTPUT_PATH=""
LIDAR_TOPIC=""
RVIZ_REQUEST=""
CHECK_ONLY=false
NO_PICK=false

while (($# > 0)); do
    case "$1" in
        --mode)
            (($# >= 2)) || { echo "[ERROR] --mode requires a value" >&2; exit 2; }
            MODE="$2"
            shift 2
            ;;
        --bag)
            (($# >= 2)) || { echo "[ERROR] --bag requires a path" >&2; exit 2; }
            BAG_PATH="$2"
            shift 2
            ;;
        --image)
            (($# >= 2)) || { echo "[ERROR] --image requires a path" >&2; exit 2; }
            IMAGE_PATH="$2"
            shift 2
            ;;
        --output)
            (($# >= 2)) || { echo "[ERROR] --output requires a directory" >&2; exit 2; }
            OUTPUT_PATH="$2"
            shift 2
            ;;
        --topic)
            (($# >= 2)) || { echo "[ERROR] --topic requires a name" >&2; exit 2; }
            LIDAR_TOPIC="$2"
            shift 2
            ;;
        --rviz)
            RVIZ_REQUEST=true
            shift
            ;;
        --no-rviz)
            RVIZ_REQUEST=false
            shift
            ;;
        --no-pick)
            NO_PICK=true
            shift
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

case "${MODE}" in
    single|one|one-scene)
        MODE="single"
        ;;
    multi|multi-scene)
        MODE="multi"
        ;;
    distance-filter|filter)
        MODE="distance-filter"
        ;;
    *)
        echo "[ERROR] Unsupported mode: ${MODE}" >&2
        usage >&2
        exit 2
        ;;
esac

CONFIG_DIR="${CODEXOPEN_INSTALL_ROOT}/bin/config/FastCalib"
RUNTIME_CONFIG="${CONFIG_DIR}/FastCalib.yaml"
INTERFACE_CONFIG="${CONFIG_DIR}/Interface.yaml"
RVIZ_CONFIG="${CONFIG_DIR}/rviz/FastCalib.rviz"
SINGLE_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/tools/fast_calib/fast_calib_ros1"
MULTI_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/tools/fast_calib/multi_fast_calib_ros1"
FILTER_TOOL="${CODEXOPEN_INSTALL_ROOT}/bin/tools/fast_calib/distance_filter_tool.py"

codexopen_require_file "${RUNTIME_CONFIG}"
codexopen_require_file "${INTERFACE_CONFIG}"
case "${MODE}" in
    single)
        codexopen_require_executable "${SINGLE_BIN}"
        ;;
    multi)
        codexopen_require_executable "${MULTI_BIN}"
        ;;
    distance-filter)
        codexopen_require_executable "${FILTER_TOOL}"
        ;;
esac

if [[ -z "${RVIZ_REQUEST}" ]]; then
    if [[ -n "${FAST_CALIB_RVIZ:-}" ]]; then
        RVIZ_REQUEST="${FAST_CALIB_RVIZ}"
    elif [[ "${MODE}" == "single" ]]; then
        RVIZ_REQUEST=true
    else
        RVIZ_REQUEST=false
    fi
fi
if [[ "${RVIZ_REQUEST}" == "true" ]]; then
    codexopen_require_file "${RVIZ_CONFIG}"
fi

if [[ "${CHECK_ONLY}" == "true" ]]; then
    echo "[OK] FAST-Calib mode '${MODE}' executable and resources are complete"
    echo "[OK] Parameter source: ${RUNTIME_CONFIG}"
    echo "[OK] Interface source: ${INTERFACE_CONFIG}"
    exit 0
fi

if [[ "${MODE}" == "distance-filter" ]]; then
    if [[ -z "${BAG_PATH}" ]]; then
        echo "[ERROR] --bag is required in distance-filter mode" >&2
        exit 2
    fi
    if [[ -z "${OUTPUT_PATH}" ]]; then
        OUTPUT_PATH="${CODEXOPEN_PROJECT_ROOT}/tmp/fast_calib/filter"
    fi
    declare -a FILTER_COMMAND=(python3 "${FILTER_TOOL}" "${BAG_PATH}" "${OUTPUT_PATH}")
    if [[ -n "${LIDAR_TOPIC}" ]]; then
        FILTER_COMMAND+=(--topic "${LIDAR_TOPIC}")
    fi
    if [[ "${NO_PICK}" == "true" ]]; then
        FILTER_COMMAND+=(--no-pick)
    fi
    "${FILTER_COMMAND[@]}"
    exit $?
fi

CODEXOPEN_LOG_DIR="${FAST_CALIB_LOG_DIR:-${CODEXOPEN_INSTALL_ROOT}/log/fast_calib/$(date '+%Y%m%d_%H%M%S')}"
export CODEXOPEN_LOG_DIR
codexopen_prepare_runtime
trap codexopen_cleanup EXIT INT TERM
if ! codexopen_start_master; then
    exit 1
fi
if [[ "${RVIZ_REQUEST}" == "true" ]]; then
    codexopen_start_optional_process fast_calib_rviz \
        rviz -d "${RVIZ_CONFIG}"
fi

if [[ "${MODE}" == "multi" ]]; then
    declare -a MULTI_COMMAND=("${MULTI_BIN}" "${RUNTIME_CONFIG}" "${INTERFACE_CONFIG}")
    if [[ -n "${OUTPUT_PATH}" ]]; then
        MULTI_COMMAND+=(--output "${OUTPUT_PATH}")
    fi
    "${MULTI_COMMAND[@]}"
    exit $?
fi

declare -a SINGLE_COMMAND=("${SINGLE_BIN}" "${RUNTIME_CONFIG}" "${INTERFACE_CONFIG}")
if [[ -n "${BAG_PATH}" ]]; then
    SINGLE_COMMAND+=(--bag "${BAG_PATH}")
fi
if [[ -n "${IMAGE_PATH}" ]]; then
    SINGLE_COMMAND+=(--image "${IMAGE_PATH}")
fi
if [[ -n "${OUTPUT_PATH}" ]]; then
    SINGLE_COMMAND+=(--output "${OUTPUT_PATH}")
fi
if [[ -n "${LIDAR_TOPIC}" ]]; then
    SINGLE_COMMAND+=(--topic "${LIDAR_TOPIC}")
fi
codexopen_start_process fast_calib "${SINGLE_COMMAND[@]}"
echo "[READY] FAST-Calib single-scene mode is running"
echo "[INFO] Runtime configuration: ${RUNTIME_CONFIG}"
echo "[INFO] Interface configuration: ${INTERFACE_CONFIG}"
echo "[INFO] Logs: ${CODEXOPEN_LOG_DIR}"
codexopen_monitor_processes
