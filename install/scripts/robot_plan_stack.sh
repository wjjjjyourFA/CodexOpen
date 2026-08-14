#!/usr/bin/env bash

set -Euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=runtime_common.sh
source "${SCRIPT_DIR}/runtime_common.sh"
codexopen_resolve_layout
codexopen_source_ros1

TERRAIN_ENABLED="${ROBOT_PLAN_ENABLE_TERRAIN_ANALYSIS:-true}"
ROG_MAP_ENABLED="${ROBOT_PLAN_ENABLE_ROG_MAP:-true}"
WORLD_ENABLED="${ROBOT_PLAN_ENABLE_WORLD_PLANNER:-true}"
LOCAL_ENABLED="${ROBOT_PLAN_ENABLE_LOCAL_PLANNER:-true}"
EXPLORER_ENABLED="${ROBOT_PLAN_ENABLE_TERRAIN_EXPLORER:-true}"
WAYPOINT_ENABLED="${ROBOT_PLAN_ENABLE_WAYPOINT_PUBLISHER:-false}"
STATIC_TF_ENABLED="${ROBOT_PLAN_ENABLE_STATIC_TF:-${LOCAL_ENABLED}}"
LOCAL_RVIZ_ENABLED="${ROBOT_PLAN_LOCAL_RVIZ:-true}"
ROG_MAP_RVIZ_ENABLED="${ROBOT_PLAN_ROG_MAP_RVIZ:-true}"

TERRAIN_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/perception/terrain_analysis/terrain_analysis_ros1"
ROG_MAP_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/perception/rog_map/rog_map_ros1"
WORLD_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/planning/world_planner/world_planner_ros1"
LOCAL_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/planning/local_planner/local_planner_ros1"
EXPLORER_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/planning/terrain_waypoint_exploration/terrain_waypoint_exploration_ros1"
WAYPOINT_BIN="${CODEXOPEN_INSTALL_ROOT}/bin/modules/planning/waypoint_publisher/waypoint_publisher_ros1"

TERRAIN_CONFIG_DIR="${ROBOT_PLAN_TERRAIN_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/TerrainAnalysis}"
ROG_MAP_CONFIG_DIR="${ROBOT_PLAN_ROG_MAP_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/RogMap}"
WORLD_CONFIG_DIR="${ROBOT_PLAN_WORLD_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/WorldPlanner}"
LOCAL_CONFIG_DIR="${ROBOT_PLAN_LOCAL_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/LocalPlanner}"
EXPLORER_CONFIG_DIR="${ROBOT_PLAN_EXPLORER_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/TerrainWaypointExploration}"
WAYPOINT_CONFIG_DIR="${ROBOT_PLAN_WAYPOINT_CONFIG_DIR:-${CODEXOPEN_INSTALL_ROOT}/bin/config/WaypointPublisher}"

TERRAIN_RUNTIME="${TERRAIN_CONFIG_DIR}/TerrainAnalysis.yaml"
TERRAIN_INTERFACE="${TERRAIN_CONFIG_DIR}/Interface.yaml"
ROG_MAP_RUNTIME="${ROG_MAP_CONFIG_DIR}/RogMap.yaml"
ROG_MAP_INTERFACE="${ROG_MAP_CONFIG_DIR}/Interface.yaml"
WORLD_RUNTIME="${WORLD_CONFIG_DIR}/WorldPlanner.yaml"
WORLD_INTERFACE="${WORLD_CONFIG_DIR}/Interface.yaml"
LOCAL_RUNTIME="${LOCAL_CONFIG_DIR}/LocalPlanner.yaml"
LOCAL_INTERFACE="${LOCAL_CONFIG_DIR}/Interface.yaml"
EXPLORER_RUNTIME="${EXPLORER_CONFIG_DIR}/TerrainWaypointExplorer.yaml"
EXPLORER_INTERFACE="${EXPLORER_CONFIG_DIR}/Interface.yaml"
WAYPOINT_RUNTIME="${WAYPOINT_CONFIG_DIR}/WaypointPublisher.yaml"
WAYPOINT_INTERFACE="${WAYPOINT_CONFIG_DIR}/Interface.yaml"

if [[ -n "${ROBOT_PLAN_WORLD_WAYPOINT_FILE:-}" ]]; then
    WORLD_WAYPOINT_FILE="${ROBOT_PLAN_WORLD_WAYPOINT_FILE}"
elif [[ "${WAYPOINT_ENABLED}" == "true" ]]; then
    WORLD_WAYPOINT_FILE="${WAYPOINT_CONFIG_DIR}/data/waypoint.txt"
else
    WORLD_WAYPOINT_FILE="${WORLD_CONFIG_DIR}/data/waypoint.txt"
fi

robot_plan_require_boolean()
{
    local name="$1"
    local value="$2"
    if [[ "${value}" != "true" && "${value}" != "false" ]]; then
        echo "[ERROR] ${name} must be true or false, got: ${value}" >&2
        return 1
    fi
}

for boolean_name in \
    TERRAIN_ENABLED ROG_MAP_ENABLED WORLD_ENABLED LOCAL_ENABLED \
    EXPLORER_ENABLED WAYPOINT_ENABLED STATIC_TF_ENABLED LOCAL_RVIZ_ENABLED \
    ROG_MAP_RVIZ_ENABLED; do
    robot_plan_require_boolean "${boolean_name}" "${!boolean_name}"
done

declare -a ENABLED_COMPONENTS=()

if [[ "${EXPLORER_ENABLED}" == "true" && "${WAYPOINT_ENABLED}" == "true" ]]; then
    echo "[ERROR] terrain_waypoint_exploration and waypoint_publisher both publish the waypoint/goal-valid interface; enable only one" >&2
    exit 1
fi

if [[ "${TERRAIN_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(terrain_analysis)
    codexopen_require_executable "${TERRAIN_BIN}"
    codexopen_require_file "${TERRAIN_RUNTIME}"
    codexopen_require_file "${TERRAIN_INTERFACE}"
fi

if [[ "${ROG_MAP_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(rog_map)
    codexopen_require_executable "${ROG_MAP_BIN}"
    codexopen_require_file "${ROG_MAP_RUNTIME}"
    codexopen_require_file "${ROG_MAP_INTERFACE}"
    if [[ "${ROG_MAP_RVIZ_ENABLED}" == "true" ]]; then
        codexopen_require_file "${ROG_MAP_CONFIG_DIR}/rviz/RogMap.rviz"
    fi
fi

if [[ "${WORLD_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(world_planner)
    codexopen_require_executable "${WORLD_BIN}"
    codexopen_require_file "${WORLD_RUNTIME}"
    codexopen_require_file "${WORLD_INTERFACE}"
    codexopen_require_file "${WORLD_WAYPOINT_FILE}"
fi

if [[ "${LOCAL_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(local_planner)
    codexopen_require_executable "${LOCAL_BIN}"
    codexopen_require_file "${LOCAL_RUNTIME}"
    codexopen_require_file "${LOCAL_INTERFACE}"
    codexopen_require_file "${LOCAL_CONFIG_DIR}/paths/correspondences.txt"
    if [[ "${LOCAL_RVIZ_ENABLED}" == "true" ]]; then
        codexopen_require_file "${LOCAL_CONFIG_DIR}/rviz/LocalPlanner.rviz"
    fi
fi

if [[ "${EXPLORER_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(terrain_waypoint_exploration)
    codexopen_require_executable "${EXPLORER_BIN}"
    codexopen_require_file "${EXPLORER_RUNTIME}"
    codexopen_require_file "${EXPLORER_INTERFACE}"
fi

if [[ "${WAYPOINT_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(waypoint_publisher)
    codexopen_require_executable "${WAYPOINT_BIN}"
    codexopen_require_file "${WAYPOINT_RUNTIME}"
    codexopen_require_file "${WAYPOINT_INTERFACE}"
    codexopen_require_file "${WAYPOINT_CONFIG_DIR}/data/waypoint.txt"
    codexopen_require_file "${WAYPOINT_CONFIG_DIR}/data/boundary.txt"
fi

if [[ "${STATIC_TF_ENABLED}" == "true" ]]; then
    ENABLED_COMPONENTS+=(static_tf)
    codexopen_require_file "${LOCAL_INTERFACE}"
fi

if [[ "${#ENABLED_COMPONENTS[@]}" -eq 0 ]]; then
    echo "[ERROR] No robot planning component is enabled" >&2
    exit 1
fi

if [[ "${STATIC_TF_ENABLED}" == "true" ]]; then
    read -r TF_X TF_Y TF_Z TF_ROLL TF_PITCH TF_YAW TF_PARENT TF_CHILD < <(
        python3 -c '
import sys, yaml
cfg = yaml.safe_load(open(sys.argv[1], encoding="utf-8"))["static_tf"]
print(cfg["x"], cfg["y"], cfg["z"], cfg["roll"], cfg["pitch"],
      cfg["yaw"], cfg["parent_frame"], cfg["child_frame"])
' "${LOCAL_INTERFACE}"
    )
fi

if [[ "${1:-}" == "--check" ]]; then
    echo "[OK] Enabled components are complete: ${ENABLED_COMPONENTS[*]}"
    exit 0
fi

ROBOT_PLAN_FLOW_NAME="${ROBOT_PLAN_FLOW_NAME:-robot_plan_stack}"
CODEXOPEN_LOG_DIR="${ROBOT_PLAN_LOG_DIR:-${CODEXOPEN_INSTALL_ROOT}/log/${ROBOT_PLAN_FLOW_NAME}/$(date '+%Y%m%d_%H%M%S')}"
export CODEXOPEN_LOG_DIR
codexopen_prepare_runtime
mkdir -p "${CODEXOPEN_INSTALL_ROOT}/pcd"
trap codexopen_cleanup EXIT INT TERM
if ! codexopen_start_master; then
    exit 1
fi

if [[ "${TERRAIN_ENABLED}" == "true" ]]; then
    codexopen_start_process terrain_analysis \
        "${TERRAIN_BIN}" "${TERRAIN_RUNTIME}" "${TERRAIN_INTERFACE}"
fi

if [[ "${WORLD_ENABLED}" == "true" ]]; then
    codexopen_start_process world_planner \
        "${WORLD_BIN}" "${WORLD_RUNTIME}" "${WORLD_INTERFACE}" \
        "${WORLD_WAYPOINT_FILE}"
fi

if [[ "${LOCAL_ENABLED}" == "true" ]]; then
    codexopen_start_process local_planner \
        "${LOCAL_BIN}" "${LOCAL_RUNTIME}" "${LOCAL_INTERFACE}" \
        "${LOCAL_CONFIG_DIR}/paths"
fi

if [[ "${STATIC_TF_ENABLED}" == "true" ]]; then
    codexopen_start_process static_tf rosrun tf2_ros static_transform_publisher \
        "${TF_X}" "${TF_Y}" "${TF_Z}" "${TF_ROLL}" "${TF_PITCH}" "${TF_YAW}" \
        "${TF_PARENT}" "${TF_CHILD}"
fi

if [[ "${ROG_MAP_ENABLED}" == "true" ]]; then
    codexopen_start_process rog_map \
        "${ROG_MAP_BIN}" "${ROG_MAP_RUNTIME}" "${ROG_MAP_INTERFACE}"
fi

if [[ "${EXPLORER_ENABLED}" == "true" ]]; then
    codexopen_start_process terrain_explorer \
        "${EXPLORER_BIN}" "${EXPLORER_RUNTIME}" "${EXPLORER_INTERFACE}"
fi

if [[ "${WAYPOINT_ENABLED}" == "true" ]]; then
    codexopen_start_process waypoint_publisher \
        "${WAYPOINT_BIN}" "${WAYPOINT_RUNTIME}" "${WAYPOINT_INTERFACE}" \
        "${WAYPOINT_CONFIG_DIR}/data/waypoint.txt" \
        "${WAYPOINT_CONFIG_DIR}/data/boundary.txt"
fi

if [[ "${LOCAL_ENABLED}" == "true" && "${LOCAL_RVIZ_ENABLED}" == "true" ]]; then
    codexopen_start_optional_process local_rviz nice rviz -d \
        "${LOCAL_CONFIG_DIR}/rviz/LocalPlanner.rviz"
fi
if [[ "${ROG_MAP_ENABLED}" == "true" && "${ROG_MAP_RVIZ_ENABLED}" == "true" ]]; then
    codexopen_start_optional_process rog_map_rviz rviz -d \
        "${ROG_MAP_CONFIG_DIR}/rviz/RogMap.rviz"
fi

echo "[READY] ${ROBOT_PLAN_FLOW_NAME} composition is running: ${ENABLED_COMPONENTS[*]}"
echo "[INFO] Logs: ${CODEXOPEN_LOG_DIR}"
codexopen_monitor_processes
