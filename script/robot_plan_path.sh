#!/usr/bin/env bash

set -Euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export ROBOT_PLAN_FLOW_NAME="${ROBOT_PLAN_FLOW_NAME:-robot_plan_path}"
export ROBOT_PLAN_ENABLE_TERRAIN_EXPLORER="${ROBOT_PLAN_ENABLE_TERRAIN_EXPLORER:-false}"
export ROBOT_PLAN_ENABLE_WAYPOINT_PUBLISHER="${ROBOT_PLAN_ENABLE_WAYPOINT_PUBLISHER:-true}"

exec "${SCRIPT_DIR}/robot_plan_stack.sh" "$@"
