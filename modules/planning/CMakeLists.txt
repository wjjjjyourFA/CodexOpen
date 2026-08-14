cmake_minimum_required(VERSION 3.16)

project(planning)

add_subdirectory(world_planner)
add_subdirectory(local_planner)
add_subdirectory(terrain_waypoint_exploration)
add_subdirectory(waypoint_publisher)
