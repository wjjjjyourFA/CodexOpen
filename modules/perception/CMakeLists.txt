# 隐式调用 cmake_policy(VERSION X.Y) → 重置所有 policy
cmake_minimum_required(VERSION 3.16)

project(perception)

add_subdirectory(tools)
add_subdirectory(common)
