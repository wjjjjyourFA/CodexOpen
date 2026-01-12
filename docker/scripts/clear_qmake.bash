#!/bin/bash
# 删除当前目录及子目录中的pro文件夹下的所有 *.pro.user* 文件, qmake 编译过程文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# 确认当前目录是工程根目录，或者指定路径
BASE_DIR="${SCRIPT_DIR}/../.."

# 指定要删除的多个目录，相对于 BASE_DIR
TARGET_DIRS=("modules" "tools")

for dir in "${TARGET_DIRS[@]}"; do
    FULL_PATH="$BASE_DIR/$dir"
    echo "Deleting all *.pro.user* files and Makefile under $FULL_PATH ..."

    # 只匹配 pro 文件夹下的 *.pro.user*
    find "${FULL_PATH}" \
        -type f \
        -path "*/pro/*.pro.user*" \
        -print -delete

    find "${FULL_PATH}" \
        -type f \
        -path "*/pro/*.pro.autosave*" \
        -print -delete

    # 删除 pro 文件夹下的 Makefile
    find "${FULL_PATH}" \
        -type f \
        -path "*/pro/Makefile" \
        -print -delete

    # 删除 qtc_clangd 文件夹
    find "${FULL_PATH}" \
        -type d \
        -path "*/.qtc_clangd" \
        -print -exec rm -rf {} +

    # 删除 qmake.stash 文件
    find "${FULL_PATH}" \
        -type f \
        -path "*/.qmake.stash" \
        -print -delete
done

echo "Done."
