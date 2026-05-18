#!/bin/bash
# 删除当前目录及子目录中的所有 *.pb.h *.pb.cc *_pb2.py 文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

# 确认当前目录是工程根目录，或者指定路径
BASE_DIR="${SCRIPT_DIR}/../.."

# 指定要删除的多个目录，相对于 BASE_DIR
TARGET_DIRS=("cyber" "modules")  # 修改为你需要的目录列表

for dir in "${TARGET_DIRS[@]}"; do
    FULL_PATH="$BASE_DIR/$dir"
    echo "Deleting all *.pb.h, *.pb.cc, and *_pb2.py files under $FULL_PATH ..."
    
    if [ -d "$FULL_PATH" ]; then
        find "$FULL_PATH" -type f \( -name "*.pb.h" -o -name "*.pb.cc" -o -name "*_pb2.py" \) -exec rm -v {} +
    else
        echo "Warning: directory $FULL_PATH does not exist, skipping."
    fi
done

echo "Done."
