#!/usr/bin/env bash

set -e  # 遇到错误退出

CURRENT_PATH=$(cd $(dirname $0) && pwd)

sudo apt-get install -y libgeographic-dev

sudo apt-get install -y geographiclib-tools

# Add EGM2008 geoid grid to geographiclib
# sudo apt install geographiclib-geoids-2008

### online
# sudo geographiclib-get-geoids egm2008-1
### offline
cd $CURRENT_PATH
cd ..
cd third_party

GEIOD_DIR="/usr/share/GeographicLib/geoids"

# 1. 创建目录（如果不存在）
# sudo mkdir -p /usr/share/GeographicLib/geoids
sudo mkdir -p "$GEIOD_DIR"


# 2. 如果目标目录已经有 egm2008-1.*，跳过所有操作
GEIOD_PREFIX="egm2008-1"
# tar -xvjf egm2008-1.tar.bz2
# cd geoids
# sudo cp -r egm2008-1.*  /usr/share/GeographicLib/geoids/

if ls "$GEIOD_DIR"/${GEIOD_PREFIX}.* 1>/dev/null 2>&1; then
    echo "EGM2008 geoid files already exist in $GEIOD_DIR, skip extraction and copying."
    exit 0
fi

# 3. 解压（如果还没有）
echo "Extracting geoid data..."
tar -xvjf egm2008-1.tar.bz2

# 4. 进入 geoids 目录
cd geoids || exit 1

# 5. 执行复制
echo "Copying geoid files to $GEIOD_DIR..."
sudo cp -r ${GEIOD_PREFIX}.* "$GEIOD_DIR"

echo "Geoid installation complete."

# test
GeoidEval -n egm2008-1 --input-string "30 120"