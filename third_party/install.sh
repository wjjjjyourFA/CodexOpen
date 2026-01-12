#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

echo "******************  ******************"

cd ./CodexOpen
ln -sfn ./../../cyber ./cyber

mkdir -p ./modules && cd ./modules
ln -sfn ./../../../modules/common ./common
ln -sfn ./../../../modules/common_struct ./common_struct