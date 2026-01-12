#!/usr/bin/bash
PWD=$(pwd)
# echo "[DEBUG] PWD = $PWD"

# 设置环境变量
LD_LIBRARY_PATH+=":/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu"

# ==> CodexOpen/install/lib
# 运行时，动态链接器查找 .so 动态库使用的是 LD_LIBRARY_PATH
LD_LIBRARY_PATH+=":$APOLLO_RUNTIME_PATH/lib"
LD_LIBRARY_PATH+=":$APOLLO_RUNTIME_PATH/lib/cyber"
# echo "[DEBUG] LD_LIBRARY_PATH = $LD_LIBRARY_PATH"

export CMAKE_PREFIX_PATH=$APOLLO_RUNTIME_PATH:$CMAKE_PREFIX_PATH
export PKG_CONFIG_PATH=$APOLLO_RUNTIME_PATH/lib/pkgconfig:$APOLLO_RUNTIME_PATH/share/pkgconfig:$PKG_CONFIG_PATH

# macOS 特殊处理
if [[ "$(uname -s)" == "Darwin" ]]; then
  export DYLD_LIBRARY_PATH=$APOLLO_RUNTIME_PATH/lib:$DYLD_LIBRARY_PATH
fi

# ==> CodexOpen/third_party/install/lib
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/googletest/lib"
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/glog/lib"
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/gflags/lib"

# LD_LIBRARY_PATH+=":$PWD/../third_party/install/abseil-cpp/lib"
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/ad_rss_lib/lib"
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/osqp/lib"
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/benchmark/lib"
# LD_LIBRARY_PATH+=":$PWD/../third_party/install/civetweb/lib"

LD_LIBRARY_PATH+=":$PWD/../third_party/install/Fast-CDR/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/foonathan_memory/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/Fast-DDS/lib"

LD_LIBRARY_PATH+=":$PWD/../third_party/install/protobuf/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/brpc/lib"

# LD_LIBRARY_PATH+=":$PWD/../third_party/install/opencv/lib"

LD_LIBRARY_PATH+=":$PWD/../third_party/install/gperftools/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/PROJ/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/json/lib"
LD_LIBRARY_PATH+=":$PWD/../third_party/install/tinyxml2/lib"

# LD_LIBRARY_PATH+=":$PWD/../third_party/install/libtorch_cpu/lib"

export LD_LIBRARY_PATH
# echo "[DEBUG] LD_LIBRARY_PATH = $LD_LIBRARY_PATH"

# Apollo / CyberRT 使用 tcmalloc 替代默认的内存分配器
if [ -z "$LD_PRELOAD" ]; then
  LD_PRELOAD="$PWD/../third_party/install/gperftools/lib/libtcmalloc_minimal.so"
else
  LD_PRELOAD+=":$PWD/../third_party/install/gperftools/lib/libtcmalloc_minimal.so"
fi

export LD_PRELOAD