每个程序自己生成 `config` 文件，自己管理自己的配置文件

包括 .ini .engine .onnx 

该文件夹下的代码，主代码应与 ROS1 ROS2 DDS RCS 等无关。

通过 `ros1_convert`\ `ros2_convert`\ `dds_convert` 等进行合作分发通信

并借助于 `"cyber/common/types_extra.h"` 中的 `#define ENABLE_ROS1` 等进行控制
