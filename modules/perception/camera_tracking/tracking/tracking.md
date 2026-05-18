## C++  TRT deep_sort

代码源自 [RichardoMrMu/deepsort-tensorrt](https://github.com/RichardoMrMu/deepsort-tensorrt)

jojo 进行了一些 bug 修复，适配 TRT8.5.3.1，新增对特征维度的自定义：128, 256, 512等



### 环境准备：

ROS1 | CUDA 11.7 | TensorRT-8.5.3.1



## C++  TRT ByteTrack

代码源自 [FoundationVision/ByteTrack/deploy/TensorRT/cpp](https://github.com/FoundationVision/ByteTrack/tree/main/deploy/TensorRT/cpp)

jojo 进行了 CodexOpen 的工程化适配（整体上与原文件基本一致），包括对 using namespace std 等的规范化。

bytetrack 的 main 入口，在 bytetrack.cpp 中。原档中个平台的不同模式，大部分更改发生在该 bytetrack.cpp。
