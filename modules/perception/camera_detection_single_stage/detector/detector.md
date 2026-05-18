## C++  TRT YOLO 

代码源自 [wang-xinyu/tensorrtx](https://github.com/wang-xinyu/tensorrtx)



### 环境准备：

ROS1 | CUDA 11.7 | TensorRT-8.5.3.1



### 相关参数修改：类别 数量 置信度

**default**: 

```
const static int kBatchSize = 1;
```

**yolov5/src/config.h**

```
// Detection model and Segmentation model' number of classes
// constexpr static int kNumClass = 80;
constexpr static int kNumClass = 8;

// NMS overlapping thresh and final detection confidence thresh
const static float kNmsThresh = 0.45f;
const static float kConfThresh = 0.25f;

```

**yolov8/include/config.h**

```
const static int kNumClass = 5;
const static int kNumberOfPoints = 17;  // number of keypoints total

const static float kNmsThresh = 0.45f;
const static float kConfThresh = 0.5f;
const static float kConfThreshKeypoints = 0.5f;  // keypoints confidence
```

**CMakeLists.txt**

```
# TODO(Call for PR): make TRT path configurable from command line
# include_directories(/home/nvidia/TensorRT-8.2.5.1/include/)
# link_directories(/home/nvidia/TensorRT-8.2.5.1/lib/)
include_directories(/opt/TensorRT/include/)
link_directories(/opt/TensorRT/lib/)
```



**20250903** : yolov8 使用的是旧版 yolov8 程序，并不是最新的。注意使用。





