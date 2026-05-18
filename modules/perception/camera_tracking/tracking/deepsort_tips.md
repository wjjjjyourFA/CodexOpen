在训练代码的工程中执行：（相关的软件包在 requirements 中，提前安装）

20251223：jojo openmmlab 虚拟环境

```
cd deep_sort_pytorch
```
`deep_sort_pytorch/configs/deep_sort.yaml` 添加 USE_FASTREID：false

```
DEEPSORT:
  REID_CKPT: "./deep_sort/deep/checkpoint/ckpt.t7"
  MAX_DIST: 0.2
  MIN_CONFIDENCE: 0.5
  NMS_MAX_OVERLAP: 0.5
  MAX_IOU_DISTANCE: 0.7
  MAX_AGE: 70
  N_INIT: 3
  NN_BUDGET: 100
USE_FASTREID: false
```

```
// 用 CPU 导出 ONNX
python exportOnnx.py --config_deepsort ./configs/deep_sort.yaml --cpu
// input weights
deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7
// output onnx
deep_sort_pytorch/deepsort.onnx
```

在转换部署的代码中执行：
```
./build/onnx2engine ./deepsort.onnx ./resources/deepsort.engine
```


#### test:

```
// 模型路径写绝对路径
// ./build/demo ./resource/deepsort.engine ./resources/track.txt
./build/demo /PATH/deepsort/resources/deepsort.engine /PATH/deepsort/resources/track.txt
```





```
./build/onnx2engine ./deepsort.onnx ./resources/deepsort.engine
[12/23/2025-07:51:40] [W] [TRT] onnx2trt_utils.cpp:377: Your ONNX model has been generated with INT64 weights, while TensorRT does not natively support INT64. Attempting to cast down to INT32.
[12/23/2025-07:53:30] [W] [TRT] TensorRT encountered issues when converting weights between types and that could affect accuracy.
[12/23/2025-07:53:30] [W] [TRT] If this is not the desired behavior, please modify the weights or retrain with regularization to adjust the magnitude of the weights.
[12/23/2025-07:53:30] [W] [TRT] Check verbose logs for the list of affected weights.
[12/23/2025-07:53:30] [W] [TRT] - 24 weights are affected by this issue: Detected subnormal FP16 values.
[12/23/2025-07:53:30] [W] [TRT] - 17 weights are affected by this issue: Detected values less than smallest positive FP16 subnormal value and converted them to the FP16 minimum subnormalized value.
[12/23/2025-07:53:31] [E] [TRT] 3: [builder.cpp::~Builder::307] Error Code 3: API Usage Error (Parameter check failed at: optimizer/api/builder.cpp::~Builder::307, condition: mObjectCounter.use_count() == 1. Destroying a builder object before destroying objects it created leads to undefined behavior.
)
==============
|  SUCCESS!  |
==============
```

