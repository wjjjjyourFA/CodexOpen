在训练代码的工程中执行：（需要安装 ultralytics ）

20251223：jojo openmmlab 虚拟环境

```
cd ultralytics-8.3.233/ultralytics
// 将 gen_wts.py 复制到 ultralytics-8.3.233/ultralytics 下
python gen_wts.py -w ./yolov8s.pt -o ./yolov8s.wts -t detect
```

在转换部署的代码中执行：
```
./build/yolov8_det -s yolov8s.wts yolov8s.engine s

// INT8
cd ./build
./yolov8_det -s ./../yolov8s.wts ./../yolov8s-int8.engine s
```

```
./yolov8_det -d yolov8s.engine ../images g  // gpu postprocess
```



部分软链接创建：

```
cd yolov8/build
ln -s ./../../../../../../install/bin/data/CameraDetection/coco_calib ./coco_calib
```





```
./build/yolov8_det -s yolov8s.wts yolov8s.engine s
Loading weights: yolov8s.wts
[12/04/2025-04:00:21] [W] [TRT] The implicit batch dimension mode has been deprecated. Please create the network with NetworkDefinitionCreationFlag::kEXPLICIT_BATCH flag whenever possible.
Building engine, please wait for a while...
[12/04/2025-04:05:30] [W] [TRT] TensorRT encountered issues when converting weights between types and that could affect accuracy.
[12/04/2025-04:05:30] [W] [TRT] If this is not the desired behavior, please modify the weights or retrain with regularization to adjust the magnitude of the weights.
[12/04/2025-04:05:30] [W] [TRT] Check verbose logs for the list of affected weights.
[12/04/2025-04:05:30] [W] [TRT] - 59 weights are affected by this issue: Detected subnormal FP16 values.
Build engine successfully!
```

