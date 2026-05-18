在训练代码的工程中执行：（相关的软件包在 requirements 中，提前安装）

20251223：jojo openmmlab 虚拟环境

```
cd yolov5-7.0
// 将 gen_wts.py 复制到 yolov5-7.0 下
python gen_wts.py -w ./yolov5s.pt -o ./yolov5s.wts -t detect
```

在转换部署的代码中执行：
```
./build/yolov5_det -s yolov5s.wts yolov5s.engine s

// INT8
cd ./build
./yolov5_det -s ./../yolov5s.wts ./../yolov5s-int8.engine s
```

```
./yolov5_det -d yolov5s.engine ../images g  // gpu postprocess
```



部分软链接创建：

```
cd yolov5
ln -s ./../yolov3-spp/samples ./images

cd yolov5/build
ln -s ./../../../../../../install/bin/data/CameraDetection/coco_calib ./coco_calib
```





```
./build/yolov5_det -s yolov5s.wts yolov5s.engine s
[12/04/2025-03:57:41] [W] [TRT] The implicit batch dimension mode has been deprecated. Please create the network with NetworkDefinitionCreationFlag::kEXPLICIT_BATCH flag whenever possible.
Loading weights: yolov5s.wts
Building engine, please wait for a while...
[12/04/2025-04:01:32] [W] [TRT] TensorRT encountered issues when converting weights between types and that could affect accuracy.
[12/04/2025-04:01:32] [W] [TRT] If this is not the desired behavior, please modify the weights or retrain with regularization to adjust the magnitude of the weights.
[12/04/2025-04:01:32] [W] [TRT] Check verbose logs for the list of affected weights.
[12/04/2025-04:01:32] [W] [TRT] - 52 weights are affected by this issue: Detected subnormal FP16 values.
[12/04/2025-04:01:32] [W] [TRT] - 2 weights are affected by this issue: Detected values less than smallest positive FP16 subnormal value and converted them to the FP16 minimum subnormalized value.
Build engine successfully!
```

