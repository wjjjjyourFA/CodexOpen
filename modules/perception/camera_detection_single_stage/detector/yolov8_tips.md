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

---

## pytorch2.6

```sh
Generating .wts for detect model
Loading ./yolov8s.pt
Traceback (most recent call last):
  File "/media/jojo/AQiDePan/CodexOpen/modules/perception/camera_detection_single_stage/detector/yolov8/gen_wts.py", line 40, in <module>
    model = torch.load(pt_file, map_location=device)  # Load FP32 weights
  File "/home/jojo/anaconda3/envs/yolo/lib/python3.10/site-packages/torch/serialization.py", line 1609, in load
    raise pickle.UnpicklingError(_get_wo_message(str(e))) from None
_pickle.UnpicklingError: Weights only load failed. This file can still be loaded, to do so you have two options, do those steps only if you trust the source of the checkpoint. 
	(1) In PyTorch 2.6, we changed the default value of the `weights_only` argument in `torch.load` from `False` to `True`. Re-running `torch.load` with `weights_only` set to `False` will likely succeed, but it can result in arbitrary code execution. Do it only if you got the file from a trusted source.
	(2) Alternatively, to load with `weights_only=True` please check the recommended steps in the following error message.
	WeightsUnpickler error: Unsupported global: GLOBAL ultralytics.nn.tasks.DetectionModel was not an allowed global by default. Please use `torch.serialization.add_safe_globals([ultralytics.nn.tasks.DetectionModel])` or the `torch.serialization.safe_globals([ultralytics.nn.tasks.DetectionModel])` context manager to allowlist this global if you trust this class/function.

Check the documentation of torch.load to learn more about types accepted by default with weights_only https://pytorch.org/docs/stable/generated/torch.load.html.
```

从 PyTorch 2.6 开始，`torch.load` 的 `weights_only` 参数默认值从 `False` 改为了 `True`（出于安全考虑，防止加载恶意代码）。

YOLOv8 的 `.pt` 文件里直接存储了整个模型的 Python 对象（`ultralytics.nn.tasks.DetectionModel`），而不单单是权重数据（State Dict），因此触发了 PyTorch 的安全拦截。

### 解决方案

解决这个问题最直接的方法有 **两种**，修改 `gen_wts.py` 的第 40 行即可：

#### 方案一：显示设置 `weights_only=False`（最简单快捷）

因为 `yolov8s.pt` 是你自己下载或训练的可信模型，直接关闭这个安全限制即可：
修改 `gen_wts.py` 第 40 行：
```Python
# 找到这一行：
# model = torch.load(pt_file, map_location=device)

# 修改为：
model = torch.load(pt_file, map_location=device, weights_only=False)
```

#### 方案二：将 YOLOv8 模型类加入安全白名单（更规范）

如果你想遵循 PyTorch 2.6 的安全规范，可以将 `DetectionModel` 添加到 PyTorch 的安全全局变量白名单中：
在 `gen_wts.py` 脚本顶部加入白名单设置：
```Python
import torch
import ultralytics.nn.tasks

# 添加 YOLOv8 的模型类到 safe_globals 白名单中
torch.serialization.add_safe_globals([ultralytics.nn.tasks.DetectionModel])

# 之后的第 40 行无需更改，或者显式写出 weights_only=True
model = torch.load(pt_file, map_location=device)
```

> **补充提示**：在基于 TensorRT 等 C++ 部署框架生成 `.wts` 文件时，通常建议优先确保本地已安装 `ultralytics` 库（`pip install ultralytics`），并在脚本开头 `import ultralytics`，这样修改后即可正常解析权重。
