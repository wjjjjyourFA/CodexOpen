### Version Updare Check List

**每当工程库版本进行一次大更新时，需要按下表顺序进行编译测试。**

- [x] modules/drviers/camera ：相机驱动
- [x] ros1/stable_msgs、ros2/stable_msgs：CodexOpen 定义的消息格式；服务于 国产`ARM`平台，旨在加速消息传输和 `OpenCV` 处理；
- [x] modules/drviers/gnss/hc_wrapper ：惯导驱动

- [x] tools/capture_record ：视频采集程序
- [x] tools/mmpose_inference_gui ：各种 PYTHON 推理程序的可视化界面；通过修改 `Run Script` + `rtsp` 实现；
- [x] tools/data_processor ：数据解析程序，用于从指定的数据接口中解析数据
- [x] tools/data_loader、tools/data_loader_ros1 ：数据加载程序，用于回放到指定的数据接口中

- [ ] modules/perception/common/fusion：
  - [x] lidar_camera_fusion_test、lidar_camera_fusion_realtime_ros1
  - [x] radar_camera_fusion_realtime_ros1
  - [ ] lidar_alignment_realtime：点云和点云之间的变换累积；动态补偿配合；降采样；
  - [ ] radar_lidar_alignment_realtime：毫米波点云到激光雷达点云的变换累积；点云添加新字段--速度

- [x] ros1/drivers、ros2/drivers：华测惯导驱动、速腾激光雷达驱动、德国大陆548毫米波驱动

- [x] modules/perception/tools/image_undistortion：用于数据库图像数据后处理时，批量去畸变
- [x] modules/perception/tools/sensor_calibration：
  - [x] camera_intrinsic/intrinsic_calib：一般相机的 k1、k2、p1、p2、k3
  - [x] camera_intrinsic/fisheye_calib：鱼眼相机的 k1、k2、k3、k4
    - [x] 调整 auto_image_picker：自动筛选采样图像
    - [x] calib_verification：传统法计算标定效果；LSD 算法；
  
---

- [ ] Hough 算法

- [ ] docker 涉及到 nvidia-container
  - [ ] modules/perception/camera_detection
  - [ ] modules/perception/camera_location
- [ ] modules/perception/meta_data_preprocess
- [ ] modules/perception/multi_sensor_tracking

**将所有的代码模块切换到 VSCODE 编译方式**

C++ 标准，默认设置为 14，兼容性支持 17。