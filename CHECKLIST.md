### Version Updare Check List

**每当工程库版本进行一次大更新时，需要按下表顺序进行编译测试。**

- [x] modules/drviers/camera ：相机驱动
- [x] ros1/stable_msgs
- [x] modules/drviers/gnss/hc_wrapper ：惯导驱动

- [x] capture_record ：视频采集程序

- [x] data_processor ：数据解析程序，用于从指定的数据接口中解析数据
- [x] data_loader ：数据加载程序，用于回放到指定的数据接口中

---

- [x] sensor_calibration/camera_intrinsic/intrinsic_calib
- [x] sensor_calibration/camera_intrinsic/fisheye_calib
  - [x] 调整 auto_image_picker ？
- [ ] LSD 算法
- [ ] Hough 算法
- [x] ros1/drivers
- [x] ros2/drivers
- [ ] 涉及到 nvidia-container
  - [ ] modules/perception/camera_detection
  - [ ] modules/perception/camera_location
- [ ] modules/perception/meta_data_preprocess
- [ ] modules/perception/multi_sensor_tracking

**将所有的代码模块切换到 VSCODE 编译方式**

C++ 标准，默认设置为 14，兼容性支持 17。