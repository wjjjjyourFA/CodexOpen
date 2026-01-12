
使用 ROS1 自带的相关消息

Vector3Int ==> GaussPoint.msg  ==> 高斯坐标点

GeoPoint.msg  ==> 经纬度坐标点

为什么我要将这几个点和角分开呢？
因为我希望提供一个标准转换函数接口
将不同坐标系的坐标和姿态角进行转换，将高斯坐标点转换为经纬度坐标点，反之亦然。



### 本工程使用   EulerAngles 做为绕自身轴的表示    OrientationAngles 作为参考某个地图旋转的表示

### 📌 1. **EulerAngles（欧拉角）**

- 明确表示 **绕三个坐标轴的旋转角度**。

- 通常是按照某种固定顺序（如 Yaw-Pitch-Roll，Z-Y-X）进行的 **顺序旋转**。

- 通常格式是：

  ```
  text
  
  
  复制编辑
  roll (X轴旋转), pitch (Y轴旋转), yaw (Z轴旋转)
  ```

- 示例：

  ```
  cpp复制编辑struct EulerAngles {
      float roll;   // X axis
      float pitch;  // Y axis
      float yaw;    // Z axis
  };
  ```

------

### 📌 2. **OrientationAngles（方向角）**

- 更宽泛、**泛化的术语**，强调物体“面向的方向”，可能是用欧拉角表示，也可能是其他方式的角度。

- 在某些系统中，`OrientationAngles` 可能不一定完全等同于标准的 `EulerAngles`，可能会按某些传感器定义角度（例如 Android 中的 orientation angles 就不是严格的欧拉角顺序）。

- 有时包含磁北方向角（azimuth）、俯仰角（pitch）、翻滚角（roll），类似但顺序可能不同。

- 示例（以某些 IMU 为例）：

  ```
  cpp复制编辑struct OrientationAngles {
      float azimuth; // 航向角 / Yaw
      float pitch;
      float roll;
  };
  ```

------

### 📎 总结区别：

| 对比项           | `EulerAngles`                    | `OrientationAngles`                            |
| ---------------- | -------------------------------- | ---------------------------------------------- |
| 表达重点         | 数学定义的绕 XYZ 的旋转          | 更抽象，强调“朝向”，可能是欧拉角或其他角度形式 |
| 常见使用顺序     | roll-pitch-yaw（或其他欧拉顺序） | 有时是 azimuth-pitch-roll（如 Android）        |
| 语义             | 更倾向于数学或机械姿态描述       | 更偏向应用层语义，如方向/朝向                  |
| 是否一定是欧拉角 | ✅ 是                             | ❌ 不一定，可能是其他定义                       |
