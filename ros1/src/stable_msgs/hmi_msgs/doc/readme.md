### 任务控制
├── ButtonEvent.msg
├── TouchEvent.msg
├── VoiceEvent.msg
├── TaskCommand.msg
├── TaskStatus.msg

#### 人机交互流程
```
+-------------------+
|   ButtonEvent     |  Voice  Touch
|   (Button Pressed)|
+-------------------+
         |
         v
+-------------------+
|   TaskCommand     |
|   (e.g., task_id = 1234)|
+-------------------+
         |
         v
+-------------------+    +-------------------+
|   TaskStatus     |    |   TaskStatus      |
|   (status: 2,    |    |   (status: 3,     |
|   success)       |    |   failure message)|
+-------------------+    +-------------------+
         |
         v
+-------------------+
|   Feedback TaskStatus |
|   (status: COMPLETED) |
+-------------------+
```



### UI显示
├── TaskStatus.msg
├── SystemStatus.msg
├── MissionList.msg
├── MissionResult.msg
├── Notification.msg
├── FaultInfo.msg



### 人工遥控
├── VehicleRemoteControl.msg
├── BodyModuleControl.msg
├── TaskPoint.msg
├── TaskPointControl.msg


TaskPoint 使用 经纬度 地图，注意坐标系转换
  尤其是 orient_angle 
  与 gauss 坐标系的区别