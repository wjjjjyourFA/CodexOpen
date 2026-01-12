            +------------------+
            | Planning Module  |
            +--------+---------+
                     |
                     v
           [chassis_command topic]
                     |
         +-----------+----------+
         |  Chassis Controller  |
         +-----------+----------+
                     |
                     v
         [读取底盘状态，发布 chassis_state]
                     |
            +--------+--------+
            | Other modules...|
            +-----------------+

**各同学只会需要关注 chassis_command topic 和 chassis_state topic**
  面向控制端实现底盘控制接口
**其他底盘模块由各自的工程师负责实现**


## 高层指令
1. ChassisAutoMode
**高层与底层解耦，避免底层控制过程中将高层指令初始化**

## 底层指令
1. ChassisCommand
2. ChassisState

## 可选指令
1. ChassisDetail



### 通用底盘控制方式：
1. 通过 chassis_command topic 控制底盘运动
2. 通过 chassis_state topic 获取底盘状态

### 不同底盘类型：
chassis_command chassis_state topic 的消息类型相同，

实际底盘控制，交由 **control_msgs** 实现
各类型底盘 采用顺序兼容 通过 chassis_type 控制
四轮阿克曼底盘   Ackermann4WD.msg
四轮差速底盘     Diff4WD.msg
四轮全向        Omni4WD.msg
履带式          Tracked.msg
自平衡单轮       SelfBalance.msg