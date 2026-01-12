std_msgs/Header header
uint32 message_num

# 这里填写 底盘详细状态信息，该消息面向底盘模块
# Vehicle status flags
uint32[] sta_code   # 底层控制器状态码 是否存在底盘故障
uint32[] err_code   # 底层控制器错误码 错误信息说明（可选）

# LightStatus
uint8 left_signal_light     # 左转向灯 left-turn light: 0-OFF, 1-ON, 2-BLINK
uint8 right_signal_light    # 右转向灯 right-turn light: 0-OFF, 1-ON, 2-BLINK
uint8 headlight             # 前大灯 front light: 0-OFF, 1-ON, 2-BLINK
uint8 brake_light           # 刹车灯 break light: 0-OFF, 1-ON, 2-BLINK 
uint8 postion_light         # 示宽灯 postion light: 0-OFF, 1-ON, 2-BLINK 
uint8 high_beam             # 远光灯 high neam: 0-OFF, 1-ON, 2-BLINK 
uint8 low_beam              # 近光灯 high neam: 0-OFF, 1-ON, 2-BLINK 
uint8 front_fog_light       # 前雾灯 front_fog_light: 0-OFF, 1-ON, 2-BLINK 
uint8 rear_fog_light        # 后雾灯 front_fog_light: 0-OFF, 1-ON, 2-BLINK 

uint8 horn         # 喇叭 0-OFF, 1-ON, 2-BLINK

uint8 fuel_level   # 燃油百分比 (0~100)，单位: %
uint8 battery_soc  # 电池电量状态 (State of Charge)(0~100)，单位: %