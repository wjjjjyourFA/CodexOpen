std_msgs/Header header

float32 longitudinal
float32 lateral
float32 vertical
float32 yaw
float32 pitch
uint8 plug_orientation
float32 length
float32 width
float32 height
float32 wheelbase
uint16 maximum_distance
uint8 frequency_slot
uint8 cycle_time
uint8 time_slot
uint8 hcc
uint8 powersave_standstill
uint32 sensor_ip_address_0
uint32 sensor_ip_address_1

uint8 new_sensor_mounting
uint8 new_vehicle_parameters
uint8 new_radar_parameters
uint8 new_network_configuration

# 注意：雷达存储器是支持掉电存储的Flash，类似U盘有擦写寿命，
# 所以切记一秒钟配置一次甚至多次这样的无用、有害配置操作，
# 而建议只在需要时根据读取的雷达主动输出的状态消息验证修改某个参数时才配置新参数。
# 配置成功后，状态消息里的Configuration counter字段数值会+1（刚上电时为0），
# 可以以此判断配置是否有被雷达接收到和接受并存储。