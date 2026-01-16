std_msgs/Header header

uint16 crc
uint16 length
uint8 sqc

uint64 part_number
uint64 assembly_part_number
uint64 sw_part_number

uint8[26] serial_number
uint32 bl_version
uint32 bl_crc
uint32 sw_version
uint32 sw_crc

uint64 utc_time_stamp
uint32 time_stamp
uint32 current_damping

uint8 op_state
uint8 current_far_cf
uint8 current_near_cf
uint8 defective
uint8 suppl_volt_limit
uint8 sensor_off_temp
uint8 gm_missing

# uint8 tx_power_status
uint8 tx_out_reduced
uint8 internal_error

# float32 maximum_range_far
# float32 maximum_range_near