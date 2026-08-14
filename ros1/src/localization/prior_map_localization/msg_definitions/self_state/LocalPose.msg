
float64 local_time		# in ms
float64 UTC_time		# in ms
int32   message_num

float64    dr_x		# in m  x vehicle front	
float64    dr_y		# in m  y vehicle left	
float64    dr_z		# in m  z vehicle up	
float64    dr_roll		# in degree  rolate along x axis, y -> z right-hand
float64    dr_pitch		# in degree  rolate along y axis, z -> x right-hand
float64    dr_heading		# in degree  rolate along z axis, x -> y right-hand

float64    vehicle_speed       # in m/s speed of the vehicle's reference point, e.g. for Ackerman-steered vehicle, it is the central speed of rear axes. 
float64    speed_x             # in m/s  speed along x-axis 
float64    speed_y             # in m/s  speed along y-axis
float64    speed_z             # in m/s  speed along z-axis
 
int8       STATUS_PARK = 0
int8       STATUS_FORWARD = 1
int8       STATUS_BACKWARD = -1  
int8       driving_direction   # 0:Park, 1:forward, -1: reverse
float64[4] reserved
