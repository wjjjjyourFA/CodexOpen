
float64 local_time     # in ms
float64 UTC_time	# in ms
int32   message_num	

int32   pos_type       # 0:invalid,  1:map matching, 2:lidar global odometry

float64 x              #    in m  
float64 y              #    in m  
float64 z              #    in m 

float64 x_speed        #    in m/s 
float64 y_speed        #    in m/s 
float64 z_speed        #    in m/s 

float64 azimuth        #    in degree  rolate along x axis, y -> z right-hand
float64 pitch          #    in degree  rolate along y axis, z -> x right-hand
float64 roll           #    in degree  rolate along z axis, x -> y right-hand

LocalPose local_pose
