#include "ars548_base_node.h"

RadarBaseNode::RadarBaseNode() {}

RadarBaseNode::~RadarBaseNode() {}

void RadarBaseNode::SetUdpIo(std::shared_ptr<UdpInterface> udp_io) {
  udp_io_ = udp_io;
}

void RadarBaseNode::publishObjectList(const RadarObjectList& list) {
  ars548_process::Object obj;
  ars548_process::ObjectList obj_list;
  int i;
  obj_list.objects.clear();

  obj_list.service_id                 = list.serviceID;
  obj_list.method_id                  = list.MethodID;
  obj_list.data_length                = list.data_length;
  obj_list.client_id                  = list.clientID;
  obj_list.session_id                 = list.sessionID;
  obj_list.protocol_version           = list.protocol_version;
  obj_list.interface_version          = list.interface_version;
  obj_list.message_type               = list.message_type;
  obj_list.return_code                = list.return_code;
  obj_list.crc                        = list.CRC;
  obj_list.length                     = list.Length;
  obj_list.sqc                        = list.SQC;
  obj_list.data_id                    = list.DataID;
  obj_list.timestamp_nanoseconds      = list.Timestamp_Nanoseconds;
  obj_list.timestamp_seconds          = list.Timestamp_Seconds;
  obj_list.timestamp_sync_status      = list.Timestamp_SyncStatus;
  obj_list.event_data_qualifier       = list.EventDataQualifier;
  obj_list.extended_qualifier         = list.ExtendedQualifier;
  obj_list.object_list_num_of_objects = list.ObjectList_NumOfObjects;

  for (i = 0; i < list.ObjectList_NumOfObjects; i++) {
    obj.header.frame_id = param_->frame_id;
    obj.header.stamp    = ros::Time::now();

    // clang-format off
    obj.u_status_sensor            = list.Objects[i].u_StatusSensor;
    obj.u_id                       = list.Objects[i].u_ID;
    obj.u_age                      = list.Objects[i].u_Age;
    obj.u_status_measurement       = list.Objects[i].u_StatusMeasurement;
    obj.u_status_movement          = list.Objects[i].u_StatusMovement;
    obj.u_position_invalid_flags   = list.Objects[i].u_Position_InvalidFlags;
    obj.u_position_reference       = list.Objects[i].u_Position_Reference;
    obj.u_position_x               = list.Objects[i].u_Position_X;
    obj.u_position_x_std           = list.Objects[i].u_Position_X_STD;
    obj.u_position_y               = list.Objects[i].u_Position_Y;
    obj.u_position_y_std           = list.Objects[i].u_Position_Y_STD;
    obj.u_position_z               = list.Objects[i].u_Position_Z;
    obj.u_position_z_std           = list.Objects[i].u_Position_Z_STD;
    obj.u_position_covariance_xy   = list.Objects[i].u_Position_CovarianceXY;
    obj.u_position_orientation     = list.Objects[i].u_Position_Orientation;
    obj.u_position_orientation_std = list.Objects[i].u_Position_Orientation_STD;
    obj.u_existence_invalid_flags  = list.Objects[i].u_Existence_InvalidFlags;
    obj.u_existence_probability    = list.Objects[i].u_Existence_Probability;
    obj.u_existence_ppv            = list.Objects[i].u_Existence_PPV;
    obj.u_classification_car       = list.Objects[i].u_Classification_Car;
    obj.u_classification_truck     = list.Objects[i].u_Classification_Truck;
    obj.u_classification_motorcycle = list.Objects[i].u_Classification_Motorcycle;
    obj.u_classification_bicycle = list.Objects[i].u_Classification_Bicycle;
    obj.u_classification_pedestrian = list.Objects[i].u_Classification_Pedestrian;
    obj.u_classification_animal  = list.Objects[i].u_Classification_Animal;
    obj.u_classification_hazard  = list.Objects[i].u_Classification_Hazard;
    obj.u_classification_unknown = list.Objects[i].u_Classification_Unknown;
    obj.u_classification_overdrivable = list.Objects[i].u_Classification_Overdrivable;
    obj.u_classification_underdrivable = list.Objects[i].u_Classification_Underdrivable;
    obj.u_dynamics_abs_vel_invalid_flags = list.Objects[i].u_Dynamics_AbsVel_InvalidFlags;
    obj.f_dynamics_abs_vel_x     = list.Objects[i].f_Dynamics_AbsVel_X;
    obj.f_dynamics_abs_vel_x_std = list.Objects[i].f_Dynamics_AbsVel_X_STD;
    obj.f_dynamics_abs_vel_y     = list.Objects[i].f_Dynamics_AbsVel_Y;
    obj.f_dynamics_abs_vel_y_std = list.Objects[i].f_Dynamics_AbsVel_Y_STD;
    obj.f_dynamics_abs_vel_covariance_xy = list.Objects[i].f_Dynamics_AbsVel_CovarianceXY;
    obj.u_dynamics_rel_vel_invalid_flags = list.Objects[i].u_Dynamics_RelVel_InvalidFlags;
    obj.f_dynamics_rel_vel_x     = list.Objects[i].f_Dynamics_RelVel_X;
    obj.f_dynamics_rel_vel_x_std = list.Objects[i].f_Dynamics_RelVel_X_STD;
    obj.f_dynamics_rel_vel_y     = list.Objects[i].f_Dynamics_RelVel_Y;
    obj.f_dynamics_rel_vel_y_std = list.Objects[i].f_Dynamics_RelVel_Y_STD;
    obj.f_dynamics_rel_vel_covariance_xy = list.Objects[i].f_Dynamics_RelVel_CovarianceXY;
    obj.u_dynamics_abs_accel_invalid_flags = list.Objects[i].u_Dynamics_AbsAccel_InvalidFlags;
    obj.f_dynamics_abs_accel_x     = list.Objects[i].f_Dynamics_AbsAccel_X;
    obj.f_dynamics_abs_accel_x_std = list.Objects[i].f_Dynamics_AbsAccel_X_STD;
    obj.f_dynamics_abs_accel_y     = list.Objects[i].f_Dynamics_AbsAccel_Y;
    obj.f_dynamics_abs_accel_y_std = list.Objects[i].f_Dynamics_AbsAccel_Y_STD;
    obj.f_dynamics_abs_accel_covariance_xy = list.Objects[i].f_Dynamics_AbsAccel_CovarianceXY;
    obj.u_dynamics_rel_accel_invalid_flags = list.Objects[i].u_Dynamics_RelAccel_InvalidFlags;
    obj.f_dynamics_rel_accel_x     = list.Objects[i].f_Dynamics_RelAccel_X;
    obj.f_dynamics_rel_accel_x_std = list.Objects[i].f_Dynamics_RelAccel_X_STD;
    obj.f_dynamics_rel_accel_y     = list.Objects[i].f_Dynamics_RelAccel_Y;
    obj.f_dynamics_rel_accel_y_std = list.Objects[i].f_Dynamics_RelAccel_Y_STD;
    obj.f_dynamics_rel_accel_covariance_xy = list.Objects[i].f_Dynamics_RelAccel_CovarianceXY;
    obj.u_dynamics_orientation_invalid_flags = list.Objects[i].u_Dynamics_Orientation_InvalidFlags;
    obj.u_dynamics_orientation_rate_mean = list.Objects[i].u_Dynamics_Orientation_Rate_Mean;
    obj.u_dynamics_orientation_rate_std = list.Objects[i].u_Dynamics_Orientation_Rate_STD;
    obj.u_shape_length_status = list.Objects[i].u_Shape_Length_Status;
    obj.u_shape_length_edge_invalid_flags = list.Objects[i].u_Shape_Length_Edge_InvalidFlags;
    obj.u_shape_length_edge_mean = list.Objects[i].u_Shape_Length_Edge_Mean;
    obj.u_shape_length_edge_std  = list.Objects[i].u_Shape_Length_Edge_STD;
    obj.u_shape_width_status     = list.Objects[i].u_Shape_Width_Status;
    obj.u_shape_width_edge_invalid_flags = list.Objects[i].u_Shape_Width_Edge_InvalidFlags;
    obj.u_shape_width_edge_mean = list.Objects[i].u_Shape_Width_Edge_Mean;
    obj.u_shape_width_edge_std  = list.Objects[i].u_Shape_Width_Edge_STD;

    obj_list.objects.push_back(obj);
  }
  // clang-format on

  objects_pub.publish(obj_list);
}

void RadarBaseNode::publishDetectionList(const RadarDetectionList& list) {
  ars548_process::Detection det;
  ars548_process::DetectionList det_list;
  int i;
  det_list.detections.clear();

  det_list.service_id               = list.serviceID;
  det_list.method_id                = list.MethodID;
  det_list.data_length              = list.data_length;
  det_list.client_id                = list.clientID;
  det_list.session_id               = list.sessionID;
  det_list.protocol_version         = list.protocol_version;
  det_list.interface_version        = list.interface_version;
  det_list.message_type             = list.message_type;
  det_list.return_code              = list.return_code;
  det_list.crc                      = list.CRC;
  det_list.length                   = list.Length;
  det_list.sqc                      = list.SQC;
  det_list.data_id                  = list.DataID;
  det_list.timestamp_nanoseconds    = list.Timestamp_Nanoseconds;
  det_list.timestamp_seconds        = list.Timestamp_Seconds;
  det_list.timestamp_sync_status    = list.Timestamp_SyncStatus;
  det_list.event_data_qualifier     = list.EventDataQualifier;
  det_list.extended_qualifier       = list.ExtendedQualifier;
  det_list.origin_invalid_flags     = list.Origin_InvalidFlags;
  det_list.origin_pos_x             = list.Origin_Xpos;
  det_list.origin_x_std             = list.Origin_Xstd;
  det_list.origin_pos_y             = list.Origin_Ypos;
  det_list.origin_y_std             = list.Origin_Ystd;
  det_list.origin_pos_z             = list.Origin_Zpos;
  det_list.origin_z_std             = list.Origin_Zstd;
  det_list.origin_roll              = list.Origin_Roll;
  det_list.origin_roll_std          = list.Origin_Rollstd;
  det_list.origin_pitch             = list.Origin_Pitch;
  det_list.origin_pitch_std         = list.Origin_Pitchstd;
  det_list.origin_yaw               = list.Origin_Yaw;
  det_list.origin_yaw_std           = list.Origin_Yawstd;
  det_list.list_invalid_flags       = list.List_InvalidFlags;
  det_list.list_rad_vel_domain_min  = list.List_RadVelDomain_Min;
  det_list.list_rad_vel_domain_max  = list.List_RadVelDomain_Max;
  det_list.list_num_of_detections   = list.List_NumOfDetections;
  det_list.aln_azimuth_correction   = list.Aln_AzimuthCorrection;
  det_list.aln_elevation_correction = list.Aln_ElevationCorrection;
  det_list.aln_status               = list.Aln_Status;

  // clang-format off
  for (i = 0; i < (int)(list.List_NumOfDetections); i++) {
    det.header.frame_id = param_->frame_id;
    det.header.stamp = ros::Time::now();

    det.f_x = list.Detections[i].f_x;
    det.f_y = list.Detections[i].f_y;
    det.f_z = list.Detections[i].f_z;

    det.f_azimuth_angle = list.Detections[i].f_AzimuthAngle;
    det.f_azimuth_angle_std = list.Detections[i].f_AzimuthAngleSTD;
    
    det.u_invalid_flags  = list.Detections[i].u_InvalidFlags;
    
    det.f_elevation_angle = list.Detections[i].f_ElevationAngle;
    det.f_elevation_angle_std = list.Detections[i].f_ElevationAngleSTD;
    det.f_range = list.Detections[i].f_Range;
    det.f_range_std = list.Detections[i].f_RangeSTD;
    
    det.f_range_rate     = list.Detections[i].f_RangeRate;
    det.f_range_rate_std = list.Detections[i].f_RangeRateSTD;
    det.s_rcs            = list.Detections[i].s_RCS;
    det.u_measurement_id = list.Detections[i].u_MeasurementID;
    det.u_positive_predictive_value = list.Detections[i].u_PositivePredictiveValue;
    det.u_classification = list.Detections[i].u_Classification;
    det.u_multi_target_probability = list.Detections[i].u_MultiTargetProbability;
    det.u_object_id      = list.Detections[i].u_ObjectID;
    det.u_ambiguity_flag = list.Detections[i].u_AmbiguityFlag;
    det.u_sort_index     = list.Detections[i].u_SortIndex;

    det_list.detections.push_back(det);
  }
  // clang-format on

  detections_pub.publish(det_list);
}

void RadarBaseNode::publishRadarStatus(const RadarStatus& status) {
  ars548_process::SensorStatus sts;

  sts.header.frame_id = param_->frame_id;
  sts.header.stamp    = ros::Time::now();

  // clang-format off
  sts.timestamp_nanoseconds            = status.Timestamp_Nanoseconds;
  sts.timestamp_seconds                = status.Timestamp_Seconds;
  sts.timestamp_sync_status            = status.Timestamp_SyncStatus;
  sts.sw_version_major                 = status.SWVersion_Major;
  sts.sw_version_minor                 = status.SWVersion_Minor;
  sts.sw_version_patch                 = status.SWVersion_Patch;
  
  sts.longitudinal                     = status.BasicStatus.Longitudinal;
  sts.lateral                          = status.BasicStatus.Lateral;
  sts.vertical                         = status.BasicStatus.Vertical;
  sts.yaw                              = status.BasicStatus.Yaw;
  sts.pitch                            = status.BasicStatus.Pitch;
  sts.plug_orientation                 = status.BasicStatus.PlugOrientation;
  sts.length                           = status.BasicStatus.Length;
  sts.width                            = status.BasicStatus.Width;
  sts.height                           = status.BasicStatus.Height;
  sts.wheelbase                        = status.BasicStatus.Wheelbase;
  sts.maximum_distance                 = status.BasicStatus.MaximumDistance;
  sts.frequency_slot                   = status.BasicStatus.FrequencySlot;
  sts.cycle_time                       = status.BasicStatus.CycleTime;
  sts.time_slot                        = status.BasicStatus.TimeSlot;
  sts.hcc                              = status.BasicStatus.HCC;
  sts.powersave_standstill             = status.BasicStatus.Powersave_Standstill;
  sts.sensor_ip_address_0              = status.BasicStatus.SensorIPAddress_0;
  sts.sensor_ip_address_1              = status.BasicStatus.SensorIPAddress_1;
  
  sts.configuration_counter            = status.Configuration_counter;
  sts.status_longitudinal_velocity     = status.Status_LongitudinalVelocity;
  sts.status_longitudinal_acceleration = status.Status_LongitudinalAcceleration;
  sts.status_lateral_acceleration      = status.Status_LateralAcceleration;
  sts.status_yaw_rate                  = status.Status_YawRate;
  sts.status_steering_angle            = status.Status_SteeringAngle;
  sts.status_driving_direction         = status.Status_DrivingDirection;
  sts.status_characteristic_speed      = status.Status_CharacteristicSpeed;
  sts.status_radar_status              = status.Status_RadarStatus;
  
  sts.status_voltage_status            = status.Status_VoltageStatus;
  sts.status_temperature_status        = status.Status_TemperatureStatus;
  sts.status_blockage_status           = status.Status_BlockageStatus;
  // clang-format on

  status_pub.publish(sts);
}

void RadarBaseNode::AccLateralCogCallBack(
    const ars548_process::AccelerationLateralCog& msg) {
  char temp[4];
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x41;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 32;

  cvt.floatToByte(msg.acceleration_lateral_err_amp, temp);
  send_data[8]  = temp[0];
  send_data[9]  = temp[1];
  send_data[10] = temp[2];
  send_data[11] = temp[3];

  send_data[12] = (char)msg.acceleration_lateral_err_amp_invalid_flag;
  send_data[13] = (char)msg.qualifier_acceleration_lateral;

  cvt.floatToByte(msg.acceleration_lateral, temp);
  send_data[14] = temp[0];
  send_data[15] = temp[1];
  send_data[16] = temp[2];
  send_data[17] = temp[3];

  send_data[18] = (char)msg.acceleration_lateral_invalid_flag;
  send_data[19] = (char)msg.acceleration_lateral_event_data_qualifier;

  for (int i = 0; i < 20; i++) send_data[20 + i] = 0;

  // udp_io_->sendToRadar(send_data, 40);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 40);
}

void RadarBaseNode::AccLongitudinalCogCallBack(
    const ars548_process::AccelerationLongitudinalCog& msg) {
  char temp[4];
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x42;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 32;

  cvt.floatToByte(msg.acceleration_longitudinal_err_amp, temp);
  send_data[8]  = temp[0];
  send_data[9]  = temp[1];
  send_data[10] = temp[2];
  send_data[11] = temp[3];

  send_data[12] = (char)msg.acceleration_longitudinal_err_amp_invalid_flag;
  send_data[13] = (char)msg.qualifier_acceleration_longitudinal;

  cvt.floatToByte(msg.acceleration_longitudinal, temp);
  send_data[14] = temp[0];
  send_data[15] = temp[1];
  send_data[16] = temp[2];
  send_data[17] = temp[3];

  send_data[18] = (char)msg.acceleration_longitudinal_invalid_flag;
  send_data[19] = (char)msg.acceleration_longitudinal_event_data_qualifier;

  for (int i = 0; i < 20; i++) send_data[20 + i] = 0;

  // udp_io_->sendToRadar(send_data, 40);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 40);
}

void RadarBaseNode::CharacteristicSpeedCallBack(
    const ars548_process::CharacteristicSpeed& msg) {
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x48;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 11;

  send_data[8]  = (char)msg.characteristic_speed_err_amp;
  send_data[9]  = (char)msg.qualifier_characteristic_speed;
  send_data[10] = (char)msg.characteristic_speed;

  for (int i = 0; i < 8; i++) send_data[11 + i] = 0;

  // udp_io_->sendToRadar(send_data, 19);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 19);
}

void RadarBaseNode::DrivingDirectionCallBack(
    const ars548_process::DrivingDirection& msg) {
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x45;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 22;

  send_data[8] = (char)msg.driving_direction_unconfirmed;
  send_data[9] = (char)msg.driving_direction_confirmed;

  for (int i = 0; i < 20; i++) send_data[10 + i] = 0;

  // udp_io_->sendToRadar(send_data, 30);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 30);
}

void RadarBaseNode::SteeringAngleCallBack(
    const ars548_process::SteeringAngleFrontAxle& msg) {
  char temp[4];
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x47;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 32;

  send_data[8] = (char)msg.qualifier_steering_angle_front_axle;

  cvt.floatToByte(msg.steering_angle_front_axle_err_amp, temp);
  send_data[9]  = temp[0];
  send_data[10] = temp[1];
  send_data[11] = temp[2];
  send_data[12] = temp[3];

  send_data[13] = (char)msg.steering_angle_front_axle_err_amp_invalid_flag;

  cvt.floatToByte(msg.steering_angle_front_axle, temp);
  send_data[14] = temp[0];
  send_data[15] = temp[1];
  send_data[16] = temp[2];
  send_data[17] = temp[3];

  send_data[18] = (char)msg.steering_angle_front_axle_invalid_flag;
  send_data[19] = (char)msg.steering_angle_front_axle_event_data_qualifier;

  for (int i = 0; i < 20; i++) send_data[20 + i] = 0;

  // udp_io_->sendToRadar(send_data, 40);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 40);
}

void RadarBaseNode::VelocityVehicleCallBack(
    const ars548_process::VelocityVehicle& msg) {
  char temp[4];
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x43;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 28;

  send_data[8]  = (char)msg.status_velocity_near_standstill;
  send_data[9]  = (char)msg.qualifier_velocity_vehicle;
  send_data[10] = (char)msg.velocity_vehicle_event_data_qualifier;

  cvt.floatToByte(msg.velocity_vehicle, temp);
  send_data[11] = temp[0];
  send_data[12] = temp[1];
  send_data[13] = temp[2];
  send_data[14] = temp[3];

  send_data[15] = (char)msg.velocity_vehicle_invalid_flag;

  for (int i = 0; i < 20; i++) send_data[16 + i] = 0;

  // udp_io_->sendToRadar(send_data, 36);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 36);
}

void RadarBaseNode::YawRateCallBack(const ars548_process::YawRate& msg) {
  char temp[4];
  // service ID
  send_data[0] = 0;
  send_data[1] = 0;
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x46;
  // Length
  send_data[4] = 0;
  send_data[5] = 0;
  send_data[6] = 0;
  send_data[7] = 32;

  cvt.floatToByte(msg.yaw_rate_err_amp, temp);
  send_data[8]  = temp[0];
  send_data[9]  = temp[1];
  send_data[10] = temp[2];
  send_data[11] = temp[3];

  send_data[12] = (char)msg.yaw_rate_err_amp_invalid_flag;
  send_data[13] = (char)msg.qualifier_yaw_rate;

  cvt.floatToByte(msg.yaw_rate, temp);
  send_data[14] = temp[0];
  send_data[15] = temp[1];
  send_data[16] = temp[2];
  send_data[17] = temp[3];

  send_data[18] = (char)msg.yaw_rate_invalid_flag;
  send_data[19] = (char)msg.yaw_rate_event_data_qualifier;

  for (int i = 0; i < 20; i++) send_data[20 + i] = 0;

  // udp_io_->sendToRadar(send_data, 40);
  udp_io_->sendToRadar(param_->radar_ip, param_->radar_port, send_data, 40);
}

void RadarBaseNode::Reset() {
  //// AccLateralCogCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x41;
  // Length
  send_data[7] = 32;
  udp_io_->sendToRadar(send_data, 40);

  //// AccLongitudinalCogCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x42;
  // Length
  send_data[7] = 32;
  udp_io_->sendToRadar(send_data, 40);

  //// CharacteristicSpeedCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x48;
  // Length
  send_data[7] = 11;
  udp_io_->sendToRadar(send_data, 19);

  //// DrivingDirectionCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x45;
  // Length
  send_data[7] = 22;
  udp_io_->sendToRadar(send_data, 30);

  //// SteeringAngleCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x47;
  // Length
  send_data[7] = 32;
  udp_io_->sendToRadar(send_data, 40);

  //// VelocityVehicleCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x43;
  // Length
  send_data[7] = 28;
  udp_io_->sendToRadar(send_data, 36);

  //// YawRateCallBack
  memset(send_data, 0, 1024);
  // Method ID
  send_data[2] = 0x01;
  send_data[3] = 0x46;
  // Length
  send_data[7] = 32;
  udp_io_->sendToRadar(send_data, 40);
}

bool RadarBaseNode::Init(ros::NodeHandle& nh,
                         std::shared_ptr<RuntimeConfig> param) {
  node   = nh;
  param_ = param;

  std::string ns    = param_->GetVehicleName();
  std::string topic = "/" + ns + param_->channel_name;
  // std::cout << "topic: " << topic << std::endl;

  // clang-format off
  objects_pub = nh.advertise<ars548_process::ObjectList>(topic + "/object_list", 10);
  detections_pub = nh.advertise<ars548_process::DetectionList>(topic + "/detection_list", 10);
  status_pub = nh.advertise<ars548_process::SensorStatus>(topic + "/radar_status", 10);

  // I don't need this config
  /*
  acc_lateral_cog_sub = nh.subscribe(topic + "/acc_lateral_cog", 10, &RadarProcessNode::AccLateralCogCallBack, this);
  acc_longitudinal_cog_sub = nh.subscribe(topic + "/acc_longitudinal_cog", 10, &RadarProcessNode::AccLongitudinalCogCallBack, this);
  characteristic_speed_sub = nh.subscribe(topic + "/characteristic_speed", 10, &RadarProcessNode::CharacteristicSpeedCallBack, this);
  driving_direction_sub = nh.subscribe(topic + "/driving_direction", 10, &RadarProcessNode::DrivingDirectionCallBack, this);
  steering_angle_sub = nh.subscribe(topic + "/steering_angle", 10, &RadarProcessNode::SteeringAngleCallBack, this);
  velocity_vehicle_sub = nh.subscribe(topic + "/velocity_vehicle", 10, &RadarProcessNode::VelocityVehicleCallBack, this);
  yaw_rate_sub = nh.subscribe(topic + "/yaw_rate", 10, &RadarProcessNode::YawRateCallBack, this);
  */
  // clang-format on

  return true;
}

void RadarBaseNode::Run() {
  ros::Rate r(10);

  // 这里是为了配合对雷达进行实时调优，所以需要一直发送数据
  // 目前不需要，所以全部设置为 0
  while (ros::ok()) {
    // ros::spinOnce();

    // I set all is zero
    this->Reset();

    r.sleep();
  }
}