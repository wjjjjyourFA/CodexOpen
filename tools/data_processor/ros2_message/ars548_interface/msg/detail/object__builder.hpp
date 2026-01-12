// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ars548_interface:msg/Object.idl
// generated code does not contain a copyright notice

#ifndef ARS548_INTERFACE__MSG__DETAIL__OBJECT__BUILDER_HPP_
#define ARS548_INTERFACE__MSG__DETAIL__OBJECT__BUILDER_HPP_

#include "ars548_interface/msg/detail/object__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace ars548_interface
{

namespace msg
{

namespace builder
{

class Init_Object_u_shape_width_edge_std
{
public:
  explicit Init_Object_u_shape_width_edge_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  ::ars548_interface::msg::Object u_shape_width_edge_std(::ars548_interface::msg::Object::_u_shape_width_edge_std_type arg)
  {
    msg_.u_shape_width_edge_std = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_width_edge_mean
{
public:
  explicit Init_Object_u_shape_width_edge_mean(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_width_edge_std u_shape_width_edge_mean(::ars548_interface::msg::Object::_u_shape_width_edge_mean_type arg)
  {
    msg_.u_shape_width_edge_mean = std::move(arg);
    return Init_Object_u_shape_width_edge_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_width_edge_invalid_flags
{
public:
  explicit Init_Object_u_shape_width_edge_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_width_edge_mean u_shape_width_edge_invalid_flags(::ars548_interface::msg::Object::_u_shape_width_edge_invalid_flags_type arg)
  {
    msg_.u_shape_width_edge_invalid_flags = std::move(arg);
    return Init_Object_u_shape_width_edge_mean(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_width_status
{
public:
  explicit Init_Object_u_shape_width_status(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_width_edge_invalid_flags u_shape_width_status(::ars548_interface::msg::Object::_u_shape_width_status_type arg)
  {
    msg_.u_shape_width_status = std::move(arg);
    return Init_Object_u_shape_width_edge_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_length_edge_std
{
public:
  explicit Init_Object_u_shape_length_edge_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_width_status u_shape_length_edge_std(::ars548_interface::msg::Object::_u_shape_length_edge_std_type arg)
  {
    msg_.u_shape_length_edge_std = std::move(arg);
    return Init_Object_u_shape_width_status(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_length_edge_mean
{
public:
  explicit Init_Object_u_shape_length_edge_mean(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_length_edge_std u_shape_length_edge_mean(::ars548_interface::msg::Object::_u_shape_length_edge_mean_type arg)
  {
    msg_.u_shape_length_edge_mean = std::move(arg);
    return Init_Object_u_shape_length_edge_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_length_edge_invalid_flags
{
public:
  explicit Init_Object_u_shape_length_edge_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_length_edge_mean u_shape_length_edge_invalid_flags(::ars548_interface::msg::Object::_u_shape_length_edge_invalid_flags_type arg)
  {
    msg_.u_shape_length_edge_invalid_flags = std::move(arg);
    return Init_Object_u_shape_length_edge_mean(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_shape_length_status
{
public:
  explicit Init_Object_u_shape_length_status(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_length_edge_invalid_flags u_shape_length_status(::ars548_interface::msg::Object::_u_shape_length_status_type arg)
  {
    msg_.u_shape_length_status = std::move(arg);
    return Init_Object_u_shape_length_edge_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_orientation_rate_std
{
public:
  explicit Init_Object_u_dynamics_orientation_rate_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_shape_length_status u_dynamics_orientation_rate_std(::ars548_interface::msg::Object::_u_dynamics_orientation_rate_std_type arg)
  {
    msg_.u_dynamics_orientation_rate_std = std::move(arg);
    return Init_Object_u_shape_length_status(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_orientation_rate_mean
{
public:
  explicit Init_Object_u_dynamics_orientation_rate_mean(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_orientation_rate_std u_dynamics_orientation_rate_mean(::ars548_interface::msg::Object::_u_dynamics_orientation_rate_mean_type arg)
  {
    msg_.u_dynamics_orientation_rate_mean = std::move(arg);
    return Init_Object_u_dynamics_orientation_rate_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_orientation_invalid_flags
{
public:
  explicit Init_Object_u_dynamics_orientation_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_orientation_rate_mean u_dynamics_orientation_invalid_flags(::ars548_interface::msg::Object::_u_dynamics_orientation_invalid_flags_type arg)
  {
    msg_.u_dynamics_orientation_invalid_flags = std::move(arg);
    return Init_Object_u_dynamics_orientation_rate_mean(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_accel_covariance_xy
{
public:
  explicit Init_Object_f_dynamics_rel_accel_covariance_xy(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_orientation_invalid_flags f_dynamics_rel_accel_covariance_xy(::ars548_interface::msg::Object::_f_dynamics_rel_accel_covariance_xy_type arg)
  {
    msg_.f_dynamics_rel_accel_covariance_xy = std::move(arg);
    return Init_Object_u_dynamics_orientation_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_accel_y_std
{
public:
  explicit Init_Object_f_dynamics_rel_accel_y_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_accel_covariance_xy f_dynamics_rel_accel_y_std(::ars548_interface::msg::Object::_f_dynamics_rel_accel_y_std_type arg)
  {
    msg_.f_dynamics_rel_accel_y_std = std::move(arg);
    return Init_Object_f_dynamics_rel_accel_covariance_xy(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_accel_x_std
{
public:
  explicit Init_Object_f_dynamics_rel_accel_x_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_accel_y_std f_dynamics_rel_accel_x_std(::ars548_interface::msg::Object::_f_dynamics_rel_accel_x_std_type arg)
  {
    msg_.f_dynamics_rel_accel_x_std = std::move(arg);
    return Init_Object_f_dynamics_rel_accel_y_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_accel_y
{
public:
  explicit Init_Object_f_dynamics_rel_accel_y(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_accel_x_std f_dynamics_rel_accel_y(::ars548_interface::msg::Object::_f_dynamics_rel_accel_y_type arg)
  {
    msg_.f_dynamics_rel_accel_y = std::move(arg);
    return Init_Object_f_dynamics_rel_accel_x_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_accel_x
{
public:
  explicit Init_Object_f_dynamics_rel_accel_x(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_accel_y f_dynamics_rel_accel_x(::ars548_interface::msg::Object::_f_dynamics_rel_accel_x_type arg)
  {
    msg_.f_dynamics_rel_accel_x = std::move(arg);
    return Init_Object_f_dynamics_rel_accel_y(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_rel_accel_invalid_flags
{
public:
  explicit Init_Object_u_dynamics_rel_accel_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_accel_x u_dynamics_rel_accel_invalid_flags(::ars548_interface::msg::Object::_u_dynamics_rel_accel_invalid_flags_type arg)
  {
    msg_.u_dynamics_rel_accel_invalid_flags = std::move(arg);
    return Init_Object_f_dynamics_rel_accel_x(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_accel_covariance_xy
{
public:
  explicit Init_Object_f_dynamics_abs_accel_covariance_xy(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_rel_accel_invalid_flags f_dynamics_abs_accel_covariance_xy(::ars548_interface::msg::Object::_f_dynamics_abs_accel_covariance_xy_type arg)
  {
    msg_.f_dynamics_abs_accel_covariance_xy = std::move(arg);
    return Init_Object_u_dynamics_rel_accel_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_accel_y_std
{
public:
  explicit Init_Object_f_dynamics_abs_accel_y_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_accel_covariance_xy f_dynamics_abs_accel_y_std(::ars548_interface::msg::Object::_f_dynamics_abs_accel_y_std_type arg)
  {
    msg_.f_dynamics_abs_accel_y_std = std::move(arg);
    return Init_Object_f_dynamics_abs_accel_covariance_xy(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_accel_x_std
{
public:
  explicit Init_Object_f_dynamics_abs_accel_x_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_accel_y_std f_dynamics_abs_accel_x_std(::ars548_interface::msg::Object::_f_dynamics_abs_accel_x_std_type arg)
  {
    msg_.f_dynamics_abs_accel_x_std = std::move(arg);
    return Init_Object_f_dynamics_abs_accel_y_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_accel_y
{
public:
  explicit Init_Object_f_dynamics_abs_accel_y(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_accel_x_std f_dynamics_abs_accel_y(::ars548_interface::msg::Object::_f_dynamics_abs_accel_y_type arg)
  {
    msg_.f_dynamics_abs_accel_y = std::move(arg);
    return Init_Object_f_dynamics_abs_accel_x_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_accel_x
{
public:
  explicit Init_Object_f_dynamics_abs_accel_x(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_accel_y f_dynamics_abs_accel_x(::ars548_interface::msg::Object::_f_dynamics_abs_accel_x_type arg)
  {
    msg_.f_dynamics_abs_accel_x = std::move(arg);
    return Init_Object_f_dynamics_abs_accel_y(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_abs_accel_invalid_flags
{
public:
  explicit Init_Object_u_dynamics_abs_accel_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_accel_x u_dynamics_abs_accel_invalid_flags(::ars548_interface::msg::Object::_u_dynamics_abs_accel_invalid_flags_type arg)
  {
    msg_.u_dynamics_abs_accel_invalid_flags = std::move(arg);
    return Init_Object_f_dynamics_abs_accel_x(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_vel_covariance_xy
{
public:
  explicit Init_Object_f_dynamics_rel_vel_covariance_xy(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_abs_accel_invalid_flags f_dynamics_rel_vel_covariance_xy(::ars548_interface::msg::Object::_f_dynamics_rel_vel_covariance_xy_type arg)
  {
    msg_.f_dynamics_rel_vel_covariance_xy = std::move(arg);
    return Init_Object_u_dynamics_abs_accel_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_vel_y_std
{
public:
  explicit Init_Object_f_dynamics_rel_vel_y_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_vel_covariance_xy f_dynamics_rel_vel_y_std(::ars548_interface::msg::Object::_f_dynamics_rel_vel_y_std_type arg)
  {
    msg_.f_dynamics_rel_vel_y_std = std::move(arg);
    return Init_Object_f_dynamics_rel_vel_covariance_xy(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_vel_x_std
{
public:
  explicit Init_Object_f_dynamics_rel_vel_x_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_vel_y_std f_dynamics_rel_vel_x_std(::ars548_interface::msg::Object::_f_dynamics_rel_vel_x_std_type arg)
  {
    msg_.f_dynamics_rel_vel_x_std = std::move(arg);
    return Init_Object_f_dynamics_rel_vel_y_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_vel_y
{
public:
  explicit Init_Object_f_dynamics_rel_vel_y(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_vel_x_std f_dynamics_rel_vel_y(::ars548_interface::msg::Object::_f_dynamics_rel_vel_y_type arg)
  {
    msg_.f_dynamics_rel_vel_y = std::move(arg);
    return Init_Object_f_dynamics_rel_vel_x_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_rel_vel_x
{
public:
  explicit Init_Object_f_dynamics_rel_vel_x(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_vel_y f_dynamics_rel_vel_x(::ars548_interface::msg::Object::_f_dynamics_rel_vel_x_type arg)
  {
    msg_.f_dynamics_rel_vel_x = std::move(arg);
    return Init_Object_f_dynamics_rel_vel_y(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_rel_vel_invalid_flags
{
public:
  explicit Init_Object_u_dynamics_rel_vel_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_rel_vel_x u_dynamics_rel_vel_invalid_flags(::ars548_interface::msg::Object::_u_dynamics_rel_vel_invalid_flags_type arg)
  {
    msg_.u_dynamics_rel_vel_invalid_flags = std::move(arg);
    return Init_Object_f_dynamics_rel_vel_x(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_vel_covariance_xy
{
public:
  explicit Init_Object_f_dynamics_abs_vel_covariance_xy(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_rel_vel_invalid_flags f_dynamics_abs_vel_covariance_xy(::ars548_interface::msg::Object::_f_dynamics_abs_vel_covariance_xy_type arg)
  {
    msg_.f_dynamics_abs_vel_covariance_xy = std::move(arg);
    return Init_Object_u_dynamics_rel_vel_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_vel_y_std
{
public:
  explicit Init_Object_f_dynamics_abs_vel_y_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_vel_covariance_xy f_dynamics_abs_vel_y_std(::ars548_interface::msg::Object::_f_dynamics_abs_vel_y_std_type arg)
  {
    msg_.f_dynamics_abs_vel_y_std = std::move(arg);
    return Init_Object_f_dynamics_abs_vel_covariance_xy(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_vel_x_std
{
public:
  explicit Init_Object_f_dynamics_abs_vel_x_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_vel_y_std f_dynamics_abs_vel_x_std(::ars548_interface::msg::Object::_f_dynamics_abs_vel_x_std_type arg)
  {
    msg_.f_dynamics_abs_vel_x_std = std::move(arg);
    return Init_Object_f_dynamics_abs_vel_y_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_vel_y
{
public:
  explicit Init_Object_f_dynamics_abs_vel_y(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_vel_x_std f_dynamics_abs_vel_y(::ars548_interface::msg::Object::_f_dynamics_abs_vel_y_type arg)
  {
    msg_.f_dynamics_abs_vel_y = std::move(arg);
    return Init_Object_f_dynamics_abs_vel_x_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_f_dynamics_abs_vel_x
{
public:
  explicit Init_Object_f_dynamics_abs_vel_x(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_vel_y f_dynamics_abs_vel_x(::ars548_interface::msg::Object::_f_dynamics_abs_vel_x_type arg)
  {
    msg_.f_dynamics_abs_vel_x = std::move(arg);
    return Init_Object_f_dynamics_abs_vel_y(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_dynamics_abs_vel_invalid_flags
{
public:
  explicit Init_Object_u_dynamics_abs_vel_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_f_dynamics_abs_vel_x u_dynamics_abs_vel_invalid_flags(::ars548_interface::msg::Object::_u_dynamics_abs_vel_invalid_flags_type arg)
  {
    msg_.u_dynamics_abs_vel_invalid_flags = std::move(arg);
    return Init_Object_f_dynamics_abs_vel_x(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_underdrivable
{
public:
  explicit Init_Object_u_classification_underdrivable(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_dynamics_abs_vel_invalid_flags u_classification_underdrivable(::ars548_interface::msg::Object::_u_classification_underdrivable_type arg)
  {
    msg_.u_classification_underdrivable = std::move(arg);
    return Init_Object_u_dynamics_abs_vel_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_overdrivable
{
public:
  explicit Init_Object_u_classification_overdrivable(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_underdrivable u_classification_overdrivable(::ars548_interface::msg::Object::_u_classification_overdrivable_type arg)
  {
    msg_.u_classification_overdrivable = std::move(arg);
    return Init_Object_u_classification_underdrivable(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_unknown
{
public:
  explicit Init_Object_u_classification_unknown(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_overdrivable u_classification_unknown(::ars548_interface::msg::Object::_u_classification_unknown_type arg)
  {
    msg_.u_classification_unknown = std::move(arg);
    return Init_Object_u_classification_overdrivable(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_hazard
{
public:
  explicit Init_Object_u_classification_hazard(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_unknown u_classification_hazard(::ars548_interface::msg::Object::_u_classification_hazard_type arg)
  {
    msg_.u_classification_hazard = std::move(arg);
    return Init_Object_u_classification_unknown(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_animal
{
public:
  explicit Init_Object_u_classification_animal(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_hazard u_classification_animal(::ars548_interface::msg::Object::_u_classification_animal_type arg)
  {
    msg_.u_classification_animal = std::move(arg);
    return Init_Object_u_classification_hazard(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_pedestrian
{
public:
  explicit Init_Object_u_classification_pedestrian(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_animal u_classification_pedestrian(::ars548_interface::msg::Object::_u_classification_pedestrian_type arg)
  {
    msg_.u_classification_pedestrian = std::move(arg);
    return Init_Object_u_classification_animal(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_bicycle
{
public:
  explicit Init_Object_u_classification_bicycle(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_pedestrian u_classification_bicycle(::ars548_interface::msg::Object::_u_classification_bicycle_type arg)
  {
    msg_.u_classification_bicycle = std::move(arg);
    return Init_Object_u_classification_pedestrian(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_motorcycle
{
public:
  explicit Init_Object_u_classification_motorcycle(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_bicycle u_classification_motorcycle(::ars548_interface::msg::Object::_u_classification_motorcycle_type arg)
  {
    msg_.u_classification_motorcycle = std::move(arg);
    return Init_Object_u_classification_bicycle(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_truck
{
public:
  explicit Init_Object_u_classification_truck(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_motorcycle u_classification_truck(::ars548_interface::msg::Object::_u_classification_truck_type arg)
  {
    msg_.u_classification_truck = std::move(arg);
    return Init_Object_u_classification_motorcycle(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_classification_car
{
public:
  explicit Init_Object_u_classification_car(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_truck u_classification_car(::ars548_interface::msg::Object::_u_classification_car_type arg)
  {
    msg_.u_classification_car = std::move(arg);
    return Init_Object_u_classification_truck(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_existence_ppv
{
public:
  explicit Init_Object_u_existence_ppv(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_classification_car u_existence_ppv(::ars548_interface::msg::Object::_u_existence_ppv_type arg)
  {
    msg_.u_existence_ppv = std::move(arg);
    return Init_Object_u_classification_car(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_existence_probability
{
public:
  explicit Init_Object_u_existence_probability(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_existence_ppv u_existence_probability(::ars548_interface::msg::Object::_u_existence_probability_type arg)
  {
    msg_.u_existence_probability = std::move(arg);
    return Init_Object_u_existence_ppv(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_existence_invalid_flags
{
public:
  explicit Init_Object_u_existence_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_existence_probability u_existence_invalid_flags(::ars548_interface::msg::Object::_u_existence_invalid_flags_type arg)
  {
    msg_.u_existence_invalid_flags = std::move(arg);
    return Init_Object_u_existence_probability(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_orientation_std
{
public:
  explicit Init_Object_u_position_orientation_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_existence_invalid_flags u_position_orientation_std(::ars548_interface::msg::Object::_u_position_orientation_std_type arg)
  {
    msg_.u_position_orientation_std = std::move(arg);
    return Init_Object_u_existence_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_orientation
{
public:
  explicit Init_Object_u_position_orientation(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_orientation_std u_position_orientation(::ars548_interface::msg::Object::_u_position_orientation_type arg)
  {
    msg_.u_position_orientation = std::move(arg);
    return Init_Object_u_position_orientation_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_covariance_xy
{
public:
  explicit Init_Object_u_position_covariance_xy(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_orientation u_position_covariance_xy(::ars548_interface::msg::Object::_u_position_covariance_xy_type arg)
  {
    msg_.u_position_covariance_xy = std::move(arg);
    return Init_Object_u_position_orientation(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_z_std
{
public:
  explicit Init_Object_u_position_z_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_covariance_xy u_position_z_std(::ars548_interface::msg::Object::_u_position_z_std_type arg)
  {
    msg_.u_position_z_std = std::move(arg);
    return Init_Object_u_position_covariance_xy(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_y_std
{
public:
  explicit Init_Object_u_position_y_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_z_std u_position_y_std(::ars548_interface::msg::Object::_u_position_y_std_type arg)
  {
    msg_.u_position_y_std = std::move(arg);
    return Init_Object_u_position_z_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_x_std
{
public:
  explicit Init_Object_u_position_x_std(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_y_std u_position_x_std(::ars548_interface::msg::Object::_u_position_x_std_type arg)
  {
    msg_.u_position_x_std = std::move(arg);
    return Init_Object_u_position_y_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_z
{
public:
  explicit Init_Object_u_position_z(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_x_std u_position_z(::ars548_interface::msg::Object::_u_position_z_type arg)
  {
    msg_.u_position_z = std::move(arg);
    return Init_Object_u_position_x_std(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_y
{
public:
  explicit Init_Object_u_position_y(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_z u_position_y(::ars548_interface::msg::Object::_u_position_y_type arg)
  {
    msg_.u_position_y = std::move(arg);
    return Init_Object_u_position_z(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_x
{
public:
  explicit Init_Object_u_position_x(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_y u_position_x(::ars548_interface::msg::Object::_u_position_x_type arg)
  {
    msg_.u_position_x = std::move(arg);
    return Init_Object_u_position_y(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_reference
{
public:
  explicit Init_Object_u_position_reference(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_x u_position_reference(::ars548_interface::msg::Object::_u_position_reference_type arg)
  {
    msg_.u_position_reference = std::move(arg);
    return Init_Object_u_position_x(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_position_invalid_flags
{
public:
  explicit Init_Object_u_position_invalid_flags(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_reference u_position_invalid_flags(::ars548_interface::msg::Object::_u_position_invalid_flags_type arg)
  {
    msg_.u_position_invalid_flags = std::move(arg);
    return Init_Object_u_position_reference(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_status_movement
{
public:
  explicit Init_Object_u_status_movement(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_position_invalid_flags u_status_movement(::ars548_interface::msg::Object::_u_status_movement_type arg)
  {
    msg_.u_status_movement = std::move(arg);
    return Init_Object_u_position_invalid_flags(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_status_measurement
{
public:
  explicit Init_Object_u_status_measurement(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_status_movement u_status_measurement(::ars548_interface::msg::Object::_u_status_measurement_type arg)
  {
    msg_.u_status_measurement = std::move(arg);
    return Init_Object_u_status_movement(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_age
{
public:
  explicit Init_Object_u_age(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_status_measurement u_age(::ars548_interface::msg::Object::_u_age_type arg)
  {
    msg_.u_age = std::move(arg);
    return Init_Object_u_status_measurement(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_id
{
public:
  explicit Init_Object_u_id(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_age u_id(::ars548_interface::msg::Object::_u_id_type arg)
  {
    msg_.u_id = std::move(arg);
    return Init_Object_u_age(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_u_status_sensor
{
public:
  explicit Init_Object_u_status_sensor(::ars548_interface::msg::Object & msg)
  : msg_(msg)
  {}
  Init_Object_u_id u_status_sensor(::ars548_interface::msg::Object::_u_status_sensor_type arg)
  {
    msg_.u_status_sensor = std::move(arg);
    return Init_Object_u_id(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

class Init_Object_header
{
public:
  Init_Object_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Object_u_status_sensor header(::ars548_interface::msg::Object::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Object_u_status_sensor(msg_);
  }

private:
  ::ars548_interface::msg::Object msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ars548_interface::msg::Object>()
{
  return ars548_interface::msg::builder::Init_Object_header();
}

}  // namespace ars548_interface

#endif  // ARS548_INTERFACE__MSG__DETAIL__OBJECT__BUILDER_HPP_
