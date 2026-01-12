// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/ChassisInfo.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__CHASSIS_INFO__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__CHASSIS_INFO__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__self_state__msg__ChassisInfo __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__ChassisInfo __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ChassisInfo_
{
  using Type = ChassisInfo_<ContainerAllocator>;

  explicit ChassisInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->lf_wheel_speed = 0.0;
      this->lr_wheel_speed = 0.0;
      this->rf_wheel_speed = 0.0;
      this->rr_wheel_speed = 0.0;
      this->lf_wheel_pulse_counter = 0ull;
      this->lr_wheel_pulse_counter = 0ull;
      this->rf_wheel_pulse_counter = 0ull;
      this->rr_wheel_pulse_counter = 0ull;
      this->lon_acc = 0.0;
      this->lat_acc = 0.0;
      this->yaw_rate = 0.0;
      this->cur_spd = 0.0;
      this->exp_speed = 0.0;
      this->exp_acceleration = 0.0;
      this->exp_yaw_rate = 0.0;
      this->exp_steer_wheel_angle = 0.0;
      this->exp_steer_wheel_speed = 0.0;
      this->exp_fuel = 0.0;
      this->exp_brake = 0.0;
      this->exp_gear = 0;
      this->exp_handbrake = 0;
      this->cur_steer_wheel_angle = 0.0;
      this->cur_steer_wheel_speed = 0.0;
      this->cur_fuel = 0.0;
      this->cur_brake = 0.0;
      this->cur_engine_speed = 0.0;
      this->cur_gear = 0;
      this->cur_handbrake = 0;
      this->actual_gear = 0;
      this->exp_auto_enable = 0;
      this->auto_enable = 0;
      this->oil = 0;
      this->battery_soc = 0;
      this->sta_code = 0ul;
      this->err_code = 0ul;
      this->horn = 0;
      this->left_signal_light = 0;
      this->right_signal_light = 0;
      this->headlight = 0;
      this->brake_light = 0;
      this->postion_light = 0;
      this->high_beam = 0;
      this->low_beam = 0;
      this->front_fog_light = 0;
      this->rear_fog_light = 0;
      this->state_act = 0;
      this->electrical_power_steering_availablity_status = 0;
      this->veh_spd_valid_flag = 0;
      std::fill<typename std::array<uint32_t, 8>::iterator, uint32_t>(this->reserved.begin(), this->reserved.end(), 0ul);
    }
  }

  explicit ChassisInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reserved(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->local_time = 0.0;
      this->utc_time = 0.0;
      this->message_num = 0l;
      this->lf_wheel_speed = 0.0;
      this->lr_wheel_speed = 0.0;
      this->rf_wheel_speed = 0.0;
      this->rr_wheel_speed = 0.0;
      this->lf_wheel_pulse_counter = 0ull;
      this->lr_wheel_pulse_counter = 0ull;
      this->rf_wheel_pulse_counter = 0ull;
      this->rr_wheel_pulse_counter = 0ull;
      this->lon_acc = 0.0;
      this->lat_acc = 0.0;
      this->yaw_rate = 0.0;
      this->cur_spd = 0.0;
      this->exp_speed = 0.0;
      this->exp_acceleration = 0.0;
      this->exp_yaw_rate = 0.0;
      this->exp_steer_wheel_angle = 0.0;
      this->exp_steer_wheel_speed = 0.0;
      this->exp_fuel = 0.0;
      this->exp_brake = 0.0;
      this->exp_gear = 0;
      this->exp_handbrake = 0;
      this->cur_steer_wheel_angle = 0.0;
      this->cur_steer_wheel_speed = 0.0;
      this->cur_fuel = 0.0;
      this->cur_brake = 0.0;
      this->cur_engine_speed = 0.0;
      this->cur_gear = 0;
      this->cur_handbrake = 0;
      this->actual_gear = 0;
      this->exp_auto_enable = 0;
      this->auto_enable = 0;
      this->oil = 0;
      this->battery_soc = 0;
      this->sta_code = 0ul;
      this->err_code = 0ul;
      this->horn = 0;
      this->left_signal_light = 0;
      this->right_signal_light = 0;
      this->headlight = 0;
      this->brake_light = 0;
      this->postion_light = 0;
      this->high_beam = 0;
      this->low_beam = 0;
      this->front_fog_light = 0;
      this->rear_fog_light = 0;
      this->state_act = 0;
      this->electrical_power_steering_availablity_status = 0;
      this->veh_spd_valid_flag = 0;
      std::fill<typename std::array<uint32_t, 8>::iterator, uint32_t>(this->reserved.begin(), this->reserved.end(), 0ul);
    }
  }

  // field types and members
  using _local_time_type =
    double;
  _local_time_type local_time;
  using _utc_time_type =
    double;
  _utc_time_type utc_time;
  using _message_num_type =
    int32_t;
  _message_num_type message_num;
  using _lf_wheel_speed_type =
    double;
  _lf_wheel_speed_type lf_wheel_speed;
  using _lr_wheel_speed_type =
    double;
  _lr_wheel_speed_type lr_wheel_speed;
  using _rf_wheel_speed_type =
    double;
  _rf_wheel_speed_type rf_wheel_speed;
  using _rr_wheel_speed_type =
    double;
  _rr_wheel_speed_type rr_wheel_speed;
  using _lf_wheel_pulse_counter_type =
    uint64_t;
  _lf_wheel_pulse_counter_type lf_wheel_pulse_counter;
  using _lr_wheel_pulse_counter_type =
    uint64_t;
  _lr_wheel_pulse_counter_type lr_wheel_pulse_counter;
  using _rf_wheel_pulse_counter_type =
    uint64_t;
  _rf_wheel_pulse_counter_type rf_wheel_pulse_counter;
  using _rr_wheel_pulse_counter_type =
    uint64_t;
  _rr_wheel_pulse_counter_type rr_wheel_pulse_counter;
  using _lon_acc_type =
    double;
  _lon_acc_type lon_acc;
  using _lat_acc_type =
    double;
  _lat_acc_type lat_acc;
  using _yaw_rate_type =
    double;
  _yaw_rate_type yaw_rate;
  using _cur_spd_type =
    double;
  _cur_spd_type cur_spd;
  using _exp_speed_type =
    double;
  _exp_speed_type exp_speed;
  using _exp_acceleration_type =
    double;
  _exp_acceleration_type exp_acceleration;
  using _exp_yaw_rate_type =
    double;
  _exp_yaw_rate_type exp_yaw_rate;
  using _exp_steer_wheel_angle_type =
    double;
  _exp_steer_wheel_angle_type exp_steer_wheel_angle;
  using _exp_steer_wheel_speed_type =
    double;
  _exp_steer_wheel_speed_type exp_steer_wheel_speed;
  using _exp_fuel_type =
    double;
  _exp_fuel_type exp_fuel;
  using _exp_brake_type =
    double;
  _exp_brake_type exp_brake;
  using _exp_gear_type =
    int8_t;
  _exp_gear_type exp_gear;
  using _exp_handbrake_type =
    int8_t;
  _exp_handbrake_type exp_handbrake;
  using _cur_steer_wheel_angle_type =
    double;
  _cur_steer_wheel_angle_type cur_steer_wheel_angle;
  using _cur_steer_wheel_speed_type =
    double;
  _cur_steer_wheel_speed_type cur_steer_wheel_speed;
  using _cur_fuel_type =
    double;
  _cur_fuel_type cur_fuel;
  using _cur_brake_type =
    double;
  _cur_brake_type cur_brake;
  using _cur_engine_speed_type =
    double;
  _cur_engine_speed_type cur_engine_speed;
  using _cur_gear_type =
    uint8_t;
  _cur_gear_type cur_gear;
  using _cur_handbrake_type =
    uint8_t;
  _cur_handbrake_type cur_handbrake;
  using _actual_gear_type =
    uint8_t;
  _actual_gear_type actual_gear;
  using _exp_auto_enable_type =
    int8_t;
  _exp_auto_enable_type exp_auto_enable;
  using _auto_enable_type =
    int8_t;
  _auto_enable_type auto_enable;
  using _oil_type =
    int8_t;
  _oil_type oil;
  using _battery_soc_type =
    int8_t;
  _battery_soc_type battery_soc;
  using _sta_code_type =
    uint32_t;
  _sta_code_type sta_code;
  using _err_code_type =
    uint32_t;
  _err_code_type err_code;
  using _horn_type =
    uint8_t;
  _horn_type horn;
  using _left_signal_light_type =
    uint8_t;
  _left_signal_light_type left_signal_light;
  using _right_signal_light_type =
    uint8_t;
  _right_signal_light_type right_signal_light;
  using _headlight_type =
    uint8_t;
  _headlight_type headlight;
  using _brake_light_type =
    uint8_t;
  _brake_light_type brake_light;
  using _postion_light_type =
    uint8_t;
  _postion_light_type postion_light;
  using _high_beam_type =
    uint8_t;
  _high_beam_type high_beam;
  using _low_beam_type =
    uint8_t;
  _low_beam_type low_beam;
  using _front_fog_light_type =
    uint8_t;
  _front_fog_light_type front_fog_light;
  using _rear_fog_light_type =
    uint8_t;
  _rear_fog_light_type rear_fog_light;
  using _state_act_type =
    uint8_t;
  _state_act_type state_act;
  using _electrical_power_steering_availablity_status_type =
    uint8_t;
  _electrical_power_steering_availablity_status_type electrical_power_steering_availablity_status;
  using _veh_spd_valid_flag_type =
    uint8_t;
  _veh_spd_valid_flag_type veh_spd_valid_flag;
  using _reserved_type =
    std::array<uint32_t, 8>;
  _reserved_type reserved;

  // setters for named parameter idiom
  Type & set__local_time(
    const double & _arg)
  {
    this->local_time = _arg;
    return *this;
  }
  Type & set__utc_time(
    const double & _arg)
  {
    this->utc_time = _arg;
    return *this;
  }
  Type & set__message_num(
    const int32_t & _arg)
  {
    this->message_num = _arg;
    return *this;
  }
  Type & set__lf_wheel_speed(
    const double & _arg)
  {
    this->lf_wheel_speed = _arg;
    return *this;
  }
  Type & set__lr_wheel_speed(
    const double & _arg)
  {
    this->lr_wheel_speed = _arg;
    return *this;
  }
  Type & set__rf_wheel_speed(
    const double & _arg)
  {
    this->rf_wheel_speed = _arg;
    return *this;
  }
  Type & set__rr_wheel_speed(
    const double & _arg)
  {
    this->rr_wheel_speed = _arg;
    return *this;
  }
  Type & set__lf_wheel_pulse_counter(
    const uint64_t & _arg)
  {
    this->lf_wheel_pulse_counter = _arg;
    return *this;
  }
  Type & set__lr_wheel_pulse_counter(
    const uint64_t & _arg)
  {
    this->lr_wheel_pulse_counter = _arg;
    return *this;
  }
  Type & set__rf_wheel_pulse_counter(
    const uint64_t & _arg)
  {
    this->rf_wheel_pulse_counter = _arg;
    return *this;
  }
  Type & set__rr_wheel_pulse_counter(
    const uint64_t & _arg)
  {
    this->rr_wheel_pulse_counter = _arg;
    return *this;
  }
  Type & set__lon_acc(
    const double & _arg)
  {
    this->lon_acc = _arg;
    return *this;
  }
  Type & set__lat_acc(
    const double & _arg)
  {
    this->lat_acc = _arg;
    return *this;
  }
  Type & set__yaw_rate(
    const double & _arg)
  {
    this->yaw_rate = _arg;
    return *this;
  }
  Type & set__cur_spd(
    const double & _arg)
  {
    this->cur_spd = _arg;
    return *this;
  }
  Type & set__exp_speed(
    const double & _arg)
  {
    this->exp_speed = _arg;
    return *this;
  }
  Type & set__exp_acceleration(
    const double & _arg)
  {
    this->exp_acceleration = _arg;
    return *this;
  }
  Type & set__exp_yaw_rate(
    const double & _arg)
  {
    this->exp_yaw_rate = _arg;
    return *this;
  }
  Type & set__exp_steer_wheel_angle(
    const double & _arg)
  {
    this->exp_steer_wheel_angle = _arg;
    return *this;
  }
  Type & set__exp_steer_wheel_speed(
    const double & _arg)
  {
    this->exp_steer_wheel_speed = _arg;
    return *this;
  }
  Type & set__exp_fuel(
    const double & _arg)
  {
    this->exp_fuel = _arg;
    return *this;
  }
  Type & set__exp_brake(
    const double & _arg)
  {
    this->exp_brake = _arg;
    return *this;
  }
  Type & set__exp_gear(
    const int8_t & _arg)
  {
    this->exp_gear = _arg;
    return *this;
  }
  Type & set__exp_handbrake(
    const int8_t & _arg)
  {
    this->exp_handbrake = _arg;
    return *this;
  }
  Type & set__cur_steer_wheel_angle(
    const double & _arg)
  {
    this->cur_steer_wheel_angle = _arg;
    return *this;
  }
  Type & set__cur_steer_wheel_speed(
    const double & _arg)
  {
    this->cur_steer_wheel_speed = _arg;
    return *this;
  }
  Type & set__cur_fuel(
    const double & _arg)
  {
    this->cur_fuel = _arg;
    return *this;
  }
  Type & set__cur_brake(
    const double & _arg)
  {
    this->cur_brake = _arg;
    return *this;
  }
  Type & set__cur_engine_speed(
    const double & _arg)
  {
    this->cur_engine_speed = _arg;
    return *this;
  }
  Type & set__cur_gear(
    const uint8_t & _arg)
  {
    this->cur_gear = _arg;
    return *this;
  }
  Type & set__cur_handbrake(
    const uint8_t & _arg)
  {
    this->cur_handbrake = _arg;
    return *this;
  }
  Type & set__actual_gear(
    const uint8_t & _arg)
  {
    this->actual_gear = _arg;
    return *this;
  }
  Type & set__exp_auto_enable(
    const int8_t & _arg)
  {
    this->exp_auto_enable = _arg;
    return *this;
  }
  Type & set__auto_enable(
    const int8_t & _arg)
  {
    this->auto_enable = _arg;
    return *this;
  }
  Type & set__oil(
    const int8_t & _arg)
  {
    this->oil = _arg;
    return *this;
  }
  Type & set__battery_soc(
    const int8_t & _arg)
  {
    this->battery_soc = _arg;
    return *this;
  }
  Type & set__sta_code(
    const uint32_t & _arg)
  {
    this->sta_code = _arg;
    return *this;
  }
  Type & set__err_code(
    const uint32_t & _arg)
  {
    this->err_code = _arg;
    return *this;
  }
  Type & set__horn(
    const uint8_t & _arg)
  {
    this->horn = _arg;
    return *this;
  }
  Type & set__left_signal_light(
    const uint8_t & _arg)
  {
    this->left_signal_light = _arg;
    return *this;
  }
  Type & set__right_signal_light(
    const uint8_t & _arg)
  {
    this->right_signal_light = _arg;
    return *this;
  }
  Type & set__headlight(
    const uint8_t & _arg)
  {
    this->headlight = _arg;
    return *this;
  }
  Type & set__brake_light(
    const uint8_t & _arg)
  {
    this->brake_light = _arg;
    return *this;
  }
  Type & set__postion_light(
    const uint8_t & _arg)
  {
    this->postion_light = _arg;
    return *this;
  }
  Type & set__high_beam(
    const uint8_t & _arg)
  {
    this->high_beam = _arg;
    return *this;
  }
  Type & set__low_beam(
    const uint8_t & _arg)
  {
    this->low_beam = _arg;
    return *this;
  }
  Type & set__front_fog_light(
    const uint8_t & _arg)
  {
    this->front_fog_light = _arg;
    return *this;
  }
  Type & set__rear_fog_light(
    const uint8_t & _arg)
  {
    this->rear_fog_light = _arg;
    return *this;
  }
  Type & set__state_act(
    const uint8_t & _arg)
  {
    this->state_act = _arg;
    return *this;
  }
  Type & set__electrical_power_steering_availablity_status(
    const uint8_t & _arg)
  {
    this->electrical_power_steering_availablity_status = _arg;
    return *this;
  }
  Type & set__veh_spd_valid_flag(
    const uint8_t & _arg)
  {
    this->veh_spd_valid_flag = _arg;
    return *this;
  }
  Type & set__reserved(
    const std::array<uint32_t, 8> & _arg)
  {
    this->reserved = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    self_state::msg::ChassisInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::ChassisInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::ChassisInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::ChassisInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::ChassisInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::ChassisInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::ChassisInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::ChassisInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::ChassisInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::ChassisInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__ChassisInfo
    std::shared_ptr<self_state::msg::ChassisInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__ChassisInfo
    std::shared_ptr<self_state::msg::ChassisInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ChassisInfo_ & other) const
  {
    if (this->local_time != other.local_time) {
      return false;
    }
    if (this->utc_time != other.utc_time) {
      return false;
    }
    if (this->message_num != other.message_num) {
      return false;
    }
    if (this->lf_wheel_speed != other.lf_wheel_speed) {
      return false;
    }
    if (this->lr_wheel_speed != other.lr_wheel_speed) {
      return false;
    }
    if (this->rf_wheel_speed != other.rf_wheel_speed) {
      return false;
    }
    if (this->rr_wheel_speed != other.rr_wheel_speed) {
      return false;
    }
    if (this->lf_wheel_pulse_counter != other.lf_wheel_pulse_counter) {
      return false;
    }
    if (this->lr_wheel_pulse_counter != other.lr_wheel_pulse_counter) {
      return false;
    }
    if (this->rf_wheel_pulse_counter != other.rf_wheel_pulse_counter) {
      return false;
    }
    if (this->rr_wheel_pulse_counter != other.rr_wheel_pulse_counter) {
      return false;
    }
    if (this->lon_acc != other.lon_acc) {
      return false;
    }
    if (this->lat_acc != other.lat_acc) {
      return false;
    }
    if (this->yaw_rate != other.yaw_rate) {
      return false;
    }
    if (this->cur_spd != other.cur_spd) {
      return false;
    }
    if (this->exp_speed != other.exp_speed) {
      return false;
    }
    if (this->exp_acceleration != other.exp_acceleration) {
      return false;
    }
    if (this->exp_yaw_rate != other.exp_yaw_rate) {
      return false;
    }
    if (this->exp_steer_wheel_angle != other.exp_steer_wheel_angle) {
      return false;
    }
    if (this->exp_steer_wheel_speed != other.exp_steer_wheel_speed) {
      return false;
    }
    if (this->exp_fuel != other.exp_fuel) {
      return false;
    }
    if (this->exp_brake != other.exp_brake) {
      return false;
    }
    if (this->exp_gear != other.exp_gear) {
      return false;
    }
    if (this->exp_handbrake != other.exp_handbrake) {
      return false;
    }
    if (this->cur_steer_wheel_angle != other.cur_steer_wheel_angle) {
      return false;
    }
    if (this->cur_steer_wheel_speed != other.cur_steer_wheel_speed) {
      return false;
    }
    if (this->cur_fuel != other.cur_fuel) {
      return false;
    }
    if (this->cur_brake != other.cur_brake) {
      return false;
    }
    if (this->cur_engine_speed != other.cur_engine_speed) {
      return false;
    }
    if (this->cur_gear != other.cur_gear) {
      return false;
    }
    if (this->cur_handbrake != other.cur_handbrake) {
      return false;
    }
    if (this->actual_gear != other.actual_gear) {
      return false;
    }
    if (this->exp_auto_enable != other.exp_auto_enable) {
      return false;
    }
    if (this->auto_enable != other.auto_enable) {
      return false;
    }
    if (this->oil != other.oil) {
      return false;
    }
    if (this->battery_soc != other.battery_soc) {
      return false;
    }
    if (this->sta_code != other.sta_code) {
      return false;
    }
    if (this->err_code != other.err_code) {
      return false;
    }
    if (this->horn != other.horn) {
      return false;
    }
    if (this->left_signal_light != other.left_signal_light) {
      return false;
    }
    if (this->right_signal_light != other.right_signal_light) {
      return false;
    }
    if (this->headlight != other.headlight) {
      return false;
    }
    if (this->brake_light != other.brake_light) {
      return false;
    }
    if (this->postion_light != other.postion_light) {
      return false;
    }
    if (this->high_beam != other.high_beam) {
      return false;
    }
    if (this->low_beam != other.low_beam) {
      return false;
    }
    if (this->front_fog_light != other.front_fog_light) {
      return false;
    }
    if (this->rear_fog_light != other.rear_fog_light) {
      return false;
    }
    if (this->state_act != other.state_act) {
      return false;
    }
    if (this->electrical_power_steering_availablity_status != other.electrical_power_steering_availablity_status) {
      return false;
    }
    if (this->veh_spd_valid_flag != other.veh_spd_valid_flag) {
      return false;
    }
    if (this->reserved != other.reserved) {
      return false;
    }
    return true;
  }
  bool operator!=(const ChassisInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ChassisInfo_

// alias to use template instance with default allocator
using ChassisInfo =
  self_state::msg::ChassisInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__CHASSIS_INFO__STRUCT_HPP_
