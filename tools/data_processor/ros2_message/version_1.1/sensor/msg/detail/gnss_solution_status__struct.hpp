// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor:msg/GnssSolutionStatus.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__STRUCT_HPP_
#define SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__sensor__msg__GnssSolutionStatus __attribute__((deprecated))
#else
# define DEPRECATED__sensor__msg__GnssSolutionStatus __declspec(deprecated)
#endif

namespace sensor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GnssSolutionStatus_
{
  using Type = GnssSolutionStatus_<ContainerAllocator>;

  explicit GnssSolutionStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sol_status = 0;
    }
  }

  explicit GnssSolutionStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sol_status = 0;
    }
  }

  // field types and members
  using _sol_status_type =
    int8_t;
  _sol_status_type sol_status;

  // setters for named parameter idiom
  Type & set__sol_status(
    const int8_t & _arg)
  {
    this->sol_status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t SOL_COMPUTED =
    0;
  static constexpr int8_t INSUFFICIENT_OBS =
    1;
  static constexpr int8_t NO_CONVERGENCE =
    2;
  static constexpr int8_t SINGULARITY =
    3;
  static constexpr int8_t COV_TRACE =
    4;
  static constexpr int8_t TEST_DIST =
    5;
  static constexpr int8_t COLD_START =
    6;
  static constexpr int8_t V_H_LIMIT =
    7;
  static constexpr int8_t VARIANCE =
    8;
  static constexpr int8_t RESIDUALS =
    9;
  static constexpr int8_t PENDING =
    18;
  static constexpr int8_t INVALID_FIX =
    19;

  // pointer types
  using RawPtr =
    sensor::msg::GnssSolutionStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor::msg::GnssSolutionStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor::msg::GnssSolutionStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor::msg::GnssSolutionStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor__msg__GnssSolutionStatus
    std::shared_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor__msg__GnssSolutionStatus
    std::shared_ptr<sensor::msg::GnssSolutionStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GnssSolutionStatus_ & other) const
  {
    if (this->sol_status != other.sol_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const GnssSolutionStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GnssSolutionStatus_

// alias to use template instance with default allocator
using GnssSolutionStatus =
  sensor::msg::GnssSolutionStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::SOL_COMPUTED;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::INSUFFICIENT_OBS;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::NO_CONVERGENCE;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::SINGULARITY;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::COV_TRACE;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::TEST_DIST;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::COLD_START;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::V_H_LIMIT;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::VARIANCE;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::RESIDUALS;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::PENDING;
template<typename ContainerAllocator>
constexpr int8_t GnssSolutionStatus_<ContainerAllocator>::INVALID_FIX;

}  // namespace msg

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__GNSS_SOLUTION_STATUS__STRUCT_HPP_
