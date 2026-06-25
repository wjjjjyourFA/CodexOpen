// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from self_state:msg/InsStatus.idl
// generated code does not contain a copyright notice

#ifndef SELF_STATE__MSG__DETAIL__INS_STATUS__STRUCT_HPP_
#define SELF_STATE__MSG__DETAIL__INS_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__self_state__msg__InsStatus __attribute__((deprecated))
#else
# define DEPRECATED__self_state__msg__InsStatus __declspec(deprecated)
#endif

namespace self_state
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct InsStatus_
{
  using Type = InsStatus_<ContainerAllocator>;

  explicit InsStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ins_status = 0;
    }
  }

  explicit InsStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ins_status = 0;
    }
  }

  // field types and members
  using _ins_status_type =
    int8_t;
  _ins_status_type ins_status;

  // setters for named parameter idiom
  Type & set__ins_status(
    const int8_t & _arg)
  {
    this->ins_status = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t INS_INACTIVE =
    0;
  static constexpr int8_t INS_ALIGNING =
    1;
  static constexpr int8_t INS_HIGH_VARIANCE =
    2;
  static constexpr int8_t INS_SOLUTION_GOOD =
    3;
  static constexpr int8_t INS_SOLUTION_FREE =
    6;
  static constexpr int8_t INS_ALIGNMENT_COMPLETE =
    7;
  static constexpr int8_t DETERMINING_ORIENTATION =
    8;
  static constexpr int8_t WAITING_INITIALPOS =
    9;

  // pointer types
  using RawPtr =
    self_state::msg::InsStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const self_state::msg::InsStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<self_state::msg::InsStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<self_state::msg::InsStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      self_state::msg::InsStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::InsStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      self_state::msg::InsStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<self_state::msg::InsStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<self_state::msg::InsStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<self_state::msg::InsStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__self_state__msg__InsStatus
    std::shared_ptr<self_state::msg::InsStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__self_state__msg__InsStatus
    std::shared_ptr<self_state::msg::InsStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const InsStatus_ & other) const
  {
    if (this->ins_status != other.ins_status) {
      return false;
    }
    return true;
  }
  bool operator!=(const InsStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct InsStatus_

// alias to use template instance with default allocator
using InsStatus =
  self_state::msg::InsStatus_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::INS_INACTIVE;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::INS_ALIGNING;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::INS_HIGH_VARIANCE;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::INS_SOLUTION_GOOD;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::INS_SOLUTION_FREE;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::INS_ALIGNMENT_COMPLETE;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::DETERMINING_ORIENTATION;
template<typename ContainerAllocator>
constexpr int8_t InsStatus_<ContainerAllocator>::WAITING_INITIALPOS;

}  // namespace msg

}  // namespace self_state

#endif  // SELF_STATE__MSG__DETAIL__INS_STATUS__STRUCT_HPP_
