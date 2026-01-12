// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sensor:msg/PosType.idl
// generated code does not contain a copyright notice

#ifndef SENSOR__MSG__DETAIL__POS_TYPE__STRUCT_HPP_
#define SENSOR__MSG__DETAIL__POS_TYPE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__sensor__msg__PosType __attribute__((deprecated))
#else
# define DEPRECATED__sensor__msg__PosType __declspec(deprecated)
#endif

namespace sensor
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PosType_
{
  using Type = PosType_<ContainerAllocator>;

  explicit PosType_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_type = 0;
    }
  }

  explicit PosType_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pos_type = 0;
    }
  }

  // field types and members
  using _pos_type_type =
    int8_t;
  _pos_type_type pos_type;

  // setters for named parameter idiom
  Type & set__pos_type(
    const int8_t & _arg)
  {
    this->pos_type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t NONE =
    0;
  static constexpr int8_t FIXEDPOS =
    1;
  static constexpr int8_t FIXEDHEIGHT =
    2;
  static constexpr int8_t FLOATCONV =
    4;
  static constexpr int8_t WIDELANE =
    5;
  static constexpr int8_t NARROWLANE =
    6;
  static constexpr int8_t DOPPLER_VELOCITY =
    8;
  static constexpr int8_t SINGLE =
    16;
  static constexpr int8_t PSRDIFF =
    17;
  static constexpr int8_t WAAS =
    18;
  static constexpr int8_t PROPOGATED =
    19;
  static constexpr int8_t OMNISTAR =
    20;
  static constexpr int8_t L1_FLOAT =
    32;
  static constexpr int8_t IONOFREE_FLOAT =
    33;
  static constexpr int8_t NARROW_FLOAT =
    34;
  static constexpr int8_t L1_INT =
    48;
  static constexpr int8_t WIDE_INT =
    49;
  static constexpr int8_t NARROW_INT =
    50;
  static constexpr int8_t RTK_DIRECT_INS =
    51;
  static constexpr int8_t INS_SBAS =
    52;
  static constexpr int8_t INS_PSRSP =
    53;
  static constexpr int8_t INS_PSRDIFF =
    54;
  static constexpr int8_t INS_RTKFLOAT =
    55;
  static constexpr int8_t INS_RTKFIXED =
    56;
  static constexpr int8_t INS_OMNISTAR =
    57;
  static constexpr int8_t INS_OMNISTAR_HP =
    58;
  static constexpr int8_t INS_OMNISTAR_XP =
    59;
  static constexpr int8_t OMNISTAR_HP =
    64;
  static constexpr int8_t OMNISTAR_XP =
    65;
  static constexpr int8_t PPP_CONVERGING =
    68;
  static constexpr int8_t PPP =
    69;
  static constexpr int8_t INS_PPP_CONVERGING =
    73;
  static constexpr int8_t INS_PPP =
    74;

  // pointer types
  using RawPtr =
    sensor::msg::PosType_<ContainerAllocator> *;
  using ConstRawPtr =
    const sensor::msg::PosType_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sensor::msg::PosType_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sensor::msg::PosType_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sensor::msg::PosType_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::PosType_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sensor::msg::PosType_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sensor::msg::PosType_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sensor::msg::PosType_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sensor::msg::PosType_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sensor__msg__PosType
    std::shared_ptr<sensor::msg::PosType_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sensor__msg__PosType
    std::shared_ptr<sensor::msg::PosType_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PosType_ & other) const
  {
    if (this->pos_type != other.pos_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const PosType_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PosType_

// alias to use template instance with default allocator
using PosType =
  sensor::msg::PosType_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::NONE;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::FIXEDPOS;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::FIXEDHEIGHT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::FLOATCONV;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::WIDELANE;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::NARROWLANE;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::DOPPLER_VELOCITY;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::SINGLE;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::PSRDIFF;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::WAAS;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::PROPOGATED;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::OMNISTAR;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::L1_FLOAT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::IONOFREE_FLOAT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::NARROW_FLOAT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::L1_INT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::WIDE_INT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::NARROW_INT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::RTK_DIRECT_INS;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_SBAS;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_PSRSP;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_PSRDIFF;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_RTKFLOAT;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_RTKFIXED;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_OMNISTAR;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_OMNISTAR_HP;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_OMNISTAR_XP;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::OMNISTAR_HP;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::OMNISTAR_XP;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::PPP_CONVERGING;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::PPP;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_PPP_CONVERGING;
template<typename ContainerAllocator>
constexpr int8_t PosType_<ContainerAllocator>::INS_PPP;

}  // namespace msg

}  // namespace sensor

#endif  // SENSOR__MSG__DETAIL__POS_TYPE__STRUCT_HPP_
