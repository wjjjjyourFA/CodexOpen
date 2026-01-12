#ifndef C_PARAMETER_PB_H_
#define C_PARAMETER_PB_H_

#include <string>
#include <type_traits>
#include <iostream>
#include <variant>

namespace jojo {
namespace common {
namespace config {

enum ParamType {
  NOT_SET = 0,
  BOOL = 1,
  INT = 2,
  DOUBLE = 3,
  STRING = 4
};

class Param {
 public:
  // 构造函数
  Param() : type_(NOT_SET) {}

  void set_name(const std::string& name) { this->name_ = name; }

  void set_type(ParamType type) { this->type_ = type; }

  void set_type_name(const std::string& type_name) { this->type_name_ = type_name; }

  // 设置不同类型的值
  void set_bool_value(bool value) {
    type_ = BOOL;
    this->value_ = value;
  }

  void set_int_value(int64_t value) {
    type_ = INT;
    this->value_ = value;
  }

  void set_double_value(double value) {
    type_ = DOUBLE;
    this->value_ = value;
  }

  void set_string_value(std::string value) {
    type_ = STRING;
    this->value_ = value;
  }

  // 获取不同类型的值
  std::string get_name() const { return name_; }

  ParamType get_type() const { return type_; }

  std::string get_type_name() const { return type_name_; }

  bool get_bool_value() const {
    if (type_ == BOOL) {
      return std::get<bool>(value_);
    }
    throw std::runtime_error("Incorrect type");
  }

  int64_t get_int_value() const {
    if (type_ == INT) {
      return std::get<int64_t>(value_);
    }
    throw std::runtime_error("Incorrect type");
  }

  double get_double_value() const {
    if (type_ == DOUBLE) {
      return std::get<double>(value_);
    }
    throw std::runtime_error("Incorrect type");
  }

  const std::string& get_string_value() const {
    if (type_ == STRING) {
      return std::get<std::string>(value_);
    }
    throw std::runtime_error("Incorrect type");
  }

  void CopyFrom(const Param& other) {
    name_ = other.name_;
    type_ = other.type_;
    type_name_ = other.type_name_;
    value_ = other.value_;
  }

 private:
  std::string name_;
  ParamType type_;
  std::string type_name_;
  std::variant<bool, int64_t, double, std::string> value_;  // oneof_value
};

} // namespace config
} // namespace common
} // namespace jojo

#endif /* C_PARAMETER_PB_H_ */