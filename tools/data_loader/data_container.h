#ifndef DATA_CONTAINER_HH
#define DATA_CONTAINER_HH

#pragma once

#include <boost/unordered/unordered_map.hpp>

namespace jojo {
namespace tools {

class DataContainerBase {
 public:
  DataContainerBase()  = default;
  ~DataContainerBase() = default;

  // 泛型接口
  virtual void insert(uint64_t t, const void* data)      = 0;
  virtual uint64_t init_ts(const uint64_t& start_time)   = 0;
  virtual uint64_t align_ts(const uint64_t& target_time) = 0;

  virtual void begin() = 0;
  virtual void next()  = 0;
  virtual bool end()   = 0;
  virtual void stop()  = 0;

  virtual void update(uint64_t t, const void* data) = 0;
  virtual void publish()                            = 0;

  // 新增虚函数，获取当前数据
  virtual void GetCurData(void* out) const     = 0;
  virtual void GetCurTime(uint64_t& out) const = 0;
};

template <typename T>
class DataContainer : public DataContainerBase {
 public:
  using TimeType = uint64_t;
  using DataMap  = std::map<TimeType, T>;
  using Iterator = typename DataMap::iterator;

  DataContainer()          = default;
  virtual ~DataContainer() = default;

  void set_name(std::string ss) { name = ss; }

  // 逐条插入数据
  void insert(TimeType t, const void* data) override {
    if (data == nullptr) {
      // 插入默认值
      map.insert(std::make_pair(t, T{}));
    } else {
      const T* real_data = static_cast<const T*>(data);
      map.insert(std::make_pair(t, *real_data));
    }
  }

  void update(TimeType t, const void* data) override {
    auto it = map.find(t);
    if (it != map.end()) {
      if (data == nullptr) {
        it->second = T{};
        std::cout << name << " update with default value" << std::endl;
      } else {
        const T* real_data = static_cast<const T*>(data);
        it->second         = *real_data;

        cur_time = iter->first;
        cur_data = iter->second;
      }
    } else {
      std::cerr << "time " << t << " not found in DataContainer" << std::endl;
    }
  }

  // 用于主数据的时间校准
  uint64_t init_ts(const uint64_t& start_time) override {
    this->begin();

    uint64_t inited_time = iter->first;
    // 获得 大于起始点 的时刻
    while (!this->end() && start_time > cur_time) {
      inited_time = this->cur_time;
      this->next();
    }
    // std::cout << name << " start time inited : " << cur_time << std::endl;

    return inited_time;
  }

  // 用于其他数据向主数据对齐
  uint64_t align_ts(const uint64_t& target_time) override {
    // 后续再查找时间戳，不用再从头开始
    static bool first_run = false;
    if (!first_run) {
      this->begin();
      first_run = true;
    }

    uint64_t best_time = iter->first;
    // 获得 最接近 的时刻
    while (!this->end()) {
      uint64_t cur = this->cur_time;

      // 如果已经找到第一个大于 target_time 的点
      if (cur > target_time) {
        uint64_t prev_time = best_time;
        uint64_t next_time = cur;

        // 选取距离 target_time 最近的
        if ((next_time - target_time) < (target_time - prev_time)) {
          best_time = next_time;
        } else {
          best_time = prev_time;
          this->prev();
        }
        break;
      } else if (cur == target_time) {
        best_time = cur;
        break;
      }

      best_time = cur;
      this->next();
    }

    // std::cout << name << " target_time : " << target_time << std::endl;
    // std::cout << name << " align time inited : " << cur_time << std::endl;

    return best_time;
  }

  void begin() override {
    iter     = map.begin();
    cur_time = iter->first;
    cur_data = iter->second;
  }

  void next() override {
    if (iter != map.end()) {
      ++iter;
      cur_time = iter->first;
      cur_data = iter->second;
    }
  }

  void prev() {
    if (iter != map.begin()) {
      --iter;
      cur_time = iter->first;
      cur_data = iter->second;
    }
  }

  bool end() override {
    if (iter == map.end()) {
      std::cout << name << " timestamp data has ended." << std::endl;
      return true;
    }
    return false;
  }

  void stop() override {}

  void publish() override {}

  void GetCurData(void* out) const override {
    *static_cast<T*>(out) = cur_data;
  }

  void GetCurTime(uint64_t& out) const override { out = cur_time; }

  std::string name  = "";
  TimeType cur_time = 0;
  T cur_data;

 private:
  DataMap map;
  Iterator iter;
};

}  // namespace tools
}  // namespace jojo

#endif
