#ifndef DATA_CONTAINER_H
#define DATA_CONTAINER_H

#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

namespace jojo {
namespace tools {

class DataContainerBase {
 public:
  DataContainerBase()          = default;
  virtual ~DataContainerBase() = default;

  // 泛型接口
  virtual void insert(uint64_t t, const void* data)      = 0;
  virtual uint64_t init_ts(const uint64_t& start_time)   = 0;
  virtual uint64_t align_ts(const uint64_t& target_time) = 0;

  virtual void reset_iter() = 0;
  virtual void next()       = 0;
  virtual bool is_end()     = 0;
  virtual void stop()       = 0;

  virtual void update(uint64_t t, const void* data) = 0;
  virtual void publish()                            = 0;

  // 新增虚函数，获取当前数据
  virtual void GetCurData(void* out) const     = 0;
  virtual void GetCurTime(uint64_t& out) const = 0;
  virtual const void* GetCurDataPtr() const    = 0;

  virtual void GetAllTimeStamp(std::vector<uint64_t>& ts) const = 0;

  virtual bool empty() const       = 0;
  virtual std::size_t size() const = 0;

  bool first_run = false;
};

template <typename T>
class DataContainer : public DataContainerBase {
 public:
  using TimeType      = uint64_t;
  using DataMap       = std::map<TimeType, T>;
  using Iterator      = typename DataMap::iterator;
  using ConstIterator = typename DataMap::const_iterator;

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
      // map.insert(std::make_pair(t, *real_data));
      map.emplace(t, *real_data);
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

        cur_time = it->first;
        cur_data = it->second;
      }
    } else {
      std::cerr << "time " << t << " not found in DataContainer" << std::endl;
    }
  }

  // 用于主数据的时间校准
  uint64_t init_ts(const uint64_t& start_time) override {
    // 返回首个 >= start_time 的节点时间。
    iter = map.lower_bound(start_time);
    if (iter == map.end()) {
      cur_time = 0;
      cur_data = T{};
      return 0;
    }

    cur_time = iter->first;
    cur_data = iter->second;
    return cur_time;
  }

  // 用于其他数据向主数据对齐：使用最近邻时间
  uint64_t align_ts(const uint64_t& target_time) override {
    if (map.empty()) {
      iter     = map.end();
      cur_time = 0;
      cur_data = T{};
      return 0;
    }

    auto next_it = map.lower_bound(target_time);
    if (next_it == map.begin()) {
      iter = next_it;
    } else if (next_it == map.end()) {
      iter = std::prev(map.end());
    } else {
      auto prev_it = std::prev(next_it);
      iter = (next_it->first - target_time < target_time - prev_it->first)
                 ? next_it
                 : prev_it;
    }

    cur_time  = iter->first;
    cur_data  = iter->second;
    first_run = true;
    return cur_time;
  }

  void reset_iter() override {
    iter = map.begin();
    if (iter != map.end()) {
      cur_time = iter->first;
      cur_data = iter->second;
    }
  }

  void next() override {
    /*
    if (iter != map.end()) {
      ++iter;
      cur_time = iter->first;
      cur_data = iter->second;
    }
    */
    // /*
    if (iter == map.end()) return;
    ++iter;
    if (iter != map.end()) {
      cur_time = iter->first;
      cur_data = iter->second;
    }
    // */
  }

  void prev() {
    if (iter != map.begin()) {
      --iter;
      cur_time = iter->first;
      cur_data = iter->second;
    }
  }

  bool is_end() override {
    if (iter == map.end()) {
      std::cout << name << " timestamp data has ended." << std::endl;
      return true;
    }
    return false;
  }

  void stop() override {}

  void publish() override {}

  void GetCurData(void* out) const override {
    // 这里是拷贝，不是引用
    *static_cast<T*>(out) = cur_data;
  }

  const void* GetCurDataPtr() const override { return &cur_data; }

  void GetCurTime(uint64_t& out) const override { out = cur_time; }

  std::string name  = "";
  TimeType cur_time = 0;
  T cur_data;

  void GetAllTimeStamp(std::vector<uint64_t>& ts) const override {
    ts.clear();
    ts.reserve(map.size());

    for (const auto& kv : map) {
      ts.push_back(kv.first);
    }
  }

  void GetDataSegment(uint64_t start_time, uint64_t end_time,
                      std::deque<T>& out_data);

  const DataMap& data() const { return map; }
  ConstIterator begin() const { return map.begin(); }
  ConstIterator end() const { return map.end(); }

  bool empty() const override { return map.empty(); }
  std::size_t size() const override { return map.size(); }

 protected:
  DataMap map;

 private:
  Iterator iter;

  ConstIterator win_begin;
  ConstIterator win_end;
  bool win_inited = false;
};

/*
  time_map: 95, 105, 115, ..., 195, 205
  find time: start = 100 ; end = 200
  output: 95, 105, 115, ..., 195, 205
*/
template <typename T>
void DataContainer<T>::GetDataSegment(uint64_t start_time, uint64_t end_time,
                                      std::deque<T>& out_data) {
  out_data.clear();
  // out_time.clear();

  if (map.empty()) return;

  /* 离线查询
  // ---- 1. 找到 >= start_time 的第一个 ----
  auto it_start = map.lower_bound(start_time);

  // ---- 2. 向前取一帧（保证早于 start_time）----
  if (it_start != map.begin()) {
    --it_start;
  }

  // ---- 3. 找到 > end_time 的位置 ----
  auto it_end = map.upper_bound(end_time);

  // ---- 4. 遍历区间 [it_start, it_end)
  for (auto it = it_start; it != it_end; ++it) {
    // out_time.push_back(it->first);
    out_data.push_back(it->second);
  }

  // ---- 5. 再补一帧（保证晚于 end_time）----
  if (it_end != map.end()) {
    // out_time.push_back(it_end->first);
    out_data.push_back(it_end->second);
  }
  */

  // /* 流式处理（滑动窗口）
  // 初始化（只做一次）
  if (!win_inited) {
    win_begin = map.lower_bound(start_time);
    if (win_begin != map.begin()) --win_begin;

    win_end = win_begin;
    while (win_end != map.end() && win_end->first <= end_time) {
      ++win_end;
    }

    win_inited = true;
  }

  // 向前滑动 begin
  while (win_begin != map.end()) {
    auto next = win_begin;
    ++next;
    if (next == map.end()) break;

    // 只要下一帧还没超过 start_time，移动
    if (next->first <= start_time) {
      ++win_begin;
    } else {
      break;
    }
  }

  // 向后滑动 end
  while (win_end != map.end() && win_end->first <= end_time) {
    ++win_end;
  }

  // 输出数据
  auto it = win_begin;

  // 中间段
  for (; it != win_end && it != map.end(); ++it) {
    // out_time.push_back(it->first);
    out_data.push_back(it->second);
  }

  // 保证包含 end 后一帧
  if (win_end != map.end()) {
    // out_time.push_back(win_end->first);
    out_data.push_back(win_end->second);
  }
  // */
}

}  // namespace tools
}  // namespace jojo

#endif
