/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef CYBER_BASE_RW_LOCK_GUARD_H_
#define CYBER_BASE_RW_LOCK_GUARD_H_

#include <unistd.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <thread>

namespace apollo {
namespace cyber {
namespace base {
  
// 自动管理锁的生命周期，避免手动加锁、解锁出错。
// 防止死锁，当作用域结束（无论是否发生异常），都会自动解锁
template <typename RWLock>
class ReadLockGuard {
 public:
  // 构造时 rw_lock_.ReadLock(); 自动加读锁
  explicit ReadLockGuard(RWLock& lock) : rw_lock_(lock) { rw_lock_.ReadLock(); }
  // 对象销毁时，rw_lock_.ReadUnlock();，自动释放读锁
  ~ReadLockGuard() { rw_lock_.ReadUnlock(); }

 private:
  ReadLockGuard(const ReadLockGuard& other) = delete;
  ReadLockGuard& operator=(const ReadLockGuard& other) = delete;
  RWLock& rw_lock_;
};

template <typename RWLock>
class WriteLockGuard {
 public:
  explicit WriteLockGuard(RWLock& lock) : rw_lock_(lock) {
    rw_lock_.WriteLock();
  }

  ~WriteLockGuard() { rw_lock_.WriteUnlock(); }

 private:
  WriteLockGuard(const WriteLockGuard& other) = delete;
  WriteLockGuard& operator=(const WriteLockGuard& other) = delete;
  RWLock& rw_lock_;
};

}  // namespace base
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_BASE_RW_LOCK_GUARD_H_
