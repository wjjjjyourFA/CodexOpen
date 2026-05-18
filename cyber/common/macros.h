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

#ifndef CYBER_COMMON_MACROS_H_
#define CYBER_COMMON_MACROS_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>

#include "cyber/base/macros.h"

// 宏定义创建了一个模板 HasShutdown<T>
DEFINE_TYPE_TRAIT(HasShutdown, Shutdown)

// 当类型 T 具有 Shutdown 成员函数时，调用其 Shutdown 方法
template <typename T>
typename std::enable_if<HasShutdown<T>::value>::type CallShutdown(T *instance) {
  instance->Shutdown();
}

// 当类型 T 不具有 Shutdown 成员函数时，不执行任何操作
template <typename T>
typename std::enable_if<!HasShutdown<T>::value>::type CallShutdown(
    T *instance) {
  (void)instance;
}

// There must be many copy-paste versions of these macros which are same
// things, undefine them to avoid conflict.
#undef UNUSED
#undef DISALLOW_COPY_AND_ASSIGN

// 这个宏用来显式地标记一个参数未被使用，以避免编译器产生未使用参数的警告
// 通过强制将参数转换为 void，编译器认为该参数已经处理，因此不会警告它未使用
#define UNUSED(param) (void)param

// 禁止拷贝构造和拷贝赋值操作符，确保类 classname 不会被拷贝
#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

// 单例模式，使得类 classname 只能有一个实例
// 单例模式（Singleton Pattern） 的核心思想是：一个类只能有一个实例，而且这个实例在整个程序中是唯一的。
// 通过单例模式，你可以确保类的实例在程序的整个生命周期中只有一个，并且可以全局访问这个实例
// 单例模式 是面向 进程 的，意味着它保证每个进程内某个类只有一个实例，但它不跨进程工作
// 多程序间的数据共享需要使用进程间通信（IPC）机制，如消息队列、共享内存、信号量等 ==> DDS
#define DECLARE_SINGLETON(classname)                                           \
 public:                                                                       \
  static classname *Instance(bool create_if_needed = true) {                   \
    static classname *instance = nullptr;                                      \
    if (!instance && create_if_needed) {                                       \
      static std::once_flag flag;                                              \
      std::call_once(flag,                                                     \
                     [&] { instance = new (std::nothrow) classname(); });      \
    }                                                                          \
    return instance;                                                           \
  }                                                                            \
                                                                               \
  static void CleanUp() {                                                      \
    auto instance = Instance(false);                                           \
    if (instance != nullptr) {                                                 \
      CallShutdown(instance);                                                  \
    }                                                                          \
  }                                                                            \
                                                                               \
 private:                                                                      \
  classname();                                                                 \
  DISALLOW_COPY_AND_ASSIGN(classname);

#endif  // CYBER_COMMON_MACROS_H_
