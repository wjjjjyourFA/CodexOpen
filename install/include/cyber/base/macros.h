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

#ifndef CYBER_BASE_MACROS_H_
#define CYBER_BASE_MACROS_H_

#include <cstdlib>
#include <new>

#ifndef eprosima
// 在预处理阶段，把源码中所有出现的 eprosima 这个标识符，统统替换成 eprosima_wrap
#define eprosima eprosima_wrap
#endif
// **** 编译时的优化工具 编译器自动处理 **** //

// 宏 cyber_likely 和 cyber_unlikely 用于帮助编译器优化分支预测，提高 CPU 执行效率
#if __GNUC__ >= 3
#define cyber_likely(x) (__builtin_expect((x), 1))
#define cyber_unlikely(x) (__builtin_expect((x), 0))
#else
#define cyber_likely(x) (x)
#define cyber_unlikely(x) (x)
#endif

// 缓存行大小（通常为 64 字节），用于优化结构体和变量的缓存对齐，避免伪共享问题
#define CACHELINE_SIZE 64

// **** 执行结果可以在编译时确定，从而优化运行时行为 **** //

// 用于检测类型 T 是否具有特定的成员函数 func
#define DEFINE_TYPE_TRAIT(name, func)                     \
  template <typename T>                                   \
  struct name {                                           \
    template <typename Class>                             \
    static constexpr bool Test(decltype(&Class::func)*) { \
      return true;                                        \
    }                                                     \
    template <typename>                                   \
    static constexpr bool Test(...) {                     \
      return false;                                       \
    }                                                     \
                                                          \
    static constexpr bool value = Test<T>(nullptr);       \
  };                                                      \
                                                          \
  template <typename T>                                   \
  constexpr bool name<T>::value;

// 用于在多核环境下让 CPU 暂时进入低功耗模式，减少竞争资源的消耗
inline void cpu_relax() {
#if defined(__aarch64__)
  asm volatile("yield" ::: "memory");
#else
  asm volatile("rep; nop" ::: "memory");
#endif
}

// **** 运行时的安全性优化工具 主动在业务代码中显式调用 **** //

// 使用标准的 malloc 分配指定大小的内存，并在内存分配失败时抛出异常
inline void* CheckedMalloc(size_t size) {
  void* ptr = std::malloc(size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

// 使用标准的 calloc 分配指定数量的内存块，并初始化为 0，并在内存分配失败时抛出异常
inline void* CheckedCalloc(size_t num, size_t size) {
  void* ptr = std::calloc(num, size);
  if (!ptr) {
    throw std::bad_alloc();
  }
  return ptr;
}

#endif  // CYBER_BASE_MACROS_H_
