## `.so`动态库 无法连接 `.a` 静态库

```sh
cmake .. \
  -DCMAKE_INSTALL_PREFIX=${installDir} \
  -DBUILD_PYTHON=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build . -j20
cmake --install .
```

### 原因解析

- 你正在构建一个 **共享库（.so）**；
- 它依赖一个 **静态库（.a）**；
- 但是这个 `.a` 静态库中的对象文件（`.o`）并没有用 `-fPIC` 编译；
- 所以不能安全地被嵌入 `.so` 中；
- **`-fPIC`** 是 Position Independent Code，所有 `.so` 中的代码都必须是可重定位的。

## `GlobalData` 初始化失败

```sh
# 配置系统环境
source setup.bash 
```
**CyberRT 框架在加载时自动调用配置加载流程**。
也就是说：
> **即使你没有写任何 `apollo::cyber::Init()`，只要你链接了 cyber 相关的库，它内部就可能通过静态单例 `GlobalData` 自动初始化。**

### 原因分析

这是 `CyberRT` 中的 **懒汉式单例初始化触发副作用**：
```c++
GlobalData::Instance();
```

只要某些库比如 `cyber/time`, `cyber/scheduler`, `cyber/common` 中静态变量用到了 `GlobalData`，即便你没写 `Init()`，`GlobalData()` 也会自动初始化：
```c++
apollo::cyber::common::GlobalData::GlobalData() {
  InitConfig(); // ← 这里就是出错点
}
```

#### 错误根因

错误核心是：
```
[example_talker]File [/apollo/cyber/conf/cyber.pb.conf] does not exist!
```

Apollo 期望默认配置文件为：
```
/apollo/cyber/conf/cyber.pb.conf
```

但你实际运行路径是：
```
/media/jojo/AQiDePan/CodexOpenNew/build/cyber/examples/example_talker
```

它找不到默认配置文件就直接报错了。

------

### 解决方法

#### 方法1：指定配置路径

你可以运行程序时用环境变量或命令行参数显式指定配置路径，例如：
```sh
export CYBER_CONF="/path/CodexOpen/install/conf/cyber.pb.conf"
```

或者运行程序前设置：
```sh
export CYBER_PATH="/path/CodexOpen"
```

Apollo Cyber 会根据 `$CYBER_PATH/cyber/conf/cyber.pb.conf` 来找默认配置。
