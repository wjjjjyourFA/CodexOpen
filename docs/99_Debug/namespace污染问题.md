#### ✅ **关键点：using namespace vs using alias（命名别名）**

有两类 using，你一定要区分：

------

##### **① using namespace X;   ← 这是“全部导入”，会污染**

把命名空间 X 里的所有名字全部导入当前作用域，就像：

```c++
using namespace std;
```

然后当前文件会被 std 下所有名字污染。

**只要写了：**

```c++
using namespace apollo::perception::base;
```

就等价于把 base 下面的几百个 class 都“复制”到当前命名空间作用域中。
 **后续引用这个头文件的地方，也会被污染**（如果写在头文件里）。

------

##### **② using X = Y;  ← 这是命名别名（type alias），不会污染**

例如 Apollo 常用：

```c++
using base::Object;
using base = apollo::perception::base;
```

这些都是 *别名*（alias）：

- **只是给一个类型或命名空间起别名**
- **不会把里面的类型导入当前作用域**
- **不会污染其他文件**

------

#### ❗你引用 Apollo 的误区

你看到 Apollo 的代码：

```c++
namespace camera {
using namespace apollo::perception::base;
}
```

实际上 **Apollo 几乎从不这么写在头文件**。
 他们写在 `.cc` 文件里是安全的。

但如果你把它写在 **头文件 (.hpp)** 中：

###### ✔ 写在类/命名空间的 *源文件(.cpp)* 中 → 不会污染外部

###### ❌ 写在头文件中 → 任何 `#include` 这个头的文件都会被污染

#### 📌 回到你的情况

你写的是：

```c++
namespace jojo {
namespace perception {
using namespace jojo::cyber::parameter;
```

✔ 这会把 `jojo::cyber::parameter` 的所有符号都引入当前命名空间
 ✔ 后续包含这个头文件的任何 cpp，都能直接访问这些符号
 ❌ **这确实会污染**

------

#### 🔧 **所以正确的改法：使用命名别名，而不是 using namespace**

##### ❌ 错误（污染）

```c++
using namespace jojo::cyber::parameter;
```

##### ✅ 正确（不污染）

```c++
namespace param = jojo::cyber::parameter;
```

然后类继承写：

```c++
class ParamSimple : public param::SimpleParamBase
```

完全 OK。
