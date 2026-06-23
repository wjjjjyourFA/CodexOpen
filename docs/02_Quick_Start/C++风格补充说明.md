## 代码风格

遵循 [C++风格Google](./C++风格Google.md)

## 快速记忆

- 文件命名 -- 小写字母 + 下划线分隔
```
test_a.cpp
```

- 类型命名 -- PascalCase
```
class UrlTable {}
```

- 结构体命名 -- PascalCase
```
struct UrlTableProperties {}
```

- 函数命名 -- PascalCase
```
void GenerateMap() {}
```

- 成员变量 -- snake_case 小写字母 + 下划线分隔
```
class UrlTable {
    int a;
    bool b_test;
}
```
- 私有成员变量 -- 小写字母 + 下划线分隔 + 后加下划线
```
class UrlTable {
  private:
    int a_;
    bool b_test_;
}
```

下划线主要用于 private 或 protected 成员变量；如果是 public 的变量，不建议使用下划线。

- 枚举 -- `kEnumName`
```
enum UrlTableErrors {
    kOK = 0,
    kErrorOutOfMemory,
    kErrorMalformedInput,
};
```

