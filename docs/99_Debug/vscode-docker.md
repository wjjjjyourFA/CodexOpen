##  方案 1：通过 .devcontainer.json 明确指定使用现有容器

在你的工程目录下（例如 /path/workspace/CodexOpen/）创建：

``.devcontainer/devcontainer.json`

### 给 vscode 当前容器改个稳定的名字：

```c++
docker rename vsc-codexopen-container jojo_shared_container
```

##  方案 2：使用 VSCode 命令直接附加（推荐）

如果只是想临时打开某个容器的环境，不想在每个项目写 `.json`：

1. 打开命令面板（`Ctrl+Shift+P`）
2. 输入并选择：
    **Dev Containers: Attach to Running Container...**
3. 选择你现有的容器（例如 `jojo_shared_container`）
4. 选中之后，VSCode 会自动 attach 并在里面打开新工程。

> ✅ 优点：无需修改配置文件
>  ❌ 缺点：每次都得手动 attach 一次（不能自动关联多个工程）
