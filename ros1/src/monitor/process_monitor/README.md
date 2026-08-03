# process_monitor

零第三方依赖的终端进程监控器。原生程序优先按 `/proc/PID/exe` 的可执行文件名精确识别；Python、roslaunch 等解释器程序使用命令行特征补充识别。

## 目录结构

```text
process_monitor/
├── process_monitor.py       # 可执行入口，不包含业务实现
├── common/                  # 配置、模型、界面和拉起公共模块
├── process_inspector.py     # /proc 扫描和进程匹配
├── monitor_service.py       # 主业务逻辑和监控循环
├── ros1_convert.py          # ROS1 在线/离线状态发布节点
├── monitor_board_ros1.py    # 95/105 ROS1 总控面板节点
├── monitor_board_ui.py      # 总控上下分区界面
├── board_models.py          # 总控线程安全状态缓存
├── msg/
│   ├── ProcessStatus.msg
│   └── ProcessStatusArray.msg
├── launch/
│   ├── ros1_convert.launch
│   └── monitor_board_ros1.launch
├── CMakeLists.txt           # catkin 消息生成配置
├── package.xml              # ROS1 包清单
├── config/
│   ├── ros1_95.json
│   └── ros1_105.json
├── tests/                   # 分模块测试
└── run.sh                   # 启动脚本
```

## 使用

```bash
cd /path/to/process_monitor

# 按 `.json` 选择独立配置
./run.sh --profile ros1_95
./run.sh --profile ros1_105
```

- 只检查一次：
```bash
./run.sh --profile ros1_95 --once --color always
```

- 加载任意符合相同结构的单配置文件：
```bash
./run.sh --config /path/to/custom.json
```

每个 JSON 文件只表示一套 `程序集` 配置，顶层结构为：
```json
{
  "profile": "ros1_95",
  "refresh_seconds": 2,
  "programs": []
}
```

## 自动拉起接口

自动拉起默认关闭，必须同时满足：
1. 运行时传入 `--auto-restart`；
2. 对应程序的 `restart.enabled` 为 `true`；
3. 对应程序配置了 `restart.command`。

- 普通可执行程序
```json
{
  "name": "tracker",
  "group": "控制",
  "match": {"process_name": "tracker"},
  "restart": {
    "enabled": true,
    "command": ["${ROOT}/install/bin/${path}/tracker"],
    "cwd": "${ROOT}/install/bin/${path}/",
    "cooldown_seconds": 30
  }
}
```

```bash
./run.sh --profile ros1_105 --auto-restart
```

---

- 字符串命令通过 `shell` 执行，适合需要 `source` ROS 环境的命令；
- 数组命令不经过 `shell`，适合直接启动可执行文件。
- 拉起日志写入 `logs/<程序名>.log`。
- 若需要接入 systemd、Docker、SSH 或自定义启动服务，只需实现或继承 `restart_handler.py` 中的 `RestartHandler`，主监控业务和界面无需修改。

## 匹配规则

- `process_name`：精确匹配可执行文件名，最推荐。
- `cmdline_contains`：所有字符串都必须出现在完整命令行中，适合 Python/roslaunch。
- `cmdline_regex`：正则匹配完整命令行。
- 同时填写多个规则时，各规则必须全部满足。

CAN 初始化、相机 `init` 等一次性命令执行完会退出，因此没有作为常驻进程监控。

## 测试

```bash
python3 -m unittest discover -s tests -v
```

## ROS1 状态发布

该目录同时是一个名为 `process_monitor` 的 catkin 包。把它放入或链接到 catkin 工作空间的 `src` 后编译：

```bash
cd /path/to/ros1/src

catkin_make -j12
source devel/setup.bash
```

使用 launch 启动本机 proc：
```bash
roslaunch process_monitor ros1_convert.launch profile:=ros1_105
```

程序会先加载 JSON，再用 JSON 顶层的 `profile` 字段作为节点名后缀；不会直接把
launch 参数或 JSON 文件名当作后缀。例如 JSON 中的 `"profile": "ros1_105"`
会注册为 `/process_monitor_converter_ros1_105`。两台车连接到同一个 ROS master 时，
不会因节点重名互相顶掉。

如需固定名称，直接运行时可显式覆盖：
```bash
rosrun process_monitor ros1_convert.py \
  --profile ros1_105 \
  --node-name my_process_monitor
```

也可以直接启动：
```bash
rosrun process_monitor ros1_convert.py --profile ros1_95
```

直接启动时同样读取 JSON 中的 `profile` 生成节点名。

默认发布：
```text
/process_monitor/status    process_monitor/ProcessStatusArray
```

查看消息：
```bash
rostopic echo /process_monitor/status
```

`ProcessStatusArray` 是带时间戳的完整状态快照，包含配置名称、在线数量、程序总数和所有程序状态。
每个 `ProcessStatus` 包含：
- `state=0`：离线，即 `OFFLINE`；
- `state=1`：在线，即 `ONLINE`；
- `name`、`group`：程序名称和分组；
- `pids`：匹配到的全部 PID，离线时为空；
- `detail`：拉起或诊断说明。

可通过 launch 参数修改 topic 和周期：
```bash
roslaunch process_monitor ros1_convert.launch \
  profile:=ros1_95 \
  topic:=/vehicle/process_status \
  interval:=1.0
```

## 95/105 总控面板

先确保 95 和 105 两端的 `ros1_convert.py` 都向同一个 ROS master 的 `/process_monitor/status` 发布消息，然后在总控机启动：
```bash
roslaunch process_monitor monitor_board_ros1.launch
```

界面上半区固定显示 `ros1_95`，下半区固定显示 `ros1_105`。
在线显示绿色，离线显示红色；默认连续 6 秒没有收到某台车的新消息时，该区域切换为“通信超时”。

可修改 topic、上下区域配置和超时阈值：
```bash
roslaunch process_monitor monitor_board_ros1.launch \
  topic:=/vehicle/process_status \
  upper_profile:=ros1_95 \
  lower_profile:=ros1_105 \
  refresh:=0.5 \
  stale_timeout:=6.0
```

也可以直接运行：
```bash
rosrun process_monitor monitor_board_ros1.py \
  --upper-profile ros1_95 \
  --lower-profile ros1_105
```

如果 95 和 105 使用不同的 ROS master，需要先使用 multimaster、topic relay 或其他 ROS 网络桥接方式，把两端的状态消息汇聚到总控机所在的 ROS master。
