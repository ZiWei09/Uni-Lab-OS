# 添加新动作指令（Action）

1. 在 `unilabos_msgs/action` 中新建实验操作名和参数列表，如 `MyDeviceCmd.action`。一个 Action 定义由三个部分组成，分别是目标（Goal）、结果（Result）和反馈（Feedback），之间使用 `---` 分隔：

```action
# 目标（Goal）
string command
---
# 结果（Result）
bool success
---
# 反馈（Feedback）
```

2. 在 `unilabos_msgs/CMakeLists.txt` 中添加新定义的 action

将新的 action 文件添加到 `action_files` 列表中的适当功能分组下：

```cmake
set(action_files
  # Basic Commands
  "action/SendCmd.action"
  "action/Wait.action"
  "action/MyDeviceCmd.action"  # 添加你的新 action 到相应分组
  
  # Chemical Operations - Basic
  "action/Add.action"
  # ... 其他文件
)
```

注意：文件按功能分组组织，请将新的 action 添加到最适合的分组中。

3. 因为在指令集中新建了指令，因此调试时需要编译，并在终端环境中加载临时路径：

```bash
cd unilabos_msgs
colcon build
source ./install/local_setup.sh
cd ..
```

调试成功后，发起 pull request，Uni-Lab 的 CI/CD 系统会自动将新的指令集编译打包，mamba执行升级即可永久生效：

```bash
mamba update ros-humble-unilabos-msgs -c http://quetz.dp.tech:8088/get/unilab -c robostack-humble -c robostack-staging
```
