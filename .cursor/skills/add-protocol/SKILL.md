---
name: add-protocol
description: Guide for adding new experiment protocols to Uni-Lab-OS (添加新实验操作协议). Walks through ROS Action definition, Pydantic model creation, protocol generator implementation, and registration. Use when the user wants to add a new protocol, create a compile function, implement an experiment operation, or mentions 协议/protocol/编译/compile/实验操作.
---

# 添加新实验操作协议（Protocol）

Protocol 是对实验有意义的完整动作（如泵转移、过滤、溶解），需要多设备协同。`compile/` 中的生成函数根据设备连接图将抽象操作"编译"为设备指令序列。

添加一个 Protocol 需修改 **6 个文件**，按以下流程执行。

---

## 第一步：确认协议信息

向用户确认：

| 信息 | 示例 |
|------|------|
| 协议英文名 | `MyNewProtocol` |
| 操作描述 | 将固体样品研磨至目标粒径 |
| Goal 参数（必需 + 可选） | `vessel: dict`, `time: float = 300.0` |
| Result 字段 | `success: bool`, `message: str` |
| 需要哪些设备协同 | 研磨器、搅拌器 |

---

## 第二步：创建 ROS Action 定义

路径：`unilabos_msgs/action/<ActionName>.action`

三段式结构（Goal / Result / Feedback），用 `---` 分隔：

```
# Goal
Resource vessel
float64 time
string mode
---
# Result
bool success
string return_info
---
# Feedback
string status
string current_device
builtin_interfaces/Duration time_spent
builtin_interfaces/Duration time_remaining
```

**类型映射：**

| Python 类型 | ROS 类型 | 说明 |
|------------|----------|------|
| `dict` | `Resource` | 容器/设备引用，自定义消息类型 |
| `float` | `float64` | |
| `int` | `int32` | |
| `str` | `string` | |
| `bool` | `bool` | |

> `Resource` 是 `unilabos_msgs/msg/Resource.msg` 中定义的自定义消息类型。

---

## 第三步：注册 Action 到 CMakeLists

在 `unilabos_msgs/CMakeLists.txt` 的 `set(action_files ...)` 块中添加：

```cmake
"action/MyNewAction.action"
```

> 调试时需编译：`cd unilabos_msgs && colcon build && source ./install/local_setup.sh && cd ..`
> PR 合并后 CI/CD 自动发布，`mamba update ros-humble-unilabos-msgs` 即可。

---

## 第四步：创建 Pydantic 模型

在 `unilabos/messages/__init__.py` 中添加（位于 `# Start Protocols` 和 `# End Protocols` 之间）：

```python
class MyNewProtocol(BaseModel):
    # === 必需参数 ===
    vessel: dict = Field(..., description="目标容器")
    
    # === 可选参数 ===
    time: float = Field(300.0, description="操作时间 (秒)")
    mode: str = Field("default", description="操作模式")
    
    def model_post_init(self, __context):
        """参数验证和修正"""
        if self.time <= 0:
            self.time = 300.0
```

**规则：**
- 参数名必须与 `.action` 文件中 Goal 字段完全一致
- `dict` 类型对应 `.action` 中的 `Resource`
- 将类名加入文件末尾的 `__all__` 列表

---

## 第五步：实现协议生成函数

路径：`unilabos/compile/<protocol_name>_protocol.py`

```python
import networkx as nx
from typing import List, Dict, Any


def generate_my_new_protocol(
    G: nx.DiGraph,
    vessel: dict,
    time: float = 300.0,
    mode: str = "default",
    **kwargs,
) -> List[Dict[str, Any]]:
    """将 MyNewProtocol 编译为设备动作序列。
    
    Args:
        G: 设备连接图（NetworkX），节点为设备/容器，边为物理连接
        vessel: 目标容器 {"id": "reactor_1"}
        time: 操作时间（秒）
        mode: 操作模式
    
    Returns:
        动作列表，每个元素为:
        - dict: 单步动作
        - list[dict]: 并行动作
    """
    from unilabos.compile.utils.vessel_parser import get_vessel
    
    vessel_id, vessel_data = get_vessel(vessel)
    actions = []
    
    # 查找相关设备（通过图的连接关系）
    # 生成动作序列
    actions.append({
        "device_id": "target_device_id",
        "action_name": "some_action",
        "action_kwargs": {"param": "value"}
    })
    
    # 等待
    actions.append({
        "action_name": "wait",
        "action_kwargs": {"time": time}
    })
    
    return actions
```

### 动作字典格式

```python
# 单步动作（发给子设备）
{"device_id": "pump_1", "action_name": "set_position", "action_kwargs": {"position": 10.0}}

# 发给工作站自身
{"device_id": "self", "action_name": "my_action", "action_kwargs": {...}}

# 等待
{"action_name": "wait", "action_kwargs": {"time": 5.0}}

# 并行动作（列表嵌套）
[
    {"device_id": "pump_1", "action_name": "set_position", "action_kwargs": {"position": 10.0}},
    {"device_id": "stirrer_1", "action_name": "start_stir", "action_kwargs": {"stir_speed": 300}}
]
```

### 关于 `vessel` 参数类型

现有协议的 `vessel` 参数类型不统一：
- 新协议趋势：使用 `dict`（如 `{"id": "reactor_1"}`）
- 旧协议：使用 `str`（如 `"reactor_1"`）
- 兼容写法：`Union[str, dict]`

**建议新协议统一使用 `dict` 类型**，通过 `get_vessel()` 兼容两种输入。

### 公共工具函数（`unilabos/compile/utils/`）

| 函数 | 用途 |
|------|------|
| `get_vessel(vessel)` | 解析容器参数为 `(vessel_id, vessel_data)`，兼容 dict 和 str |
| `find_solvent_vessel(G, solvent)` | 根据溶剂名查找容器（精确→命名规则→模糊→液体类型） |
| `find_reagent_vessel(G, reagent)` | 根据试剂名查找容器（支持固体和液体） |
| `find_connected_stirrer(G, vessel)` | 查找与容器相连的搅拌器 |
| `find_solid_dispenser(G)` | 查找固体加样器 |

### 协议内专属查找函数

许多协议在自己的文件内定义了专属的 `find_*` 函数（不在 `utils/` 中）。编写新协议时，优先复用 `utils/` 中的公共函数；如需特殊查找逻辑，在协议文件内部定义即可：

```python
def find_my_special_device(G: nx.DiGraph, vessel: str) -> str:
    """查找与容器相关的特殊设备"""
    for node in G.nodes():
        if 'my_device_type' in G.nodes[node].get('class', '').lower():
            return node
    raise ValueError("未找到特殊设备")
```

### 复用已有协议

复杂协议通常组合已有协议：

```python
from unilabos.compile.pump_protocol import generate_pump_protocol_with_rinsing

actions.extend(generate_pump_protocol_with_rinsing(
    G, from_vessel=solvent_vessel, to_vessel=vessel, volume=volume
))
```

### 图查询模式

```python
# 查找与容器相连的特定类型设备
for neighbor in G.neighbors(vessel_id):
    node_data = G.nodes[neighbor]
    if "heater" in node_data.get("class", ""):
        heater_id = neighbor
        break

# 查找最短路径（泵转移）
path = nx.shortest_path(G, source=from_vessel_id, target=to_vessel_id)
```

---

## 第六步：注册协议生成函数

在 `unilabos/compile/__init__.py` 中：

1. 顶部添加导入：

```python
from .my_new_protocol import generate_my_new_protocol
```

2. 在 `action_protocol_generators` 字典中添加映射：

```python
action_protocol_generators = {
    # ... 已有协议
    MyNewProtocol: generate_my_new_protocol,
}
```

---

## 第七步：配置图文件

在工作站的图文件中，将协议名加入 `protocol_type`：

```json
{
    "id": "my_station",
    "class": "workstation",
    "config": {
        "protocol_type": ["PumpTransferProtocol", "MyNewProtocol"]
    }
}
```

---

## 第八步：验证

```bash
# 1. 模块可导入
python -c "from unilabos.messages import MyNewProtocol; print(MyNewProtocol.model_fields)"

# 2. 生成函数可导入
python -c "from unilabos.compile import action_protocol_generators; print(list(action_protocol_generators.keys()))"

# 3. 启动测试（可选）
unilab -g <graph>.json --complete_registry
```

---

## 工作流清单

```
协议接入进度：
- [ ] 1. 确认协议名、参数、涉及设备
- [ ] 2. 创建 .action 文件 (unilabos_msgs/action/<Name>.action)
- [ ] 3. 注册到 CMakeLists.txt
- [ ] 4. 创建 Pydantic 模型 (unilabos/messages/__init__.py) + 更新 __all__
- [ ] 5. 实现生成函数 (unilabos/compile/<name>_protocol.py)
- [ ] 6. 注册到 compile/__init__.py
- [ ] 7. 配置图文件 protocol_type
- [ ] 8. 验证
```

---

## 高级模式

实现复杂协议时，详见 [reference.md](reference.md)：协议运行时数据流、mock graph 测试模式、单位解析工具（`unit_parser.py`）、复杂协议组合模式（以 dissolve 为例）。

---

## 现有协议速查

| 协议 | Pydantic 类 | 生成函数 | 核心参数 |
|------|-------------|---------|---------|
| 泵转移 | `PumpTransferProtocol` | `generate_pump_protocol_with_rinsing` | `from_vessel, to_vessel, volume` |
| 简单转移 | `TransferProtocol` | `generate_pump_protocol` | `from_vessel, to_vessel, volume` |
| 加样 | `AddProtocol` | `generate_add_protocol` | `vessel, reagent, volume` |
| 过滤 | `FilterProtocol` | `generate_filter_protocol` | `vessel, filtrate_vessel` |
| 溶解 | `DissolveProtocol` | `generate_dissolve_protocol` | `vessel, solvent, volume` |
| 加热/冷却 | `HeatChillProtocol` | `generate_heat_chill_protocol` | `vessel, temp, time` |
| 搅拌 | `StirProtocol` | `generate_stir_protocol` | `vessel, time` |
| 分离 | `SeparateProtocol` | `generate_separate_protocol` | `from_vessel, separation_vessel, solvent` |
| 蒸发 | `EvaporateProtocol` | `generate_evaporate_protocol` | `vessel, pressure, temp, time` |
| 清洗 | `CleanProtocol` | `generate_clean_protocol` | `vessel, solvent, volume` |
| 离心 | `CentrifugeProtocol` | `generate_centrifuge_protocol` | `vessel, speed, time` |
| 抽气充气 | `EvacuateAndRefillProtocol` | `generate_evacuateandrefill_protocol` | `vessel, gas` |
