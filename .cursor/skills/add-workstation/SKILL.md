---
name: add-workstation
description: Guide for adding new workstations to Uni-Lab-OS (接入新工作站). Walks through workstation type selection, sub-device composition, external system integration, driver creation, registry YAML, deck setup, and graph file configuration. Use when the user wants to add/integrate a new workstation, create a workstation driver, configure a station with sub-devices, set up deck and materials, or mentions 工作站/工站/station/workstation.
---

# Uni-Lab-OS 工作站接入指南

工作站（workstation）是组合多个子设备的大型设备，拥有独立的物料管理系统（PLR Deck）和工作流引擎。本指南覆盖从需求分析到验证的全流程。

> **前置知识**：工作站接入基于 `docs/ai_guides/add_device.md` 的通用设备接入框架，但有显著差异。阅读本指南前无需先读通用指南。

## 第一步：确定工作站类型

向用户确认以下信息：

**Q1: 工作站的业务场景？**

| 类型 | 基类 | 适用场景 | 示例 |
|------|------|----------|------|
| **Protocol 工作站** | `ProtocolNode` | 标准化学操作协议（过滤、转移、加热等） | FilterProtocolStation |
| **外部系统工作站** | `WorkstationBase` | 与外部 LIMS/MES 系统对接，有专属 API | BioyondStation |
| **硬件控制工作站** | `WorkstationBase` | 直接控制 PLC/硬件，无外部系统 | CoinCellAssembly |

**Q2: 工作站英文名称？**（如 `my_reaction_station`）

**Q3: 与外部系统的交互方式？**

| 方式 | 适用场景 | 需要的配置 |
|------|----------|-----------|
| 无外部系统 | Protocol 工作站、纯硬件控制 | 无 |
| HTTP API | LIMS/MES 系统（如 Bioyond） | `api_host`, `api_key` |
| Modbus TCP | PLC 控制 | `address`, `port` |
| OPC UA | 工业设备 | `url` |

**Q4: 子设备组成？**
- 列出所有子设备（如反应器、泵、阀、传感器等）
- 哪些是已有设备类型？哪些需要新增？
- 子设备之间的硬件代理关系（如泵通过串口设备通信）

**Q5: 物料管理需求？**
- 是否需要 Deck（物料面板）？
- 物料类型（plate、tip_rack、bottle 等）
- 是否需要与外部物料系统同步？

---

## 第二步：理解工作站架构

工作站与普通设备的核心差异：

| 维度 | 普通设备 | 工作站 |
|------|---------|--------|
| 基类 | 无（纯 Python 类） | `WorkstationBase` 或 `ProtocolNode` |
| ROS 节点 | `BaseROS2DeviceNode` | `ROS2WorkstationNode` |
| 状态管理 | `self.data` 字典 | 通常不用 `self.data`，用 `@property` 直接访问 |
| 子设备 | 无 | `children` 列表，通过 `self._children` 访问 |
| 物料 | 无 | `self.deck`（PLR Deck） |
| 图文件角色 | `parent: null` 或 `parent: "<station>"` | `parent: null`，含 `children` 和 `deck` |

### 继承体系

`WorkstationBase` (ABC) → `ProtocolNode` (通用协议) / `BioyondWorkstation` (→ ReactionStation, DispensingStation) / `CoinCellAssemblyWorkstation` (硬件控制)

### ROS 层

`ROS2WorkstationNode` 额外负责：初始化 children 子设备节点、为子设备创建 ActionClient、配置硬件代理、为 protocol_type 创建协议 ActionServer。

---

## 第三步：创建驱动文件

文件路径：`unilabos/devices/workstation/<station_name>/<station_name>.py`

### 模板 A：基于外部系统的工作站

适用于与 LIMS/MES 等外部系统对接的场景。

```python
import logging
from typing import Dict, Any, Optional, List
from pylabrobot.resources import Deck

from unilabos.devices.workstation.workstation_base import WorkstationBase

try:
    from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode
except ImportError:
    ROS2WorkstationNode = None


class MyWorkstation(WorkstationBase):
    """工作站描述"""

    _ros_node: "ROS2WorkstationNode"

    def __init__(
        self,
        config: dict = None,
        deck: Optional[Deck] = None,
        protocol_type: list = None,
        **kwargs,
    ):
        super().__init__(deck=deck, **kwargs)
        self.config = config or {}
        self.logger = logging.getLogger(f"MyWorkstation")

        # 外部系统连接配置
        self.api_host = self.config.get("api_host", "")
        self.api_key = self.config.get("api_key", "")

        # 工作站业务状态（不同于 self.data 模式）
        self._status = "Idle"

    def post_init(self, ros_node: "ROS2WorkstationNode") -> None:
        super().post_init(ros_node)
        # 在这里启动后台服务、连接监控等

    # ============ 子设备访问 ============

    def _get_child_device(self, device_id: str):
        """通过 ID 获取子设备节点"""
        return self._children.get(device_id)

    # ============ 动作方法 ============

    async def scheduler_start(self, **kwargs) -> Dict[str, Any]:
        """启动调度器"""
        return {"success": True}

    async def create_order(self, json_str: str, **kwargs) -> Dict[str, Any]:
        """创建工单"""
        return {"success": True}

    # ============ 属性 ============

    @property
    def workflow_sequence(self) -> str:
        return "[]"

    @property
    def material_info(self) -> str:
        return "{}"
```

### 模板 B：基于硬件控制的工作站

适用于直接与 PLC/硬件通信的场景。

```python
import logging
from typing import Dict, Any, Optional
from pylabrobot.resources import Deck

from unilabos.devices.workstation.workstation_base import WorkstationBase

try:
    from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode
except ImportError:
    ROS2WorkstationNode = None


class MyHardwareWorkstation(WorkstationBase):
    """硬件控制工作站"""

    _ros_node: "ROS2WorkstationNode"

    def __init__(
        self,
        config: dict = None,
        deck: Optional[Deck] = None,
        address: str = "192.168.1.100",
        port: str = "502",
        debug_mode: bool = False,
        *args,
        **kwargs,
    ):
        super().__init__(deck=deck, *args, **kwargs)
        self.config = config or {}
        self.address = address
        self.port = int(port)
        self.debug_mode = debug_mode
        self.logger = logging.getLogger("MyHardwareWorkstation")

        # 初始化通信客户端
        if not debug_mode:
            from unilabos.device_comms.modbus_plc.client import ModbusTcpClient
            self.client = ModbusTcpClient(host=self.address, port=self.port)
        else:
            self.client = None

    def post_init(self, ros_node: "ROS2WorkstationNode") -> None:
        super().post_init(ros_node)

    # ============ 硬件读写 ============

    def _read_register(self, name: str):
        """读取 Modbus 寄存器"""
        if self.debug_mode:
            return 0
        # 实际读取逻辑
        pass

    # ============ 动作方法 ============

    async def start_process(self, **kwargs) -> Dict[str, Any]:
        """启动加工流程"""
        return {"success": True}

    async def stop_process(self, **kwargs) -> Dict[str, Any]:
        """停止加工流程"""
        return {"success": True}

    # ============ 属性（从硬件实时读取）============

    @property
    def sys_status(self) -> str:
        return str(self._read_register("SYS_STATUS"))
```

### 模板 C：Protocol 工作站

适用于标准化学操作协议的场景，直接使用 `ProtocolNode`。

```python
from typing import List, Optional
from pylabrobot.resources import Resource as PLRResource

from unilabos.devices.workstation.workstation_base import ProtocolNode


class MyProtocolStation(ProtocolNode):
    """Protocol 工作站 — 使用标准化学操作协议"""

    def __init__(
        self,
        protocol_type: List[str],
        deck: Optional[PLRResource] = None,
        *args,
        **kwargs,
    ):
        super().__init__(protocol_type=protocol_type, deck=deck, *args, **kwargs)
```

> Protocol 工作站通常不需要自定义驱动类，直接使用 `ProtocolNode` 并在注册表和图文件中配置 `protocol_type` 即可。

---

## 第四步：创建子设备驱动（如需要）

工作站的子设备本身是独立设备。按 `docs/ai_guides/add_device.md` 的标准流程创建。

子设备的关键约束：
- 在图文件中 `parent` 指向工作站 ID
- 图文件中在工作站的 `children` 数组里列出
- 如需硬件代理，在子设备的 `config.hardware_interface.name` 指向通信设备 ID

---

## 第五步：创建注册表 YAML

路径：`unilabos/registry/devices/<station_name>.yaml`

### 最小配置

```yaml
my_workstation:
  category:
    - workstation
  class:
    module: unilabos.devices.workstation.my_station.my_station:MyWorkstation
    type: python
```

启动时 `--complete_registry` 自动补全 `status_types` 和 `action_value_mappings`。

### 完整配置参考

```yaml
my_workstation:
  description: "我的工作站"
  version: "1.0.0"
  category:
    - workstation
    - my_category
  class:
    module: unilabos.devices.workstation.my_station.my_station:MyWorkstation
    type: python
    status_types:
      workflow_sequence: String
      material_info: String
    action_value_mappings:
      scheduler_start:
        type: UniLabJsonCommandAsync
        goal: {}
        result:
          success: success
      create_order:
        type: UniLabJsonCommandAsync
        goal:
          json_str: json_str
        result:
          success: success
  init_param_schema:
    config:
      type: object
    deck:
      type: object
    protocol_type:
      type: array
```

### 子设备注册表

子设备有独立的注册表文件，需要在 `category` 中包含工作站标识：

```yaml
my_reactor:
  category:
    - reactor
    - my_workstation
  class:
    module: unilabos.devices.workstation.my_station.my_reactor:MyReactor
    type: python
```

---

## 第六步：配置 Deck 资源（如需要）

如果工作站有物料管理需求，需要定义 Deck 类。

### 使用已有 Deck 类

查看 `unilabos/resources/` 目录下是否有适用的 Deck 类。

### 创建自定义 Deck

在 `unilabos/resources/<category>/decks.py` 中定义：

```python
from pylabrobot.resources import Deck
from pylabrobot.resources.coordinate import Coordinate


def MyStation_Deck(name: str = "MyStation_Deck") -> Deck:
    deck = Deck(name=name, size_x=2700.0, size_y=1080.0, size_z=1500.0)
    # 在 deck 上定义子资源位置（carrier、plate 等）
    return deck
```

在 `unilabos/resources/<category>/` 下注册或通过注册表引用。

---

## 第七步：配置图文件

图文件路径：`unilabos/test/experiments/<station_name>.json`

### 完整结构

```json
{
    "nodes": [
        {
            "id": "my_station",
            "name": "my_station",
            "children": ["my_deck", "sub_device_1", "sub_device_2"],
            "parent": null,
            "type": "device",
            "class": "my_workstation",
            "position": {"x": 0, "y": 0, "z": 0},
            "config": {
                "api_host": "http://192.168.1.100:8080",
                "api_key": "YOUR_KEY"
            },
            "deck": {
                "data": {
                    "_resource_child_name": "my_deck",
                    "_resource_type": "unilabos.resources.my_module.decks:MyStation_Deck"
                }
            },
            "size_x": 2700.0,
            "size_y": 1080.0,
            "size_z": 1500.0,
            "protocol_type": [],
            "data": {}
        },
        {
            "id": "my_deck",
            "name": "my_deck",
            "children": [],
            "parent": "my_station",
            "type": "deck",
            "class": "MyStation_Deck",
            "position": {"x": 0, "y": 0, "z": 0},
            "config": {
                "type": "MyStation_Deck",
                "setup": true,
                "rotation": {"x": 0, "y": 0, "z": 0, "type": "Rotation"}
            },
            "data": {}
        },
        {
            "id": "sub_device_1",
            "name": "sub_device_1",
            "children": [],
            "parent": "my_station",
            "type": "device",
            "class": "sub_device_registry_name",
            "position": {"x": 100, "y": 0, "z": 0},
            "config": {},
            "data": {}
        }
    ]
}
```

### 图文件规则

| 字段 | 说明 |
|------|------|
| `id` | 节点唯一标识，与 `children` 数组中的引用一致 |
| `children` | 包含 deck ID 和所有子设备 ID |
| `parent` | 工作站节点为 `null`；子设备/deck 指向工作站 ID |
| `type` | 工作站和子设备为 `"device"`；deck 为 `"deck"` |
| `class` | 对应注册表中的设备名 |
| `deck.data._resource_child_name` | 必须与 deck 节点的 `id` 一致 |
| `deck.data._resource_type` | Deck 工厂函数的完整 Python 路径 |
| `protocol_type` | Protocol 工作站填入协议名列表；否则为 `[]` |
| `config` | 传入驱动 `__init__` 的 `config` 参数 |

---

## 第八步：验证

```bash
# 1. 模块可导入
python -c "from unilabos.devices.workstation.<name>.<name> import <ClassName>"

# 2. 注册表补全
unilab -g <graph>.json --complete_registry

# 3. 启动测试
unilab -g <graph>.json
```

---

## 高级模式

实现外部系统对接型工作站时，详见 [reference.md](reference.md)：RPC 客户端、HTTP 回调服务、连接监控、Config 结构模式（material_type_mappings / warehouse_mapping / workflow_mappings）、ResourceSynchronizer、update_resource、工作流序列、站间物料转移、post_init 完整模式。

---

## 关键规则

1. **`__init__` 必须接受 `deck` 和 `**kwargs`** — `WorkstationBase.__init__` 需要 `deck` 参数
2. **通过 `self._children` 访问子设备** — 不要自行维护子设备引用
3. **`post_init` 中启动后台服务** — 不要在 `__init__` 中启动网络连接
4. **异步方法使用 `await self._ros_node.sleep()`** — 禁止 `time.sleep()` 和 `asyncio.sleep()`
5. **子设备在图文件中声明** — 不在驱动代码中创建子设备实例
6. **`deck` 配置中的 `_resource_child_name` 必须与 deck 节点 ID 一致**
7. **Protocol 工作站优先使用 `ProtocolNode`** — 不需要自定义类

---

## 工作流清单

```
工作站接入进度：
- [ ] 1. 确定工作站类型（Protocol / 外部系统 / 硬件控制）
- [ ] 2. 确认子设备组成和物料需求
- [ ] 3. 创建工作站驱动 unilabos/devices/workstation/<name>/<name>.py
- [ ] 4. 创建子设备驱动（如需要，按 add_device.md 流程）
- [ ] 5. 创建注册表 unilabos/registry/devices/<name>.yaml
- [ ] 6. 创建/选择 Deck 资源类（如需要）
- [ ] 7. 配置图文件 unilabos/test/experiments/<name>.json
- [ ] 8. 验证：可导入 + 注册表补全 + 启动测试
```

---

## 现有工作站参考

| 工作站 | 注册表名 | 驱动类 | 类型 |
|--------|----------|--------|------|
| Protocol 通用 | `workstation` | `ProtocolNode` | Protocol |
| Bioyond 反应站 | `reaction_station.bioyond` | `BioyondReactionStation` | 外部系统 |
| Bioyond 配液站 | `bioyond_dispensing_station` | `BioyondDispensingStation` | 外部系统 |
| 纽扣电池组装 | `coincellassemblyworkstation_device` | `CoinCellAssemblyWorkstation` | 硬件控制 |

### 参考文件路径

- 基类: `unilabos/devices/workstation/workstation_base.py`
- Bioyond 基类: `unilabos/devices/workstation/bioyond_studio/station.py`
- 反应站: `unilabos/devices/workstation/bioyond_studio/reaction_station/reaction_station.py`
- 配液站: `unilabos/devices/workstation/bioyond_studio/dispensing_station/dispensing_station.py`
- 纽扣电池: `unilabos/devices/workstation/coin_cell_assembly/coin_cell_assembly.py`
- ROS 节点: `unilabos/ros/nodes/presets/workstation.py`
- 图文件: `unilabos/test/experiments/reaction_station_bioyond.json`, `dispensing_station_bioyond.json`
