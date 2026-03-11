---
name: add-workstation
description: Guide for adding new workstations to Uni-Lab-OS (接入新工作站). Walks through workstation type selection, sub-device composition, external system integration, driver creation, registry YAML, deck setup, and graph file configuration. Use when the user wants to add/integrate a new workstation, create a workstation driver, configure a station with sub-devices, set up deck and materials, or mentions 工作站/工站/station/workstation.
---

# Uni-Lab-OS 工作站接入指南

工作站是组合多个子设备的大型设备，拥有独立的物料管理系统（PLR Deck）和工作流引擎。

> **完整代码模板**见 [templates.md](templates.md)，**高级模式**见 [reference.md](reference.md)。

## 第一步：确定工作站类型

向用户确认：

| 类型 | 基类 | 适用场景 | 示例 |
|------|------|----------|------|
| **Protocol** | `ProtocolNode` | 标准化学操作协议 | FilterProtocolStation |
| **外部系统** | `WorkstationBase` | 对接 LIMS/MES API | BioyondStation |
| **硬件控制** | `WorkstationBase` | 直接控制 PLC/硬件 | CoinCellAssembly |

还需确认：
- 英文名称、通信方式（HTTP/Modbus/OPC UA/无）
- 子设备组成（哪些已有、哪些新增、硬件代理关系）
- 物料需求（是否需要 Deck、物料类型、是否需外部同步）

---

## 第二步：理解工作站架构

| 维度 | 普通设备 | 工作站 |
|------|---------|--------|
| 基类 | 纯 Python 类 | `WorkstationBase` / `ProtocolNode` |
| ROS 节点 | `BaseROS2DeviceNode` | `ROS2WorkstationNode` |
| 状态管理 | `self.data` 字典 | `@property` 直接访问 |
| 子设备 | 无 | `self._children` / `self._ros_node.sub_devices` |
| 物料 | 无 | `self.deck`（PLR Deck） |

### 继承体系

```
WorkstationBase (ABC)
├── BioyondWorkstation          ← HTTP RPC + 资源同步
│   ├── BioyondReactionStation
│   └── BioyondDispensingStation
├── CoinCellAssemblyWorkstation ← Modbus/PLC
└── ProtocolNode                ← 标准化学协议
```

### 子设备初始化流程

`ROS2WorkstationNode.__init__` → 遍历 `children`（type=="device"）→ `initialize_device_from_dict()` → 存入 `sub_devices` → 为每个动作创建 `ActionClient` → 识别通信设备（`serial_*`/`io_*`）→ `_setup_hardware_proxy()`

---

## 第三步：创建驱动文件

路径：`unilabos/devices/workstation/<station_name>/<station_name>.py`

根据类型选择模板（完整代码见 [templates.md](templates.md)）：

| 类型 | 模板 | 关键要素 |
|------|------|---------|
| 外部系统 | Template A | `config` 接收 API 配置，`post_init` 启动 RPC/HTTP 服务 |
| 硬件控制 | Template B | `TCPClient` + CSV 寄存器映射，`use_node()` 读写 |
| Protocol | Template C | 直接使用 `ProtocolNode`，通常不需要自定义类 |

**所有模板的 `__init__` 必须接受 `deck` 和 `**kwargs`。**

---

## 第四步：创建子设备（如需要）

子设备是独立设备，有自己的驱动类和注册表。完整模板见 [templates.md § 子设备模板](templates.md)。

### 关键要点

1. **驱动类**：普通 Python 类，`self.data` 预填所有属性
2. **注册表**：`category` 包含工作站标识，`auto-` 前缀动作不创建 ActionClient
3. **图文件**：`parent` 指向工作站 ID，`type: "device"`
4. **代码访问**：`self._children.get("reactor_1").driver_instance`

### 硬件代理模式

当子设备需要通过通信设备（串口/IO）通信时：

1. 通信设备 ID 必须以 `serial_` 或 `io_` 开头
2. 子设备注册表中声明 `hardware_interface: {name, read, write}`
3. 子设备实例的 `name` 属性值 = 通信设备 ID
4. ROS 节点自动将通信设备的 read/write 方法注入到子设备上

---

## 第五步：创建注册表 YAML

路径：`unilabos/registry/devices/<station_name>.yaml`

**最小配置（`--complete_registry` 自动补全）：**

```yaml
my_workstation:
  category:
    - workstation
  class:
    module: unilabos.devices.workstation.my_station.my_station:MyWorkstation
    type: python
```

**完整配置**见 [templates.md § 注册表完整配置](templates.md)。

---

## 第六步：配置物料系统（如需要）

物料层级：`Deck` → `WareHouse` → `ResourceHolder` (site) → `BottleCarrier` → `Bottle`

### 快速流程

1. **创建 Bottle**（`unilabos/resources/<project>/bottles.py`）— 工厂函数，返回 `Bottle` 实例
2. **创建 Carrier**（`.../bottle_carriers.py`）— 工厂函数，用 `create_ordered_items_2d` 定义槽位
3. **创建 WareHouse**（`.../warehouses.py`）— 用 `warehouse_factory()` 创建堆栈
4. **创建 Deck**（`.../decks.py`）— 继承 `pylabrobot.resources.Deck`，`setup()` 中放置 WareHouse
5. **注册表**（`unilabos/registry/resources/<project>/`）— `class.type: pylabrobot`
6. **PLR 扩展**（`unilabos/resources/plr_additional_res_reg.py`）— 导入新 Deck 类

完整代码模板见 [templates.md § 物料资源模板](templates.md)。

### 图文件中的 Deck 配置

工作站节点引用 Deck：

```json
"deck": {
    "data": {
        "_resource_child_name": "my_deck",
        "_resource_type": "unilabos.resources.my_project.decks:MyStation_Deck"
    }
}
```

Deck 子节点：

```json
{
    "id": "my_deck",
    "parent": "my_station",
    "type": "deck",
    "class": "MyStation_Deck",
    "config": {"type": "MyStation_Deck", "setup": true, "rotation": {"x": 0, "y": 0, "z": 0, "type": "Rotation"}}
}
```

> **`_resource_child_name`** 必须与 Deck 节点的 `id` 一致。

---

## 第七步：配置图文件

路径：`unilabos/test/experiments/<station_name>.json`

```json
{
    "nodes": [
        {
            "id": "my_station",
            "name": "my_station",
            "children": ["my_deck", "sub_device_1"],
            "parent": null,
            "type": "device",
            "class": "my_workstation",
            "position": {"x": 0, "y": 0, "z": 0},
            "config": {},
            "deck": {"data": {"_resource_child_name": "my_deck", "_resource_type": "...decks:MyStation_Deck"}},
            "size_x": 2700.0, "size_y": 1080.0, "size_z": 1500.0,
            "protocol_type": [],
            "data": {}
        },
        {"id": "my_deck", "parent": "my_station", "type": "deck", "class": "MyStation_Deck", "config": {"type": "MyStation_Deck", "setup": true}},
        {"id": "sub_device_1", "parent": "my_station", "type": "device", "class": "sub_device_class", "config": {}}
    ]
}
```

### 图文件规则

| 字段 | 说明 |
|------|------|
| `children` | 包含 deck ID 和所有子设备 ID |
| `parent` | 工作站为 `null`；子设备/deck 指向工作站 ID |
| `type` | 工作站和子设备 `"device"`；deck 为 `"deck"` |
| `class` | 注册表中的设备名 |
| `protocol_type` | Protocol 工作站填协议名列表；否则 `[]` |
| `config` | 传入 `__init__` 的 `config` 参数 |

### Config 字段速查

| 字段 | 外部系统 | PLC/硬件 | 说明 |
|------|---------|---------|------|
| `api_host` / `api_key` | ✅ | — | 外部 API 连接 |
| `address` / `port` | — | ✅ | PLC 地址（init 参数，非 config 内） |
| `workflow_mappings` | ✅ | — | 工作流名 → 外部 UUID |
| `material_type_mappings` | ✅ | — | PLR 资源类 → 外部物料类型 |
| `warehouse_mapping` | ✅ | — | 仓库 → 外部 UUID + 库位 UUID |
| `http_service_config` | ✅ | — | HTTP 回调 host/port |

> 完整 Config 结构详见 [reference.md § 2](reference.md)

---

## 第八步：验证

```bash
python -c "from unilabos.devices.workstation.<name>.<name> import <ClassName>"
unilab -g <graph>.json --complete_registry
unilab -g <graph>.json
```

---

## 关键规则

1. `__init__` 必须接受 `deck` 和 `**kwargs`
2. 通过 `self._children` 访问子设备，不自行维护引用
3. `post_init` 中启动后台服务，不在 `__init__` 中启动网络连接
4. 异步方法使用 `await self._ros_node.sleep()`，禁止 `time.sleep()` / `asyncio.sleep()`
5. 子设备在图文件中声明，不在驱动代码中创建
6. `_resource_child_name` 必须与 deck 节点 ID 一致
7. Protocol 工作站优先使用 `ProtocolNode`
8. 通信设备 ID 以 `serial_` 或 `io_` 开头

---

## 工作流清单

```
- [ ] 1. 确定类型（Protocol / 外部系统 / 硬件控制）
- [ ] 2. 确认子设备组成和物料需求
- [ ] 3. 创建工作站驱动
- [ ] 4. 创建子设备驱动 + 注册表（如需要）
- [ ] 5. 创建工作站注册表
- [ ] 6. 创建物料资源 Bottle→Carrier→WareHouse→Deck（如需要）
- [ ] 7. 注册 PLR 扩展（Deck 类需要）
- [ ] 8. 配置图文件
- [ ] 9. 验证
```

---

## 参考资源

- **代码模板**：[templates.md](templates.md) — 驱动模板 A/B/C、子设备、注册表、物料资源
- **高级模式**：[reference.md](reference.md) — 外部系统集成、Config 模式、资源同步、PLC 框架、端到端案例
- **现有工作站**：

| 工作站 | 注册表名 | 类型 | 驱动路径 |
|--------|----------|------|---------|
| Bioyond 反应站 | `reaction_station.bioyond` | 外部系统 | `bioyond_studio/reaction_station/` |
| Bioyond 配液站 | `bioyond_dispensing_station` | 外部系统 | `bioyond_studio/dispensing_station/` |
| 纽扣电池组装 | `coincellassemblyworkstation_device` | 硬件控制 | `coin_cell_assembly/` |
| Protocol 通用 | `workstation` | Protocol | `workstation_base.py` |
