---
name: edit-experiment-graph
description: Guide for creating and editing experiment graph files in Uni-Lab-OS (创建/编辑实验组态图). Covers node types, link types, parent-child relationships, deck configuration, and common graph patterns. Use when the user wants to create a graph file, edit an experiment configuration, set up device topology, or mentions 图文件/graph/组态/拓扑/实验图/experiment JSON.
---

# 创建/编辑实验组态图

实验图（Graph File）定义设备拓扑、物理连接和物料配置。系统启动时加载图文件，初始化所有设备和连接关系。

路径：`unilabos/test/experiments/<name>.json`

---

## 第一步：确认需求

向用户确认：

| 信息 | 说明 |
|------|------|
| 场景类型 | 单设备调试 / 多设备联调 / 工作站完整图 |
| 包含的设备 | 设备 ID、注册表 class 名、配置参数 |
| 连接关系 | 物理连接（管道）/ 通信连接（串口）/ 无连接 |
| 父子关系 | 是否有工作站包含子设备 |
| 物料需求 | 是否需要 Deck、容器、试剂瓶 |

---

## 第二步：JSON 顶层结构

```json
{
    "nodes": [],
    "links": []
}
```

> `links` 也可写作 `edges`，加载时两者等效。

---

## 第三步：定义 Nodes

### 节点字段

| 字段 | 类型 | 必需 | 默认值 | 说明 |
|------|------|------|--------|------|
| `id` | string | **是** | — | 节点唯一标识，links 和 children 中引用此值 |
| `class` | string | **是** | — | 对应注册表名（设备/资源 YAML 的 key），容器可为 `null` |
| `name` | string | 否 | 同 `id` | 显示名称，缺省时自动用 `id` |
| `type` | string | 否 | `"device"` | 节点类型（见下表），缺省时自动设为 `"device"` |
| `children` | string[] | 否 | `[]` | 子节点 ID 列表 |
| `parent` | string\|null | 否 | `null` | 父节点 ID，顶层设备为 `null` |
| `position` | object | 否 | `{x:0,y:0,z:0}` | 空间坐标 |
| `config` | object | 否 | `{}` | 传给驱动 `__init__` 的参数 |
| `data` | object | 否 | `{}` | 初始运行状态 |
| `size_x/y/z` | float | 否 | — | 节点物理尺寸（工作站节点常用） |

> 非标准字段（如 `api_host`）会自动移入 `config`。

### 节点类型

| `type` | 用途 | `class` 要求 |
|--------|------|-------------|
| `device` | 设备（默认） | 注册表中的设备名 |
| `deck` | 工作台面 | Deck 工厂函数/类名 |
| `container` | 容器（烧瓶、反应釜） | `null` 或具体容器类名 |

### 设备节点模板

```json
{
    "id": "my_device",
    "name": "我的设备",
    "children": [],
    "parent": null,
    "type": "device",
    "class": "registry_device_name",
    "position": {"x": 0, "y": 0, "z": 0},
    "config": {
        "port": "/dev/ttyUSB0",
        "baudrate": 115200
    },
    "data": {
        "status": "Idle"
    }
}
```

### 容器节点模板

容器用于协议系统中表示试剂瓶、反应釜等，`class` 通常为 `null`：

```json
{
    "id": "flask_DMF",
    "name": "DMF试剂瓶",
    "children": [],
    "parent": "my_station",
    "type": "container",
    "class": null,
    "position": {"x": 200, "y": 500, "z": 0},
    "config": {"max_volume": 1000.0},
    "data": {
        "liquid": [{"liquid_type": "DMF", "liquid_volume": 800.0}]
    }
}
```

### Deck 节点模板

```json
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
}
```

---

## 第四步：定义 Links

### Link 字段

| 字段 | 类型 | 说明 |
|------|------|------|
| `source` | string | 源节点 ID |
| `target` | string | 目标节点 ID |
| `type` | string | `"physical"` / `"fluid"` / `"communication"` |
| `port` | object | 端口映射 `{source_id: "port_name", target_id: "port_name"}` |

### 物理/流体连接

设备间的管道连接，协议系统用此查找路径：

```json
{
    "source": "multiway_valve_1",
    "target": "flask_DMF",
    "type": "fluid",
    "port": {
        "multiway_valve_1": "2",
        "flask_DMF": "outlet"
    }
}
```

### 通信连接

设备间的串口/IO 通信代理，加载时自动将端口信息写入目标设备 config：

```json
{
    "source": "pump_1",
    "target": "serial_device",
    "type": "communication",
    "port": {
        "pump_1": "port",
        "serial_device": "port"
    }
}
```

---

## 第五步：父子关系与工作站配置

### 工作站 + 子设备

工作站节点的 `children` 列出所有子节点 ID，子节点的 `parent` 指向工作站：

```json
{
    "id": "my_station",
    "children": ["my_deck", "pump_1", "valve_1", "reactor_1"],
    "parent": null,
    "type": "device",
    "class": "workstation",
    "config": {
        "protocol_type": ["PumpTransferProtocol", "CleanProtocol"]
    }
}
```

### 工作站 + Deck 引用

工作站节点中通过 `deck` 字段引用 Deck：

```json
{
    "id": "my_station",
    "children": ["my_deck", "sub_device_1"],
    "deck": {
        "data": {
            "_resource_child_name": "my_deck",
            "_resource_type": "unilabos.resources.my_module.decks:MyDeck"
        }
    }
}
```

**关键约束：**
- `_resource_child_name` 必须与 Deck 节点的 `id` 一致
- `_resource_type` 为 Deck 类/工厂函数的完整 Python 路径

---

## 常见图模式

### 模式 A：单设备调试

最简形式，一个设备节点，无连接：

```json
{
    "nodes": [
        {
            "id": "my_device",
            "name": "my_device",
            "children": [],
            "parent": null,
            "type": "device",
            "class": "motor.zdt_x42",
            "position": {"x": 0, "y": 0, "z": 0},
            "config": {"port": "/dev/ttyUSB0", "baudrate": 115200},
            "data": {"status": "idle"}
        }
    ],
    "links": []
}
```

### 模式 B：Protocol 工作站（泵+阀+容器）

工作站配合泵、阀、容器和物理连接，用于协议编译：

```json
{
    "nodes": [
        {
            "id": "station", "name": "协议工作站",
            "class": "workstation", "type": "device", "parent": null,
            "children": ["pump", "valve", "flask_solvent", "reactor", "waste"],
            "config": {"protocol_type": ["PumpTransferProtocol"]}
        },
        {"id": "pump", "name": "转移泵", "class": "virtual_transfer_pump",
         "type": "device", "parent": "station",
         "config": {"port": "VIRTUAL", "max_volume": 25.0},
         "data": {"status": "Idle", "position": 0.0, "valve_position": "0"}},
        {"id": "valve", "name": "多通阀", "class": "virtual_multiway_valve",
         "type": "device", "parent": "station",
         "config": {"port": "VIRTUAL", "positions": 8}},
        {"id": "flask_solvent", "name": "溶剂瓶", "type": "container",
         "class": null, "parent": "station",
         "config": {"max_volume": 1000.0},
         "data": {"liquid": [{"liquid_type": "DMF", "liquid_volume": 500}]}},
        {"id": "reactor", "name": "反应器", "type": "container",
         "class": null, "parent": "station"},
        {"id": "waste", "name": "废液瓶", "type": "container",
         "class": null, "parent": "station"}
    ],
    "links": [
        {"source": "pump", "target": "valve", "type": "fluid",
         "port": {"pump": "transferpump", "valve": "transferpump"}},
        {"source": "valve", "target": "flask_solvent", "type": "fluid",
         "port": {"valve": "1", "flask_solvent": "outlet"}},
        {"source": "valve", "target": "reactor", "type": "fluid",
         "port": {"valve": "2", "reactor": "inlet"}},
        {"source": "valve", "target": "waste", "type": "fluid",
         "port": {"valve": "3", "waste": "inlet"}}
    ]
}
```

### 模式 C：外部系统工作站 + Deck

```json
{
    "nodes": [
        {
            "id": "bioyond_station", "class": "reaction_station.bioyond",
            "parent": null, "children": ["bioyond_deck"],
            "config": {
                "api_host": "http://192.168.1.100:8080",
                "api_key": "YOUR_KEY",
                "material_type_mappings": {},
                "warehouse_mapping": {}
            },
            "deck": {
                "data": {
                    "_resource_child_name": "bioyond_deck",
                    "_resource_type": "unilabos.resources.bioyond.decks:BIOYOND_PolymerReactionStation_Deck"
                }
            }
        },
        {
            "id": "bioyond_deck", "class": "BIOYOND_PolymerReactionStation_Deck",
            "parent": "bioyond_station", "type": "deck",
            "config": {"type": "BIOYOND_PolymerReactionStation_Deck", "setup": true}
        }
    ],
    "links": []
}
```

### 模式 D：通信代理（串口设备）

泵通过串口设备通信，使用 `communication` 类型的 link。加载时系统会自动将串口端口信息写入泵的 `config`：

```json
{
    "nodes": [
        {"id": "station", "name": "工作站", "type": "device",
         "class": "workstation", "parent": null,
         "children": ["serial_1", "pump_1"]},
        {"id": "serial_1", "name": "串口", "type": "device",
         "class": "serial", "parent": "station",
         "config": {"port": "COM7", "baudrate": 9600}},
        {"id": "pump_1", "name": "注射泵", "type": "device",
         "class": "syringe_pump_with_valve.runze.SY03B-T08", "parent": "station"}
    ],
    "links": [
        {"source": "pump_1", "target": "serial_1", "type": "communication",
         "port": {"pump_1": "port", "serial_1": "port"}}
    ]
}
```

---

## 验证

```bash
# 启动测试
unilab -g unilabos/test/experiments/<name>.json --complete_registry

# 仅检查注册表
python -m unilabos --check_mode --skip_env_check
```

---

## 高级模式

处理复杂图文件时，详见 [reference.md](reference.md)：ResourceDict 完整字段 schema、Pose 标准化规则、Handle 验证机制、GraphML 格式支持、外部系统工作站完整 config 结构。

---

## 常见错误

| 错误 | 原因 | 修复 |
|------|------|------|
| `class` 找不到 | 注册表中无此设备名 | 在 `unilabos/registry/devices/` 或 `resources/` 中搜索正确名称 |
| children/parent 不一致 | 子节点 `parent` 与父节点 `children` 不匹配 | 确保双向一致 |
| `_resource_child_name` 不匹配 | Deck 引用名与 Deck 节点 `id` 不同 | 保持一致 |
| Link 端口错误 | `port` 中的 key 不是 source/target 的 `id` | key 必须是对应节点的 `id` |
| 重复 UUID | 多个节点有相同 `uuid` | 删除或修改 UUID |

---

## 参考路径

| 内容 | 路径 |
|------|------|
| 图文件目录 | `unilabos/test/experiments/` |
| 协议测试站 | `unilabos/test/experiments/Protocol_Test_Station/` |
| 图加载代码 | `unilabos/resources/graphio.py` |
| 节点模型 | `unilabos/resources/resource_tracker.py` |
| 设备注册表 | `unilabos/registry/devices/` |
| 资源注册表 | `unilabos/registry/resources/` |
| 用户文档 | `docs/user_guide/graph_files.md` |
