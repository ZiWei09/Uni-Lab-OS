# 工作站高级模式参考

本文件是 SKILL.md 的补充，包含外部系统集成、物料同步、PLC 框架、硬件代理等高级模式。
Agent 在需要实现这些功能时按需阅读。

---

## 1. 外部系统集成模式

### 1.1 RPC 客户端

与外部 LIMS/MES 系统通信的标准模式。继承 `BaseRequest`，所有接口统一用 POST。

```python
from unilabos.device_comms.rpc import BaseRequest


class MySystemRPC(BaseRequest):
    """外部系统 RPC 客户端"""

    def __init__(self, host: str, api_key: str):
        super().__init__(host)
        self.api_key = api_key

    def _request(self, endpoint: str, data: dict = None) -> dict:
        return self.post(
            url=f"{self.host}/api/{endpoint}",
            params={
                "apiKey": self.api_key,
                "requestTime": self.get_current_time_iso8601(),
                "data": data or {},
            },
        )

    def query_status(self) -> dict:
        return self._request("status/query")

    def create_order(self, order_data: dict) -> dict:
        return self._request("order/create", order_data)
```

参考：`unilabos/devices/workstation/bioyond_studio/bioyond_rpc.py`（`BioyondV1RPC`）

### 1.2 HTTP 回调服务

接收外部系统报送的标准模式。使用 `WorkstationHTTPService`，在 `post_init` 中启动。

```python
from unilabos.devices.workstation.workstation_http_service import WorkstationHTTPService


class MyWorkstation(WorkstationBase):
    def __init__(self, config=None, deck=None, **kwargs):
        super().__init__(deck=deck, **kwargs)
        self.config = config or {}
        http_cfg = self.config.get("http_service_config", {})
        self._http_service_config = {
            "host": http_cfg.get("http_service_host", "127.0.0.1"),
            "port": http_cfg.get("http_service_port", 8080),
        }
        self.http_service = None

    def post_init(self, ros_node):
        super().post_init(ros_node)
        self.http_service = WorkstationHTTPService(
            workstation_instance=self,
            host=self._http_service_config["host"],
            port=self._http_service_config["port"],
        )
        self.http_service.start()
```

**HTTP 服务路由**（固定端点，由 `WorkstationHTTPHandler` 自动分发）：

| 端点 | 调用的工作站方法 |
|------|-----------------|
| `/report/step_finish` | `process_step_finish_report(report_request)` |
| `/report/sample_finish` | `process_sample_finish_report(report_request)` |
| `/report/order_finish` | `process_order_finish_report(report_request, used_materials)` |
| `/report/material_change` | `process_material_change_report(report_data)` |
| `/report/error_handling` | `handle_external_error(error_data)` |

实现对应方法即可接收回调：

```python
def process_step_finish_report(self, report_request) -> Dict[str, Any]:
    """处理步骤完成报告"""
    step_name = report_request.data.get("stepName")
    return {"success": True, "message": f"步骤 {step_name} 已处理"}

def process_order_finish_report(self, report_request, used_materials) -> Dict[str, Any]:
    """处理订单完成报告"""
    order_code = report_request.data.get("orderCode")
    return {"success": True}
```

参考：`unilabos/devices/workstation/workstation_http_service.py`

### 1.3 连接监控

独立线程周期性检测外部系统连接状态，状态变化时发布 ROS 事件。

```python
class ConnectionMonitor:
    def __init__(self, workstation, check_interval=30):
        self.workstation = workstation
        self.check_interval = check_interval
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._thread.start()

    def _monitor_loop(self):
        while self._running:
            try:
                self.workstation.hardware_interface.ping()
                status = "online"
            except Exception:
                status = "offline"
            time.sleep(self.check_interval)
```

参考：`unilabos/devices/workstation/bioyond_studio/station.py`（`ConnectionMonitor`）

---

## 2. Config 结构模式

工作站的 `config` 在图文件中定义，传入 `__init__`。以下是常见字段模式：

### 2.1 外部系统连接

```json
{
    "api_host": "http://192.168.1.100:8080",
    "api_key": "YOUR_API_KEY"
}
```

### 2.2 HTTP 回调服务

```json
{
    "http_service_config": {
        "http_service_host": "127.0.0.1",
        "http_service_port": 8080
    }
}
```

### 2.3 物料类型映射

将 PLR 资源类名映射到外部系统的物料类型（名称 + UUID）。用于双向物料转换。

```json
{
    "material_type_mappings": {
        "PLR_ResourceClassName": ["外部系统显示名", "external-type-uuid"],
        "BIOYOND_PolymerStation_Reactor": ["反应器", "3a14233b-902d-0d7b-..."]
    }
}
```

### 2.4 仓库映射

将仓库名映射到外部系统的仓库 UUID 和库位 UUID。用于入库/出库操作。

```json
{
    "warehouse_mapping": {
        "仓库名": {
            "uuid": "warehouse-uuid",
            "site_uuids": {
                "A01": "site-uuid-A01",
                "A02": "site-uuid-A02"
            }
        }
    }
}
```

### 2.5 工作流映射

将内部工作流名映射到外部系统的工作流 ID。

```json
{
    "workflow_mappings": {
        "internal_workflow_name": "external-workflow-uuid"
    }
}
```

### 2.6 物料默认参数

```json
{
    "material_default_parameters": {
        "NMP": {
            "unit": "毫升",
            "density": "1.03",
            "densityUnit": "g/mL",
            "description": "N-甲基吡咯烷酮"
        }
    }
}
```

### 2.7 工作流到工序名映射

```json
{
    "workflow_to_section_map": {
        "reactor_taken_in": "反应器放入",
        "reactor_taken_out": "反应器取出",
        "Solid_feeding_vials": "固体投料-小瓶"
    }
}
```

### 2.8 动作名称映射

```json
{
    "action_names": {
        "reactor_taken_in": {
            "config": "通量-配置",
            "stirring": "反应模块-开始搅拌"
        },
        "solid_feeding_vials": {
            "feeding": "粉末加样模块-投料",
            "observe": "反应模块-观察搅拌结果"
        }
    }
}
```

---

## 3. 资源同步机制

### 3.1 ResourceSynchronizer

抽象基类，用于与外部物料系统双向同步。定义在 `workstation_base.py`。

```python
from unilabos.devices.workstation.workstation_base import ResourceSynchronizer


class MyResourceSynchronizer(ResourceSynchronizer):
    def __init__(self, workstation, api_client):
        super().__init__(workstation)
        self.api_client = api_client

    def sync_from_external(self) -> bool:
        """从外部系统拉取物料到 deck"""
        external_materials = self.api_client.list_materials()
        for material in external_materials:
            plr_resource = self._convert_to_plr(material)
            self.workstation.deck.assign_child_resource(plr_resource, coordinate)
        return True

    def sync_to_external(self, plr_resource) -> bool:
        """将 deck 中的物料变更推送到外部系统"""
        external_data = self._convert_from_plr(plr_resource)
        self.api_client.update_material(external_data)
        return True

    def handle_external_change(self, change_info) -> bool:
        """处理外部系统推送的物料变更"""
        return True
```

### 3.2 资源树回调

Bioyond 工作站注册了资源树变更回调，实现与外部系统的自动同步：

| 回调名 | 触发时机 | 外部操作 |
|--------|---------|---------|
| `resource_tree_add` | PLR Deck 中添加资源 | 入库到外部系统 |
| `resource_tree_remove` | PLR Deck 中移除资源 | 出库 |
| `resource_tree_transfer` | 创建物料（不入库） | 创建外部物料记录 |
| `resource_tree_update` | 资源位置移动 | 更新外部系统库位 |

### 3.3 update_resource — 上传资源树到云端

将 PLR Deck 序列化后通过 ROS 服务上传。典型使用场景：

```python
from unilabos.ros.nodes.base_device_node import ROS2DeviceNode

# 在 post_init 中上传初始 deck
ROS2DeviceNode.run_async_func(
    self._ros_node.update_resource, True,
    **{"resources": [self.deck]}
)

# 在动作方法中更新特定资源
ROS2DeviceNode.run_async_func(
    self._ros_node.update_resource, True,
    **{"resources": [updated_plate]}
)
```

---

## 4. 工作流序列管理

工作站通过 `workflow_sequence` 属性管理任务队列（JSON 字符串形式）。

```python
class MyWorkstation(WorkstationBase):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._workflow_sequence = []

    @property
    def workflow_sequence(self) -> str:
        """返回 JSON 字符串，ROS 自动发布"""
        import json
        return json.dumps(self._workflow_sequence)

    async def append_to_workflow_sequence(self, workflow_name: str) -> Dict[str, Any]:
        """添加工作流到队列"""
        self._workflow_sequence.append({
            "name": workflow_name,
            "status": "pending",
            "created_at": time.time(),
        })
        return {"success": True}

    async def clear_workflows(self) -> Dict[str, Any]:
        """清空工作流队列"""
        self._workflow_sequence = []
        return {"success": True}
```

---

## 5. 站间物料转移

工作站之间转移物料的模式。通过 ROS ActionClient 调用目标站的动作。

```python
async def transfer_materials_to_another_station(
    self,
    target_device_id: str,
    transfer_groups: list,
    **kwargs,
) -> Dict[str, Any]:
    """将物料转移到另一个工作站"""
    target_node = self._children.get(target_device_id)
    if not target_node:
        pass

    for group in transfer_groups:
        resource = self.find_resource_by_name(group["resource_name"])
        resource.unassign()

    return {"success": True, "transferred": len(transfer_groups)}
```

参考：`BioyondDispensingStation.transfer_materials_to_reaction_station`

---

## 6. post_init 完整模式

`post_init` 是工作站初始化的关键阶段，此时 ROS 节点和子设备已就绪。

```python
def post_init(self, ros_node):
    super().post_init(ros_node)

    # 1. 初始化外部系统客户端（此时 config 已可用）
    self.rpc_client = MySystemRPC(
        host=self.config.get("api_host"),
        api_key=self.config.get("api_key"),
    )
    self.hardware_interface = self.rpc_client

    # 2. 启动连接监控
    self.connection_monitor = ConnectionMonitor(self)
    self.connection_monitor.start()

    # 3. 启动 HTTP 回调服务
    if hasattr(self, '_http_service_config'):
        self.http_service = WorkstationHTTPService(
            workstation_instance=self,
            host=self._http_service_config["host"],
            port=self._http_service_config["port"],
        )
        self.http_service.start()

    # 4. 上传 deck 到云端
    ROS2DeviceNode.run_async_func(
        self._ros_node.update_resource, True,
        **{"resources": [self.deck]}
    )

    # 5. 初始化资源同步器（可选）
    self.resource_synchronizer = MyResourceSynchronizer(self, self.rpc_client)
```

---

## 7. PLC/Modbus 完整框架

### 7.1 寄存器映射 CSV 格式

PLC 工作站使用 CSV 文件定义寄存器映射表。路径通常为工作站目录下的 `<name>.csv`。

**CSV 列定义：**

| 列名 | 说明 | 值示例 |
|------|------|--------|
| `Name` | 寄存器节点名称（代码中引用的唯一标识） | `COIL_SYS_START_CMD` |
| `DataType` | 数据类型 | `BOOL`, `INT16`, `FLOAT32` |
| `InitValue` | 初始值（可选） | — |
| `Comment` | 注释（可选） | — |
| `Attribute` | 自定义属性（可选） | — |
| `DeviceType` | Modbus 设备类型 | `coil`, `hold_register`, `input_register`, `discrete_inputs` |
| `Address` | Modbus 地址 | `8010`, `11000` |

**CSV 示例：**

```csv
Name,DataType,InitValue,Comment,Attribute,DeviceType,Address,
COIL_SYS_START_CMD,BOOL,,系统启动命令,,coil,8010,
COIL_SYS_STOP_CMD,BOOL,,系统停止命令,,coil,8020,
COIL_SYS_RESET_CMD,BOOL,,系统复位命令,,coil,8030,
REG_MSG_ELECTROLYTE_VOLUME,INT16,,电解液体积,,hold_register,11004,
REG_DATA_OPEN_CIRCUIT_VOLTAGE,FLOAT32,,开路电压,,hold_register,10002,
REG_DATA_AXIS_X_POS,FLOAT32,,X轴位置,,hold_register,10004,
```

**命名约定：**
- 线圈：`COIL_` 前缀（读写布尔量）
- 保持寄存器：`REG_MSG_`（消息/命令寄存器）、`REG_DATA_`（数据/状态寄存器）
- `_CMD` 后缀：写入命令
- `_STATUS` 后缀：读取状态

### 7.2 TCPClient 初始化

```python
from unilabos.device_comms.modbus_plc.client import TCPClient, BaseClient
from unilabos.device_comms.modbus_plc.modbus import DataType, WorderOrder

# 创建 Modbus TCP 客户端
modbus_client = TCPClient(addr="192.168.1.100", port=502)
modbus_client.client.connect()

# 从 CSV 加载寄存器映射
import os
csv_path = os.path.join(os.path.dirname(__file__), 'register_map.csv')
nodes = BaseClient.load_csv(csv_path)
client = modbus_client.register_node_list(nodes)
```

### 7.3 寄存器读写操作

```python
# 读取线圈（布尔值）
result, err = client.use_node('COIL_SYS_START_STATUS').read(1)
is_started = result[0] if not err else False

# 写入线圈
client.use_node('COIL_SYS_START_CMD').write(True)

# 读取保持寄存器（INT16）
result, err = client.use_node('REG_DATA_ASSEMBLY_COIN_CELL_NUM').read(1)

# 读取保持寄存器（FLOAT32，需要 2 个寄存器）
result, err = client.use_node('REG_DATA_OPEN_CIRCUIT_VOLTAGE').read(2)

# 写入保持寄存器（FLOAT32）
client.use_node('REG_MSG_ELECTROLYTE_VOLUME').write(
    100.0,
    data_type=DataType.FLOAT32,
    word_order=WorderOrder.LITTLE,
)
```

**FLOAT32 字节序注意：** 许多 PLC 使用 Big Byte Order + Little Word Order，需要交换两个 16 位寄存器的顺序。参考 `coin_cell_assembly.py` 中的 `_decode_float32_correct` 函数。

### 7.4 ModbusWorkflow 生命周期

PLC 工作站的动作通过 `ModbusWorkflow` + `WorkflowAction` 组织，每个动作有 4 个生命周期阶段：

```python
from unilabos.device_comms.modbus_plc.client import ModbusWorkflow, WorkflowAction

# 定义动作的生命周期函数
def my_init(use_node):
    """初始化：设置参数"""
    use_node('REG_MSG_ELECTROLYTE_VOLUME').write(
        100.0, data_type=DataType.FLOAT32, word_order=WorderOrder.LITTLE
    )
    return True

def my_start(use_node):
    """启动：触发动作并轮询等待完成"""
    use_node('COIL_SYS_START_CMD').write(True)
    while True:
        result, err = use_node('COIL_SYS_START_STATUS').read(1)
        if not err and result[0]:
            break
        time.sleep(0.5)
    return True

def my_stop(use_node):
    """停止：复位触发信号"""
    use_node('COIL_SYS_START_CMD').write(False)
    return True

def my_cleanup(use_node):
    """清理：无论成功失败都执行"""
    use_node('COIL_SYS_RESET_CMD').write(True)

# 组合成工作流
workflow = ModbusWorkflow(
    name="我的加工流程",
    actions=[
        WorkflowAction(init=my_init, start=my_start, stop=my_stop, cleanup=my_cleanup)
    ],
)

# 执行
client.run_modbus_workflow(workflow)
```

**生命周期执行顺序：** `init` → `start` → `stop` → `cleanup`（cleanup 始终执行，即使前序步骤失败）

### 7.5 PLC 工作站中的握手循环

纽扣电池组装站的典型 PLC 交互模式（信息交换握手）：

```python
async def _send_msg_to_plc(self, data: dict):
    """向 PLC 发送消息并等待确认"""
    # 1. 写入数据寄存器
    for key, value in data.items():
        self._write_register(key, value)

    # 2. 发送"消息已准备好"信号
    self._write_coil('COIL_UNILAB_SEND_MSG_SUCC_CMD', True)

    # 3. 等待 PLC 读取确认
    while not self._read_coil('COIL_REQUEST_REC_MSG_STATUS'):
        await self._ros_node.sleep(0.3)

    # 4. 撤销发送信号
    self._write_coil('COIL_UNILAB_SEND_MSG_SUCC_CMD', False)

async def _recv_msg_from_plc(self) -> dict:
    """等待 PLC 发送消息"""
    # 1. 等待 PLC 发送信号
    while not self._read_coil('COIL_REQUEST_SEND_MSG_STATUS'):
        await self._ros_node.sleep(0.3)

    # 2. 读取数据寄存器
    data = {}
    for key in self._recv_registers:
        data[key] = self._read_register(key)

    # 3. 发送"已收到"确认
    self._write_coil('COIL_UNILAB_REC_MSG_SUCC_CMD', True)

    # 4. 等待 PLC 撤销发送信号
    while self._read_coil('COIL_REQUEST_SEND_MSG_STATUS'):
        await self._ros_node.sleep(0.3)

    # 5. 撤销确认信号
    self._write_coil('COIL_UNILAB_REC_MSG_SUCC_CMD', False)

    return data
```

### 7.6 JSON 驱动的 PLC 工作流

PLC 工作站还支持通过 JSON 描述工作流，无需编写 Python 代码。使用 `BaseClient.execute_procedure_from_json`：

```json
{
    "register_node_list_from_csv_path": {"path": "register_map.csv"},
    "create_flow": [
        {
            "name": "初始化系统",
            "action": [
                {
                    "address_function_to_create": [
                        {"func_name": "write_start", "node_name": "COIL_SYS_START_CMD", "mode": "write", "value": true},
                        {"func_name": "read_status", "node_name": "COIL_SYS_START_STATUS", "mode": "read", "value": 1}
                    ],
                    "create_init_function": null,
                    "create_start_function": {
                        "func_name": "start_sys",
                        "write_functions": ["write_start"],
                        "condition_functions": ["read_status"],
                        "stop_condition_expression": "read_status[0]"
                    },
                    "create_stop_function": {"func_name": "stop_start", "node_name": "COIL_SYS_START_CMD", "mode": "write", "value": false},
                    "create_cleanup_function": null
                }
            ]
        }
    ],
    "execute_flow": ["初始化系统"]
}
```

参考：`unilabos/device_comms/modbus_plc/client.py`（`ExecuteProcedureJson` 类型定义）

---

## 8. 端到端案例 Walkthrough：Bioyond 反应站

以 Bioyond 反应站为例，展示从零接入一个带物料输入的外部系统工作站的完整过程。

### 8.1 需求

- **类型**：外部系统工作站（与 Bioyond LIMS 系统对接）
- **通信**：HTTP API（RPC 客户端 + HTTP 回调服务）
- **子设备**：5 个反应器（reactor_1 ~ reactor_5）
- **物料**：反应器、试剂瓶、烧杯、样品板、小瓶、枪头盒 → 6 种 WareHouse → 1 个 Deck

### 8.2 文件结构

```
unilabos/
├── devices/workstation/bioyond_studio/
│   ├── station.py                 # BioyondWorkstation 基类
│   ├── bioyond_rpc.py             # RPC 客户端
│   └── reaction_station/
│       └── reaction_station.py    # BioyondReactionStation + BioyondReactor
├── resources/bioyond/
│   ├── bottles.py                 # Bottle 工厂函数（8 种）
│   ├── bottle_carriers.py         # Carrier 工厂函数（8 种）
│   ├── warehouses.py              # WareHouse 工厂函数（6 种）
│   └── decks.py                   # BIOYOND_PolymerReactionStation_Deck
├── registry/
│   ├── devices/reaction_station_bioyond.yaml
│   └── resources/bioyond/
│       ├── bottles.yaml
│       ├── bottle_carriers.yaml
│       └── decks.yaml
└── test/experiments/reaction_station_bioyond.json
```

### 8.3 继承链

```
WorkstationBase
└── BioyondWorkstation                  # 通用 Bioyond 逻辑
    ├── __init__(config, deck, protocol_type)
    ├── post_init() → 启动连接监控 + HTTP 服务 + 上传 deck
    ├── BioyondResourceSynchronizer     # 物料双向同步
    └── BioyondReactionStation          # 反应站特化
        ├── reactor_taken_in()          # 反应器放入工作流
        ├── solid_feeding_vials()       # 固体投料
        ├── liquid_feeding_solvents()   # 液体投料
        └── workflow_sequence @property # 工作流序列状态
```

### 8.4 物料资源层级（反应站实例）

```
BIOYOND_PolymerReactionStation_Deck (2700×1080×1500mm)
├── 堆栈1左 (WareHouse 4x4)  ← Coordinate(-200, 400, 0)
│   ├── A01 → BottleCarrier → Reactor
│   ├── A02 → BottleCarrier → Reactor
│   └── ...（共 16 槽位）
├── 堆栈1右 (WareHouse 4x4, col_offset=4)  ← Coordinate(350, 400, 0)
│   ├── A05 → BottleCarrier → Reactor
│   └── ...
├── 站内试剂存放堆栈 (WareHouse 1x2)  ← Coordinate(1050, 400, 0)
│   ├── A01 → 1BottleCarrier → Bottle
│   └── A02 → 1BottleCarrier → Bottle
├── 测量小瓶仓库 (WareHouse 3x2)  ← Coordinate(...)
├── 站内Tip盒堆栈(左) (WareHouse, removed_positions)
└── 站内Tip盒堆栈(右) (WareHouse)
```

### 8.5 图文件关键结构

```json
{
    "nodes": [
        {
            "id": "reaction_station_bioyond",
            "children": ["Bioyond_Deck", "reactor_1", "reactor_2", "reactor_3", "reactor_4", "reactor_5"],
            "parent": null,
            "type": "device",
            "class": "reaction_station.bioyond",
            "config": {
                "api_key": "DE9BDDA0",
                "api_host": "http://172.21.103.36:45388",
                "workflow_mappings": {
                    "reactor_taken_out": "3a16081e-...",
                    "reactor_taken_in": "3a160df6-..."
                },
                "material_type_mappings": {
                    "BIOYOND_PolymerStation_Reactor": ["反应器", "3a14233b-..."],
                    "BIOYOND_PolymerStation_1BottleCarrier": ["试剂瓶", "3a14233b-..."]
                },
                "warehouse_mapping": {
                    "堆栈1左": {
                        "uuid": "3a14aa17-...",
                        "site_uuids": {"A01": "3a14aa17-...", "A02": "3a14aa17-..."}
                    }
                },
                "http_service_config": {
                    "http_service_host": "127.0.0.1",
                    "http_service_port": 8080
                }
            },
            "deck": {
                "data": {
                    "_resource_child_name": "Bioyond_Deck",
                    "_resource_type": "unilabos.resources.bioyond.decks:BIOYOND_PolymerReactionStation_Deck"
                }
            },
            "size_x": 2700.0,
            "size_y": 1080.0,
            "size_z": 2500.0,
            "protocol_type": [],
            "data": {}
        },
        {
            "id": "Bioyond_Deck",
            "parent": "reaction_station_bioyond",
            "type": "deck",
            "class": "BIOYOND_PolymerReactionStation_Deck",
            "config": {"type": "BIOYOND_PolymerReactionStation_Deck", "setup": true}
        },
        {
            "id": "reactor_1",
            "parent": "reaction_station_bioyond",
            "type": "device",
            "class": "reaction_station.reactor",
            "position": {"x": 1150, "y": 300, "z": 0},
            "config": {}
        }
    ]
}
```

### 8.6 初始化时序

```
1. ROS2WorkstationNode.__init__
   ├── 创建 BioyondReactionStation 实例（__init__）
   ├── 加载 Deck（BIOYOND_PolymerReactionStation_Deck, setup=true → 创建 6 个 WareHouse）
   ├── 初始化 reactor_1~5（BioyondReactor 实例）→ sub_devices
   └── 为每个 reactor 创建 ActionClient

2. BioyondReactionStation.post_init(ros_node)
   ├── 初始化 BioyondV1RPC（HTTP 客户端）
   ├── 初始化 BioyondResourceSynchronizer
   ├── 启动 ConnectionMonitor（30s 轮询）
   ├── 启动 WorkstationHTTPService（接收回调）
   ├── sync_from_external()（从 Bioyond 拉取物料到 Deck）
   └── update_resource([self.deck])（上传 Deck 到云端）
```

### 8.7 物料同步流程

```
外部入库：
  Bioyond API → stock_material() → 获取物料列表
  → resource_bioyond_to_plr() → 转为 PLR Bottle/Carrier
  → deck.warehouses["堆栈1左"]["A01"] = carrier
  → update_resource([deck])

外部变更回调：
  Bioyond POST /report/material_change
  → WorkstationHTTPService 接收
  → process_material_change_report()
  → 更新 Deck 中的资源
  → update_resource([affected_resource])
```

### 8.8 工作站动作执行流程（以 reactor_taken_in 为例）

```python
async def reactor_taken_in(self, assign_material_name, cutoff, temperature, **kwargs):
    # 1. 从 config 获取工作流 UUID
    workflow_id = self.config["workflow_mappings"]["reactor_taken_in"]

    # 2. 构建工序参数
    sections = self._build_sections(temperature, cutoff, ...)

    # 3. 合并到工作流序列
    self._workflow_sequence.append({"name": "reactor_taken_in", ...})

    # 4. 调用外部系统创建工单
    result = self.hardware_interface.create_order(order_data)

    # 5. 等待外部系统完成（通过 HTTP 回调通知）
    # process_order_finish_report 被回调时更新状态

    return {"success": True}
```

---

## 9. 现有工作站 Config 结构完整对比

| 特性 | BioyondReactionStation | BioyondDispensingStation | CoinCellAssemblyWorkstation |
|------|----------------------|------------------------|-----------------------------|
| **继承** | BioyondWorkstation | BioyondWorkstation | WorkstationBase (直接) |
| **通信方式** | HTTP RPC | HTTP RPC | Modbus TCP |
| **`__init__` 签名** | `(config, deck, protocol_type, **kwargs)` | `(config, deck, protocol_type, **kwargs)` | `(config, deck, address, port, debug_mode, **kwargs)` |
| **子设备** | 5 个 BioyondReactor | 无 | 无 |
| **Deck** | BioyondReactionDeck (6 个 WareHouse) | BioyondDispensingDeck | CoincellDeck |
| **物料同步** | BioyondResourceSynchronizer (双向) | BioyondResourceSynchronizer (双向) | 无（本地 PLR） |
| **status_types** | `workflow_sequence: str` | 空 | 18 个属性 (sys_status, 传感器数据等) |
| **动作风格** | 语义化 (reactor_taken_in, ...) | 语义化 (compute_experiment_design, ...) | PLC 操作 (func_pack_device_init, ...) |
| **post_init** | 连接监控 + HTTP 服务 + 资源同步 + 上传 deck | 继承父类 | 上传 deck |
| **工作流管理** | workflow_mappings → 合并序列 → create_order | batch_create → wait_for_reports | PLC 握手循环 |

### Config 字段对比

| 字段 | 反应站 | 配液站 | 纽扣电池 |
|------|--------|--------|---------|
| `api_host` | ✅ | ✅ | — |
| `api_key` | ✅ | ✅ | — |
| `workflow_mappings` | ✅ (8 个工作流) | — | — |
| `material_type_mappings` | ✅ (8 种物料) | ✅ | — |
| `warehouse_mapping` | ✅ (6 个仓库) | ✅ (3 个仓库) | — |
| `workflow_to_section_map` | ✅ | — | — |
| `action_names` | ✅ | — | — |
| `http_service_config` | ✅ | — | — |
| `material_default_parameters` | ✅ | — | — |
| `address` (init 参数) | — | — | ✅ |
| `port` (init 参数) | — | — | ✅ |
| `debug_mode` (init 参数) | — | — | ✅ |
