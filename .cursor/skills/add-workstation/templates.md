# 工作站代码模板

本文件包含 SKILL.md 引用的所有代码模板。Agent 根据需要按需阅读。

---

## Template A：外部系统工作站

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
        self.api_host = self.config.get("api_host", "")
        self.api_key = self.config.get("api_key", "")
        self._status = "Idle"

    def post_init(self, ros_node: "ROS2WorkstationNode") -> None:
        super().post_init(ros_node)

    def _get_child_device(self, device_id: str):
        return self._children.get(device_id)

    async def scheduler_start(self, **kwargs) -> Dict[str, Any]:
        return {"success": True}

    async def create_order(self, json_str: str, **kwargs) -> Dict[str, Any]:
        return {"success": True}

    @property
    def workflow_sequence(self) -> str:
        return "[]"

    @property
    def material_info(self) -> str:
        return "{}"
```

---

## Template B：PLC/Modbus 硬件控制工作站

```python
import os
import logging
from typing import Dict, Any, Optional
from pylabrobot.resources import Deck

from unilabos.devices.workstation.workstation_base import WorkstationBase
from unilabos.device_comms.modbus_plc.client import (
    TCPClient, BaseClient, ModbusWorkflow, WorkflowAction,
)
from unilabos.device_comms.modbus_plc.modbus import (
    Base as ModbusNodeBase, DataType, WorderOrder,
)

try:
    from unilabos.ros.nodes.presets.workstation import ROS2WorkstationNode
except ImportError:
    ROS2WorkstationNode = None


class MyHardwareWorkstation(WorkstationBase):
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

        if not debug_mode:
            modbus_client = TCPClient(addr=self.address, port=self.port)
            modbus_client.client.connect()
            csv_path = os.path.join(os.path.dirname(__file__), 'register_map.csv')
            self.nodes = BaseClient.load_csv(csv_path)
            self.client = modbus_client.register_node_list(self.nodes)
        else:
            self.client = None

    def _read_coil(self, name: str) -> bool:
        if self.debug_mode:
            return False
        result, err = self.client.use_node(name).read(1)
        return result[0] if not err else False

    def _write_coil(self, name: str, value: bool):
        if not self.debug_mode:
            self.client.use_node(name).write(value)

    def _read_register(self, name: str, data_type: DataType = DataType.INT16):
        if self.debug_mode:
            return 0
        result, err = self.client.use_node(name).read(
            2 if data_type == DataType.FLOAT32 else 1
        )
        return result if not err else 0

    def _write_register(self, name: str, value, data_type: DataType = DataType.FLOAT32):
        if not self.debug_mode:
            self.client.use_node(name).write(
                value, data_type=data_type, word_order=WorderOrder.LITTLE
            )

    async def start_process(self, **kwargs) -> Dict[str, Any]:
        self._write_coil('COIL_SYS_START_CMD', True)
        return {"success": True}

    async def stop_process(self, **kwargs) -> Dict[str, Any]:
        self._write_coil('COIL_SYS_STOP_CMD', True)
        return {"success": True}

    @property
    def sys_status(self) -> str:
        return str(self._read_coil("COIL_SYS_START_STATUS"))
```

### PLC 寄存器映射 CSV 格式

```csv
Name,DataType,InitValue,Comment,Attribute,DeviceType,Address,
COIL_SYS_START_CMD,BOOL,,系统启动命令,,coil,8010,
COIL_SYS_STOP_CMD,BOOL,,系统停止命令,,coil,8020,
REG_MSG_ELECTROLYTE_VOLUME,INT16,,电解液体积,,hold_register,11004,
REG_DATA_OPEN_CIRCUIT_VOLTAGE,FLOAT32,,开路电压,,hold_register,10002,
```

命名约定：`COIL_` 线圈 / `REG_MSG_` 命令寄存器 / `REG_DATA_` 数据寄存器 / `_CMD` 写入 / `_STATUS` 读取

---

## Template C：Protocol 工作站

```python
from typing import List, Optional
from pylabrobot.resources import Resource as PLRResource
from unilabos.devices.workstation.workstation_base import ProtocolNode


class MyProtocolStation(ProtocolNode):
    def __init__(
        self,
        protocol_type: List[str],
        deck: Optional[PLRResource] = None,
        *args,
        **kwargs,
    ):
        super().__init__(protocol_type=protocol_type, deck=deck, *args, **kwargs)
```

> 通常不需要自定义类，直接在注册表和图文件中配置 `ProtocolNode` + `protocol_type` 即可。

---

## 子设备模板

### 驱动类

```python
class MyReactor:
    def __init__(self, **kwargs):
        self.data = {
            "temperature": 0.0,
            "status": "Idle",
        }

    async def update_metrics(self, metrics_json: str, **kwargs):
        import json
        metrics = json.loads(metrics_json)
        self.data["temperature"] = metrics.get("temperature", 0.0)
        self.data["status"] = metrics.get("status", "Idle")
        return {"success": True}
```

### 注册表

```yaml
reaction_station.reactor:
  category:
    - reactor
    - my_workstation
  class:
    module: unilabos.devices.workstation.my_station.my_station:MyReactor
    type: python
    status_types:
      temperature: Float64
      status: String
    action_value_mappings:
      auto-update_metrics:
        type: UniLabJsonCommandAsync
        goal:
          metrics_json: json_str
        result:
          success: success
```

> `auto-` 前缀动作不创建 ActionClient，仅供工作站驱动内部调用。

### 图文件节点

```json
{
    "id": "reactor_1",
    "name": "reactor_1",
    "children": [],
    "parent": "my_station",
    "type": "device",
    "class": "reaction_station.reactor",
    "position": {"x": 1150, "y": 300, "z": 0},
    "config": {},
    "data": {}
}
```

### 代码中访问子设备

```python
child_node = self._children.get("reactor_1")
child_node.driver_instance.update_metrics(data)

child = self._ros_node.sub_devices.get("reactor_1")
child.driver_instance.data["temperature"]
```

### 硬件代理配置

通信设备节点（ID 以 `serial_` 或 `io_` 开头）：

```json
{
    "id": "serial_port_1",
    "parent": "my_station",
    "type": "device",
    "class": "serial_device",
    "config": {
        "hardware_interface": {
            "name": "hardware_interface_name",
            "read": "read_method",
            "write": "write_method"
        }
    }
}
```

子设备注册表中声明代理：

```yaml
my_sub_device:
  class:
    hardware_interface:
      name: hardware_interface_name   # 属性名，值为通信设备 ID
      read: read_from_device          # 注入的读方法
      write: write_to_device          # 注入的写方法
```

ROS 节点初始化时自动检测 `name` 属性值是否为另一子设备 ID，若是则注入通信设备的 read/write 方法。

---

## 注册表完整配置

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

---

## 物料资源模板

### Bottle 工厂函数

```python
from unilabos.resources.itemized_carrier import Bottle


def My_Reagent_Bottle(
    name: str,
    diameter: float = 70.0,
    height: float = 120.0,
    max_volume: float = 500000.0,   # 单位 μL（500mL = 500000）
    barcode: str = None,
) -> Bottle:
    return Bottle(
        name=name, diameter=diameter, height=height,
        max_volume=max_volume, barcode=barcode,
        model="My_Reagent_Bottle",
    )
```

### BottleCarrier 工厂函数

```python
from pylabrobot.resources import ResourceHolder
from pylabrobot.resources.carrier import create_ordered_items_2d
from unilabos.resources.itemized_carrier import BottleCarrier


def My_6SlotCarrier(name: str) -> BottleCarrier:
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3, num_items_y=2,
        dx=10.0, dy=10.0, dz=5.0,
        item_dx=42.0, item_dy=35.0,
        size_x=20.0, size_y=20.0, size_z=50.0,
    )
    carrier = BottleCarrier(
        name=name, size_x=146.0, size_y=80.0, size_z=55.0,
        sites=sites, model="My_6SlotCarrier",
    )
    carrier.num_items_x = 3
    carrier.num_items_y = 2
    carrier.num_items_z = 1
    return carrier
```

### WareHouse 工厂函数

```python
from unilabos.resources.warehouse import warehouse_factory


def my_warehouse_4x4(name: str) -> "WareHouse":
    return warehouse_factory(
        name=name,
        num_items_x=4, num_items_y=4, num_items_z=1,
        dx=137.0, dy=96.0, dz=120.0,
        item_dx=137.0, item_dy=125.0, item_dz=10.0,
        resource_size_x=127.0, resource_size_y=85.0, resource_size_z=100.0,
        model="my_warehouse_4x4",
    )
```

### Deck 类

```python
from pylabrobot.resources import Deck, Coordinate


class MyStation_Deck(Deck):
    def __init__(
        self,
        name: str = "MyStation_Deck",
        size_x: float = 2700.0,
        size_y: float = 1080.0,
        size_z: float = 1500.0,
        category: str = "deck",
        setup: bool = False,
    ) -> None:
        super().__init__(name=name, size_x=size_x, size_y=size_y, size_z=size_z)
        if setup:
            self.setup()

    def setup(self) -> None:
        self.warehouses = {
            "堆栈A": my_warehouse_4x4("堆栈A"),
            "堆栈B": my_warehouse_4x4("堆栈B"),
        }
        self.warehouse_locations = {
            "堆栈A": Coordinate(-200.0, 400.0, 0.0),
            "堆栈B": Coordinate(2350.0, 400.0, 0.0),
        }
        for wh_name, wh in self.warehouses.items():
            self.assign_child_resource(wh, location=self.warehouse_locations[wh_name])
```

### 资源注册表

```yaml
# unilabos/registry/resources/<project>/bottles.yaml
My_Reagent_Bottle:
  category: [bottles]
  class:
    module: unilabos.resources.my_project.bottles:My_Reagent_Bottle
    type: pylabrobot

# unilabos/registry/resources/<project>/decks.yaml
MyStation_Deck:
  category: [deck]
  class:
    module: unilabos.resources.my_project.decks:MyStation_Deck
    type: pylabrobot
  registry_type: resource
```

### PLR 扩展注册

新 Deck 类需要在 `unilabos/resources/plr_additional_res_reg.py` 中导入：

```python
def register():
    from unilabos.resources.my_project.decks import MyStation_Deck
```
