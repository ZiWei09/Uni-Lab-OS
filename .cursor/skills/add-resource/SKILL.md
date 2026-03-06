---
name: add-resource
description: Guide for adding new resources (materials, bottles, carriers, decks, warehouses) to Uni-Lab-OS (添加新物料/资源). Covers Bottle, Carrier, Deck, WareHouse definitions and registry YAML. Use when the user wants to add resources, define materials, create a deck layout, add bottles/carriers/plates, or mentions 物料/资源/resource/bottle/carrier/deck/plate/warehouse.
---

# 添加新物料资源

Uni-Lab-OS 的资源体系基于 PyLabRobot，通过扩展实现 Bottle、Carrier、WareHouse、Deck 等实验室物料管理。

---

## 第一步：确认资源类型

向用户确认需要添加的资源类型：

| 类型 | 基类 | 用途 | 示例 |
|------|------|------|------|
| **Bottle** | `Well` (PyLabRobot) | 单个容器（瓶、小瓶、烧杯、反应器） | 试剂瓶、粉末瓶 |
| **BottleCarrier** | `ItemizedCarrier` | 多槽位载架（放多个 Bottle） | 6 位试剂架、枪头盒 |
| **WareHouse** | `ItemizedCarrier` | 堆栈/仓库（放多个 Carrier） | 4x4 堆栈 |
| **Deck** | `Deck` (PyLabRobot) | 工作站台面（放多个 WareHouse） | 反应站 Deck |

**层级关系：** `Deck` → `WareHouse` → `BottleCarrier` → `Bottle`

还需确认：
- 资源所属的项目/场景（如 bioyond、battery、通用）
- 尺寸参数（直径、高度、最大容积等）
- 布局参数（行列数、间距等）

---

## 第二步：创建资源定义

### 文件位置

```
unilabos/resources/
├── <project>/          # 按项目分组
│   ├── bottles.py      # Bottle 工厂函数
│   ├── bottle_carriers.py  # Carrier 工厂函数
│   ├── warehouses.py   # WareHouse 工厂函数
│   └── decks.py        # Deck 类定义
├── itemized_carrier.py # Bottle, BottleCarrier, ItemizedCarrier 基类
├── warehouse.py        # WareHouse 基类
└── container.py        # 通用容器
```

### 2A. 添加 Bottle（工厂函数）

```python
from unilabos.resources.itemized_carrier import Bottle


def My_Reagent_Bottle(
    name: str,
    diameter: float = 70.0,     # 瓶体直径 (mm)
    height: float = 120.0,      # 瓶体高度 (mm)
    max_volume: float = 500000.0,  # 最大容积 (μL)
    barcode: str = None,
) -> Bottle:
    """创建试剂瓶"""
    return Bottle(
        name=name,
        diameter=diameter,
        height=height,
        max_volume=max_volume,
        barcode=barcode,
        model="My_Reagent_Bottle",  # 唯一标识，用于注册表和物料映射
    )
```

**Bottle 参数：**
- `name`: 实例名称（运行时唯一）
- `diameter`: 瓶体直径 (mm)
- `height`: 瓶体高度 (mm)
- `max_volume`: 最大容积 (**μL**，注意单位！500mL = 500000)
- `barcode`: 条形码（可选）
- `model`: 模型标识，与注册表 key 一致

### 2B. 添加 BottleCarrier（工厂函数）

```python
from pylabrobot.resources import ResourceHolder
from pylabrobot.resources.carrier import create_ordered_items_2d

from unilabos.resources.itemized_carrier import BottleCarrier


def My_6SlotCarrier(name: str) -> BottleCarrier:
    """创建 3x2 六槽位载架"""
    sites = create_ordered_items_2d(
        klass=ResourceHolder,
        num_items_x=3,       # 列数
        num_items_y=2,       # 行数
        dx=10.0,             # X 起始偏移
        dy=10.0,             # Y 起始偏移
        dz=5.0,              # Z 偏移
        item_dx=42.0,        # X 间距
        item_dy=35.0,        # Y 间距
        size_x=20.0,         # 槽位宽
        size_y=20.0,         # 槽位深
        size_z=50.0,         # 槽位高
    )
    carrier = BottleCarrier(
        name=name,
        size_x=146.0,        # 载架总宽
        size_y=80.0,         # 载架总深
        size_z=55.0,         # 载架总高
        sites=sites,
        model="My_6SlotCarrier",
    )
    carrier.num_items_x = 3
    carrier.num_items_y = 2
    carrier.num_items_z = 1

    # 预装 Bottle（可选）
    ordering = ["A01", "A02", "A03", "B01", "B02", "B03"]
    for i in range(6):
        carrier[i] = My_Reagent_Bottle(f"{ordering[i]}")

    return carrier
```

### 2C. 添加 WareHouse（工厂函数）

```python
from unilabos.resources.warehouse import warehouse_factory


def my_warehouse_4x4(name: str) -> "WareHouse":
    """创建 4行x4列 堆栈仓库"""
    return warehouse_factory(
        name=name,
        num_items_x=4,            # 列数
        num_items_y=4,            # 行数
        num_items_z=1,            # 层数（通常为 1）
        dx=137.0,                 # X 起始偏移
        dy=96.0,                  # Y 起始偏移
        dz=120.0,                 # Z 起始偏移
        item_dx=137.0,            # X 间距
        item_dy=125.0,            # Y 间距
        item_dz=10.0,             # Z 间距（多层时用）
        resource_size_x=127.0,    # 槽位宽
        resource_size_y=85.0,     # 槽位深
        resource_size_z=100.0,    # 槽位高
        model="my_warehouse_4x4",
    )
```

**`warehouse_factory` 参数说明：**

| 参数 | 说明 |
|------|------|
| `num_items_x/y/z` | 列数/行数/层数 |
| `dx, dy, dz` | 第一个槽位的起始坐标偏移 |
| `item_dx, item_dy, item_dz` | 相邻槽位间距 |
| `resource_size_x/y/z` | 单个槽位的物理尺寸 |
| `col_offset` | 列命名偏移（如设 4 则从 A05 开始） |
| `row_offset` | 行命名偏移（如设 5 则从 F 行开始） |
| `layout` | 排序方式：`"col-major"`（列优先，默认）/ `"row-major"`（行优先） |
| `removed_positions` | 要移除的位置索引列表 |

自动生成 `ResourceHolder` 槽位，命名规则为 `A01, B01, C01, D01, A02, ...`（列优先）或 `A01, A02, A03, A04, B01, ...`（行优先）。

### 2D. 添加 Deck（类定义）

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
            "仓库A": my_warehouse_4x4("仓库A"),
            "仓库B": my_warehouse_4x4("仓库B"),
        }
        self.warehouse_locations = {
            "仓库A": Coordinate(-200.0, 400.0, 0.0),
            "仓库B": Coordinate(2350.0, 400.0, 0.0),
        }
        for wh_name, wh in self.warehouses.items():
            self.assign_child_resource(wh, location=self.warehouse_locations[wh_name])
```

**Deck 要点：**
- 继承 `pylabrobot.resources.Deck`
- `setup()` 创建 WareHouse 并通过 `assign_child_resource` 放置到指定坐标
- `setup` 参数控制是否在构造时自动调用 `setup()`（图文件中通过 `config.setup: true` 触发）

---

## 第三步：创建注册表 YAML

路径：`unilabos/registry/resources/<project>/<type>.yaml`

### Bottle 注册

```yaml
My_Reagent_Bottle:
  category:
  - bottles
  class:
    module: unilabos.resources.my_project.bottles:My_Reagent_Bottle
    type: pylabrobot
  description: 我的试剂瓶
  handles: []
  icon: ''
  init_param_schema: {}
  version: 1.0.0
```

### Carrier 注册

```yaml
My_6SlotCarrier:
  category:
  - bottle_carriers
  class:
    module: unilabos.resources.my_project.bottle_carriers:My_6SlotCarrier
    type: pylabrobot
  handles: []
  icon: ''
  init_param_schema: {}
  version: 1.0.0
```

### Deck 注册

```yaml
MyStation_Deck:
  category:
  - deck
  class:
    module: unilabos.resources.my_project.decks:MyStation_Deck
    type: pylabrobot
  description: 我的工作站 Deck
  handles: []
  icon: ''
  init_param_schema: {}
  registry_type: resource
  version: 1.0.0
```

**注册表规则：**
- `class.module` 格式为 `python.module.path:ClassName_or_FunctionName`
- `class.type` 固定为 `pylabrobot`
- Key（如 `My_Reagent_Bottle`）必须与工厂函数名 / 类名一致
- `category` 按类型标注（`bottles`, `bottle_carriers`, `deck` 等）

---

## 第四步：在图文件中引用

### Deck 在工作站中的引用

工作站节点通过 `deck` 字段引用，Deck 作为子节点：

```json
{
    "id": "my_station",
    "children": ["my_deck"],
    "deck": {
        "data": {
            "_resource_child_name": "my_deck",
            "_resource_type": "unilabos.resources.my_project.decks:MyStation_Deck"
        }
    }
},
{
    "id": "my_deck",
    "parent": "my_station",
    "type": "deck",
    "class": "MyStation_Deck",
    "config": {"type": "MyStation_Deck", "setup": true}
}
```

### 物料类型映射（外部系统对接时）

如果工作站需要与外部系统同步物料，在 config 中配置 `material_type_mappings`：

```json
"material_type_mappings": {
    "My_Reagent_Bottle": ["试剂瓶", "external-type-uuid"],
    "My_6SlotCarrier": ["六槽载架", "external-type-uuid"]
}
```

---

## 第五步：注册 PLR 扩展（如需要）

如果添加了新的 Deck 类，需要在 `unilabos/resources/plr_additional_res_reg.py` 中导入，使 `find_subclass` 能发现它：

```python
def register():
    from unilabos.resources.my_project.decks import MyStation_Deck
```

---

## 第六步：验证

```bash
# 1. 资源可导入
python -c "from unilabos.resources.my_project.bottles import My_Reagent_Bottle; print(My_Reagent_Bottle('test'))"

# 2. Deck 可创建
python -c "
from unilabos.resources.my_project.decks import MyStation_Deck
d = MyStation_Deck('test', setup=True)
print(d.children)
"

# 3. 启动测试
unilab -g <graph>.json --complete_registry
```

---

## 工作流清单

```
资源接入进度：
- [ ] 1. 确定资源类型（Bottle / Carrier / WareHouse / Deck）
- [ ] 2. 创建资源定义（工厂函数/类）
- [ ] 3. 创建注册表 YAML (unilabos/registry/resources/<project>/<type>.yaml)
- [ ] 4. 在图文件中引用（如需要）
- [ ] 5. 注册 PLR 扩展（Deck 类需要）
- [ ] 6. 验证
```

---

## 高级模式

实现复杂资源系统时，详见 [reference.md](reference.md)：类继承体系完整图、序列化/反序列化流程、Bioyond 物料双向同步、非瓶类资源（ElectrodeSheet / Magazine）、仓库工厂 layout 模式。

---

## 现有资源参考

| 项目 | Bottles | Carriers | WareHouses | Decks |
|------|---------|----------|------------|-------|
| bioyond | `bioyond/bottles.py` | `bioyond/bottle_carriers.py` | `bioyond/warehouses.py`, `YB_warehouses.py` | `bioyond/decks.py` |
| battery | — | `battery/bottle_carriers.py` | — | — |
| 通用 | — | — | `warehouse.py` | — |

### 关键路径

| 内容 | 路径 |
|------|------|
| Bottle/Carrier 基类 | `unilabos/resources/itemized_carrier.py` |
| WareHouse 基类 + 工厂 | `unilabos/resources/warehouse.py` |
| PLR 注册 | `unilabos/resources/plr_additional_res_reg.py` |
| 资源注册表 | `unilabos/registry/resources/` |
| 图文件加载 | `unilabos/resources/graphio.py` |
| 资源跟踪器 | `unilabos/resources/resource_tracker.py` |
