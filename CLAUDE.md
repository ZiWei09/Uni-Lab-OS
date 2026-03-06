# Uni-Lab-OS

## 设备接入

读取 `docs/ai_guides/add_device.md` 获取完整的自包含指南。
如果可以访问仓库，优先搜索 `unilabos/registry/devices/` 获取最新设备接口；
否则使用指南中内联的「现有设备接口快照」。

## 关键规则

- 动作方法的参数名是接口契约，不可重命名（如 `volume` 不能改为 `volume_ml`）
- `status` 字符串必须与同类已有设备一致（如 `"Idle"` 不能改为 `"就绪"`）
- `self.data` 必须在 `__init__` 中预填充所有属性字段
- 异步方法中使用 `await self._ros_node.sleep()`，禁止 `time.sleep()` 和 `asyncio.sleep()`
