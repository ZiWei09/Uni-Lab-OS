# Uni-Lab-OS AI Agent 指南

## 设备接入

当用户要求添加/接入新设备时，读取 `docs/ai_guides/add_device.md` 并按其流程执行。
该指南完全自包含，包含物模型模板、现有设备接口快照、常见错误和验证清单。

## 关键规则

- 动作方法的参数名是接口契约，不可重命名（如 `volume` 不能改为 `volume_ml`）
- `status` 字符串必须与同类已有设备一致（如 `"Idle"` 不能改为 `"就绪"`）
- `self.data` 必须在 `__init__` 中预填充所有属性字段
- 异步方法中使用 `await self._ros_node.sleep()`，禁止 `time.sleep()` 和 `asyncio.sleep()`

## 项目结构

- 设备驱动：`unilabos/devices/<category>/<device_name>.py`
- 设备注册表：`unilabos/registry/devices/<device_name>.yaml`
- 实验图文件：`unilabos/test/experiments/*.json`
- 人类开发文档：`docs/developer_guide/`
- AI 专用指南：`docs/ai_guides/`
