---
name: add-device
description: Guide for adding new devices to Uni-Lab-OS (接入新设备). Walks through device category selection (thing model), communication protocol, command protocol collection, driver creation, registry YAML, and graph file setup. Use when the user wants to add/integrate a new device, create a device driver, write a device class, configure device registry, or mentions 接入设备/添加设备/设备驱动/物模型.
---

# 添加新设备到 Uni-Lab-OS

**第一步：** 使用 Read 工具读取 `docs/ai_guides/add_device.md`，获取完整的设备接入指南并严格遵循。

该指南包含：
- 8 步完整流程（设备类别、通信协议、指令收集、接口对齐、驱动创建、注册表、图文件、验证）
- 所有物模型代码模板（注射泵、电磁阀、蠕动泵、温控、电机等）
- 通信协议代码片段（Serial、Modbus、TCP、HTTP、OPC UA）
- 现有设备接口快照（用于第四步对齐，包含参数名、status_types、方法签名）
- 常见错误检查清单

**Cursor 工具映射：**

| 指南中的操作 | Cursor 中使用的工具 |
|---|---|
| 向用户确认设备类别、协议等信息 | 使用 AskQuestion 工具 |
| 搜索已有设备注册表 | 使用 Grep 在 `unilabos/registry/devices/` 中搜索 |
| 读取用户提供的协议文档/SDK 代码 | 使用 Read 工具 |
| 第四步对齐：查找同类设备接口 | 优先使用 Grep 搜索仓库中的最新注册表；指南中的「现有设备接口快照」作为兜底参考 |
