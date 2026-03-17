---
name: validate-device
description: Validate Uni-Lab-OS device or workstation implementations against interface contracts and project rules. Use when users ask to validate device code, audit compliance, check contract compatibility, review registry alignment, or mention 验证设备/检查设备/设备审计/接口对齐/device validation/check compliance/audit device/workstation validation. Prioritize docs/ai_guides/add_device.md and unilabos/registry/devices/ as validation baselines.
---

# 设备合规验证 (Device Validation)

验证设备实现是否符合 Uni-Lab-OS 的接口契约和编码标准。

## 第一步：确定验证目标

先确定验证范围：
- 单文件：`unilabos/devices/.../*.py`
- 同类别批量：如 `pump_and_valve`、`temperature`
- 自动检测：用户刚改动设备代码时，从上下文与 git 变更推断目标

## 第二步：读取必要文件（按优先级）

使用 `ReadFile` 工具读取以下文件，并按优先级作为验证基线：

1. **设备实现文件** - 待验证的 Python 设备类
2. **对应的 YAML 注册表** - `unilabos/registry/devices/` 下的对应文件
3. **设备接入指南** - `docs/ai_guides/add_device.md`（权威规则与接口快照）
4. **CLAUDE.md / AGENTS.md** - 项目规则（关键硬约束）
5. **同类设备参考** - `unilabos/registry/devices/` 中同类别设备（优先最新仓库内容）

如果规则冲突，优先级为：**仓库当前注册表与 `add_device.md` > `CLAUDE.md/AGENTS.md` 中通用约束 > 历史示例**。

## 第三步：执行验证检查

按顺序执行并记录证据（文件路径 + 关键信息）：

### 检查 1：参数名契约验证

**规则：** 动作方法的参数名是接口契约，不可重命名。

**检查内容：**
- 扫描公开动作方法（不以 `_` 开头）
- 从 YAML `action_value_mappings` 提取动作参数名
- 对比 Python 方法签名参数名是否完全一致（不能改名）

### 检查 2：status 字符串一致性

**规则：** status 字符串必须与同类已有设备一致。

**检查内容：**
- 收集 `self.data["status"]` 所有赋值
- 对比同类设备与注册表中的标准值
- 标记中文状态、大小写不一致和自定义状态

### 检查 3：self.data 初始化完整性

**规则：** `self.data` 必须在 `__init__` 中预填充所有属性字段。

**检查内容：**
- 检查 `__init__` 中 `self.data` 初始化
- 提取 `@property` 与 YAML `status_types` 字段
- 校验 `self.data` 已预填充全部字段，且不是空字典

### 检查 4：异步 sleep 使用规范

**规则：** 异步方法中使用 `await self._ros_node.sleep()`，禁止 `time.sleep()` 和 `asyncio.sleep()`。

**检查内容：**
- 扫描所有 `async def`
- 禁止：`time.sleep(...)` / `asyncio.sleep(...)`
- 正确：`await self._ros_node.sleep(...)`

### 检查 5：YAML 与代码接口对齐

**规则：** 设备实现必须与 YAML 注册表定义的接口完全匹配。

**检查内容：**
- 属性对齐：`status_types` 字段都有对应 `@property`
- 动作对齐：`action_value_mappings` 中非 `auto-` 动作都有实现
- 返回值检查：优先 `bool` 或 `Dict[str, Any]`

### 检查 6：串口响应解析健壮性（按需适用）

**规则：** 禁止用硬编码索引解析串口响应，必须先定位帧起始标记。

**检查内容：**
- 仅在存在串口/二进制协议代码时执行
- 禁止直接按固定索引解析原始响应
- 必须先定位帧起始标记（如 `/`、`0xFE`）

### 检查 7：代码质量检查（补充项，不计入强制合规）

**可选检查项：**
- 是否有 `post_init` 方法接收 `ros_node`
- 是否有 `initialize` 和 `cleanup` 方法
- 类型转换：参数是否显式转换（`float(temp)`、`int(position)`）
- 错误处理：是否有基本的异常捕获
- 日志记录：是否使用 `self.logger`

## 第四步：生成合规报告

使用以下结构输出（简洁、可执行）：

```markdown
# 设备验证报告

**设备：** unilabos/devices/pump_and_valve/my_pump.py
**类名：** MyPump
**类别：** pump_and_valve
**验证时间：** {{date}}

## 总体评分（仅统计适用检查）

- ✅ 通过检查：5/6（Applicable）
- ❌ 失败检查：1/6
- ⏭️  跳过检查：1 项（Not Applicable）
- ⚠️  警告：3 项

## 详细结果

### ❌ 必须修复（Blocking）
1. [检查名] 问题描述（文件与位置）
   - 当前：`...`
   - 应改：`...`

### ⚠️ 建议修复（Non-blocking）
1. [检查名] 问题描述（文件与位置）
   - 建议：`...`

### ✅ 通过项
- 检查 1：...
- 检查 3：...

## 结论

该设备实现存在 **N 个必须修复问题**。修复后复检。
```

## 第五步：询问是否自动修复

如果发现了问题，询问用户：

```
发现了 2 个必须修复的问题。是否需要我自动修复这些问题？

选项：
1. 自动修复所有问题
2. 仅修复高优先级问题
3. 手动修复（我会提供详细指导）
4. 仅查看报告，不修复
```

如果用户选择自动修复，使用当前环境可用的编辑工具（优先 `ApplyPatch`）逐个修复问题，并在修复后重新验证。

## 注意事项

1. **不要过度严格** - 某些警告可能是合理的设计选择，询问用户确认
2. **提供上下文** - 解释为什么某个规则很重要
3. **批量验证** - 如果验证多个设备，提供汇总报告
4. **版本兼容** - 旧设备可能使用旧的约定，注意区分

## 常见问题

**Q: 如果找不到对应的 YAML 文件怎么办？**
A: 提示用户可能需要先创建 YAML 注册表，或者仅执行不依赖 YAML 的检查（如 status 字符串、async sleep）。

**Q: 如果设备是工作站（workstation）怎么办？**
A: 工作站有不同的验证规则，检查是否继承自 `WorkstationBase`，是否有 `deck` 配置等。

**Q: 如何处理虚拟设备（mock）？**
A: 虚拟设备可以放宽某些要求（如串口解析），但接口契约仍需遵守。

## 触发示例

用户可能这样说：
- "验证一下这个设备代码"
- "检查 my_pump.py 是否符合规范"
- "这个设备实现有什么问题吗"
- "帮我 audit 一下设备接口"
- "检查设备合规性"
- "validate device implementation"
- "验证这个 workstation 是否合规"
- "帮我检查设备实现和 registry 是否对齐"
- "做一次设备接口审计"
