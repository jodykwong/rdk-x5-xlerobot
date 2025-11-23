# Technical Specification: 智能家居控制模块

Date: 2025-11-02
Author: BMad Method Tech Architect
Epic ID: 5
Status: Draft

---

## Overview

智能家居控制模块是XLeRobot智能语音机器人系统的重要业务功能模块，负责与智能家居设备进行通信和控制，实现语音控制智能设备、设备联动和场景自动化等功能。该模块支持多种主流智能家居协议（WiFi、蓝牙、Zigbee、Matter），能够自动发现和管理100+智能设备，提供直观的语音控制接口和场景自动化配置。通过与ASR、LLM模块的深度集成，用户可以通过自然粤语语音实现智能家居的便捷控制，提升生活品质和居住体验。

## Objectives and Scope

### In-Scope功能
- 多协议智能家居设备适配（WiFi、蓝牙、Zigbee、Matter）（优先级P1）
- 智能设备自动发现和管理（支持100+设备）
- 语音控制接口和命令解析
- 场景自动化配置和执行（支持10+预设场景）
- 设备联动控制和冲突处理
- 设备状态同步和实时监控

### Out-of-Scope功能
- 智能家居设备的生产和销售
- 复杂安防系统集成
- 能源管理系统
- 第三方智能家居云平台接入
- 设备固件升级管理

### 技术边界
- 协议支持：WiFi、蓝牙、Zigbee、Matter
- 设备容量：支持100+设备同时管理
- 控制响应时间：<2秒
- 场景自动化：支持10+预设场景
- 联动响应时间：<3秒

## System Architecture Alignment

智能家居控制模块在软件架构中的位置：
- 位于应用服务层，与核心服务并列
- 作为下游服务，订阅LLM的设备控制意图
- 通过设备适配层与物理设备通信
- 通过ROS2中间件与其他模块交互

组件对齐：
- 订阅 `/llm/response` 主题（解析设备控制意图）
- 发布 `/iot/device/status` 主题（设备状态更新）
- 与系统控制模块共享 `/system/status` 状态
- 与ASR模块协作进行语音指令识别

架构约束：
- 支持多协议并发
- 设备驱动可热插拔
- 支持设备厂商定制
- 遵循智能家居行业标准

## Detailed Design

### Services and Modules

| 服务/模块 | 职责 | 输入 | 输出 | 负责人 |
|----------|------|------|------|--------|
| ProtocolAdapter | 协议适配器 | 设备通信 | 标准化数据 | IoT工程师 |
| DeviceDiscovery | 设备发现和管理 | 网络扫描 | 设备列表 | IoT工程师 |
| VoiceControlInterface | 语音控制接口 | 语音指令 | 控制命令 | 后端工程师 |
| SceneAutomation | 场景自动化引擎 | 场景配置 | 执行结果 | 自动化工程师 |
| DeviceLinkage | 设备联动控制 | 联动规则 | 控制命令 | 自动化工程师 |
| DeviceRegistry | 设备注册表 | 设备信息 | 设备实例 | 系统工程师 |
| IoTServiceNode | ROS2服务节点 | IoT消息 | 设备消息 | IoT工程师 |

**协议支持规格：**
- WiFi：IEEE 802.11n/ac
- 蓝牙：Bluetooth 5.0+
- Zigbee：Zigbee 3.0
- Matter：Matter 1.0标准

### Data Models and Contracts

```python
# 设备信息
DeviceInfo:
  - device_id: str  # 设备唯一标识
  - name: str  # 设备名称
  - type: str  # 设备类型：light, switch, sensor, thermostat, curtain
  - protocol: str  # 协议：wifi, bluetooth, zigbee, matter
  - manufacturer: str  # 厂商
  - model: str  # 型号
  - location: str  # 位置：客厅、卧室等
  - capabilities: List[str]  # 能力：on_off, dimmable, color, temperature
  - status: str  # 在线、离线、故障

# 设备控制命令
DeviceCommand:
  - device_id: str  # 目标设备
  - command_type: str  # 命令类型
  - parameters: Dict[str, Any]  # 命令参数
  - timestamp: int64
  - source: str  # 命令来源：voice, scene, manual

# 设备状态
DeviceStatus:
  - device_id: str  # 设备ID
  - is_online: bool  # 在线状态
  - state: Dict[str, Any]  # 设备状态
  - last_update: int64  # 最后更新时间
  - battery_level: int  # 电池电量（可选）
  - signal_strength: int  # 信号强度（可选）

# 场景配置
SceneConfig:
  - scene_id: str  # 场景ID
  - name: str  # 场景名称
  - description: str  # 场景描述
  - devices: List[DeviceAction]  # 设备动作列表
  - triggers: List[Trigger]  # 触发条件
  - is_active: bool  # 是否启用

# 设备动作
DeviceAction:
  - device_id: str  # 设备ID
  - action: str  # 动作类型
  - parameters: Dict[str, Any]  # 动作参数
  - delay_ms: int  # 延迟执行（毫秒）
  - order: int  # 执行顺序

# 触发条件
Trigger:
  - trigger_type: str  # 触发类型：time, sensor, voice, manual
  - condition: Dict[str, Any]  # 触发条件
  - threshold: Any  # 阈值（可选）
  - enabled: bool  # 是否启用

# 设备联动规则
LinkageRule:
  - rule_id: str  # 联动规则ID
  - source_device: str  # 源设备
  - source_action: str  # 源动作
  - target_devices: List[DeviceAction]  # 目标设备动作
  - conditions: List[Condition]  # 联动条件
  - priority: int  # 优先级

# 联动条件
Condition:
  - condition_type: str  # 条件类型：time, state, sensor
  - key: str  # 条件键
  - operator: str  # 操作符：>, <, ==, !=
  - value: Any  # 条件值

# 语音控制指令
VoiceControlCommand:
  - intent: str  # 意图：device_control, query_status, scene_execute
  - device_type: str  # 设备类型
  - device_location: str  # 设备位置
  - action: str  # 动作
  - parameters: Dict[str, Any]  # 参数
  - original_text: str  # 原始语音文本
  - confidence: float32  # 置信度
```

### APIs and Interfaces

#### ROS2服务接口
**服务名称：** `/xlerobot/iot_service`
**消息类型：** 自定义消息包

**发布主题：**
- `/iot/device/status` - 设备状态更新
- `/iot/scene/executed` - 场景执行结果
- `/iot/control/response` - 控制响应

**订阅主题：**
- `/llm/response` - LLM设备控制指令
- `/system/control` - 系统控制命令

**服务调用：**
- `discover_devices()` - 发现设备
- `control_device()` - 控制设备
- `get_device_status()` - 获取设备状态
- `create_scene()` - 创建场景
- `execute_scene()` - 执行场景
- `add_linkage_rule()` - 添加联动规则

#### 设备通信接口

**WiFi设备通信（MQTT）：**
```python
# MQTT主题结构
home/+/+/+/status  # 设备状态上报
home/+/+/+/command # 设备控制命令

# 示例：控制客厅灯
Topic: home/living_room/light_001/command
Payload: {
  "action": "turn_on",
  "brightness": 80,
  "color": "warm_white"
}
```

**Zigbee设备通信：**
```python
# Zigbee簇（Cluster）定义
OnOff Cluster: 0x0006
Level Control: 0x0008
Color Control: 0x0300

# 示例：开灯命令
Cluster: 0x0006 (OnOff)
Command: 0x01 (On)
```

**Matter设备通信：**
```python
# Matter设备类型
Light: 0x0100
Dimmable Light: 0x0101
Extended Color Light: 0x010C
OnOff Plug-in Unit: 0x010A

# 示例：设置亮度
Cluster: 0x0008 (Level Control)
Attribute: 0x0000 (CurrentLevel)
Value: 254 (80%亮度)
```

#### 错误码定义
| 错误码 | 说明 | 处理策略 |
|--------|------|----------|
| IOT_001 | 设备不在线 | 提示用户设备离线 |
| IOT_002 | 设备无响应 | 重试机制，退避策略 |
| IOT_003 | 协议不支持 | 降级处理或提示不支持 |
| IOT_004 | 控制失败 | 记录日志，提示用户 |
| IOT_005 | 设备已存在 | 合并设备信息 |

### Workflows and Sequencing

#### 设备控制流程
```
语音指令 → 意图解析 → 设备匹配 →
协议适配 → 命令发送 → 状态确认 →
语音反馈 → 记录日志
```

**详细时序：**
1. 接收LLM设备控制指令（<100ms）
2. 解析设备类型和动作（<50ms）
3. 从设备注册表匹配设备（<20ms）
4. 选择合适协议（<10ms）
5. 发送控制命令（<2s）
6. 等待设备确认（<3s）
7. 更新设备状态（<100ms）
8. 发布状态更新（<50ms）

#### 设备发现流程
```
启动扫描 → 网络探测 → 协议识别 →
设备认证 → 信息采集 → 注册存储
```

**扫描策略：**
- WiFi：ARP扫描 + mDNS发现
- 蓝牙：低功耗扫描（LE Scan）
- Zigbee：网络拓扑扫描
- Matter： commissioning过程

#### 场景自动化流程
```
触发检测 → 条件评估 → 动作执行 →
结果反馈 → 日志记录
```

**执行策略：**
- 串行执行：按配置顺序执行
- 并行执行：同时控制多个设备
- 条件判断：执行前检查条件
- 错误处理：单个设备失败不影响整体

#### 设备联动流程
```
源设备事件 → 规则匹配 → 条件验证 →
执行联动动作 → 状态同步
```

**联动优先级：**
- 手动控制 > 场景执行 > 自动联动
- 高优先级规则优先执行
- 冲突时遵循最新规则

## Non-Functional Requirements

### Performance

**延迟要求：**
- 设备控制响应时间：<2秒
- 场景执行时间：<10秒（10设备）
- 联动响应时间：<3秒
- 设备发现时间：<30秒

**吞吐量要求：**
- 支持设备数：100+
- 并发控制：20个设备
- 场景数量：50+
- 联动规则：100+

**资源使用限制：**
- CPU使用率：<30% (平均)
- 内存使用：<1GB (设备缓存)
- 网络带宽：<50Mbps
- 磁盘使用：<200MB

### Security

**通信安全：**
- 设备通信加密（AES-128）
- 证书验证（设备认证）
- 密钥轮换（定期更新）
- 传输加密：TLS 1.3

**访问控制：**
- 设备访问权限
- 场景执行权限
- 配置修改权限
- 审计日志记录

**隐私保护：**
- 设备数据本地存储
- 敏感信息加密
- 访问日志脱敏

### Reliability/Availability

**可用性指标：**
- 模块可用性：>99%
- 设备控制成功率：>95%
- 场景执行成功率：>98%

**容错机制：**
- 设备离线自动重连
- 协议失败自动切换
- 单设备失败不影响整体

**故障恢复：**
- 设备重连机制
- 协议自动恢复
- 配置自动备份

### Observability

**日志记录：**
- 设备控制日志
- 场景执行日志
- 联动规则日志
- 错误和异常日志

**监控指标：**
- 在线设备数量
- 设备响应时间
- 控制成功率
- 协议分布统计

**告警机制：**
- 设备离线告警
- 控制失败告警
- 协议异常告警
- 性能下降告警

## Dependencies and Integrations

**内部依赖：**
- ROS2 Humble中间件
- LLM模块意图识别
- ASR模块语音识别
- 系统控制模块

**外部依赖：**
- 智能家居协议库
- 设备厂商SDK
- 网络通信库
- MQTT Broker

**版本约束：**
```
Python: >=3.10,<3.11
paho-mqtt: >=1.6.0  # MQTT客户端
bleak: >=0.20.0     # 蓝牙低功耗
zigpy: >=0.54.0     # Zigbee协议
```

**集成点：**
- `/llm/response` - LLM服务（上游）
- `/iot/device/status` - 设备状态（下游）
- `/system/status` - 系统控制（状态同步）

## Acceptance Criteria (Authoritative)

**AC-1: 多协议适配测试**
- 支持至少3种主流协议（WiFi、蓝牙、Zigbee）
- 协议切换成功率>95%
- 测试方法：多协议设备测试
- 验收标准：3种协议全部可用

**AC-2: 设备管理功能验证**
- 支持100+设备同时管理
- 设备发现成功率>90%
- 测试方法：批量设备接入测试
- 验收标准：发现率≥90%

**AC-3: 语音控制界面验收**
- 语音控制准确率>95%
- 控制响应时间<2秒
- 测试方法：50次语音控制测试
- 验收标准：准确率和响应时间达标

**AC-4: 场景自动化测试**
- 支持10+预设场景
- 场景执行成功率>95%
- 测试方法：20个场景测试
- 验收标准：成功率≥95%

**AC-5: 设备联动功能验证**
- 联动响应时间<3秒
- 联动准确率>90%
- 测试方法：10组联动测试
- 验收标准：响应时间和准确率达标

**AC-6: 设备发现性能**
- 设备发现时间<30秒
- 设备识别准确率>95%
- 测试方法：10次发现测试
- 验收标准：发现时间≤30s，准确率≥95%

**AC-7: 系统稳定性**
- 连续运行24小时无故障
- 设备连接稳定，无频繁断开
- 测试方法：24小时稳定性测试
- 验收标准：0故障

**AC-8: 协议兼容性**
- 支持主流品牌设备（小米、欧瑞博、涂鸦等）
- 协议版本兼容性测试通过
- 测试方法：多品牌设备测试
- 验收标准：支持≥5个品牌

## Traceability Mapping

| AC编号 | 规格章节 | 组件/API | 测试用例 | 测试方法 |
|--------|----------|----------|----------|----------|
| AC-1 | 详细设计-协议适配 | ProtocolAdapter | TC-IOT-001 | 协议测试 |
| AC-2 | 详细设计-设备管理 | DeviceRegistry | TC-IOT-002 | 设备管理测试 |
| AC-3 | 详细设计-语音控制 | VoiceControlInterface | TC-IOT-003 | 语音控制测试 |
| AC-4 | 详细设计-场景自动化 | SceneAutomation | TC-IOT-004 | 场景测试 |
| AC-5 | 详细设计-设备联动 | DeviceLinkage | TC-IOT-005 | 联动测试 |
| AC-6 | 详细设计-设备发现 | DeviceDiscovery | TC-IOT-006 | 发现性能测试 |
| AC-7 | 非功能性-可靠性 | IoTServiceNode | TC-IOT-007 | 稳定性测试 |
| AC-8 | 详细设计-协议适配 | ProtocolAdapter | TC-IOT-008 | 兼容性测试 |

**需求追踪矩阵：**
```
PRD需求 IOT-001 → AC-1, AC-2, AC-6, AC-8
PRD需求 IOT-002 → AC-3, AC-5
Story 5.1 → AC-1, AC-8
Story 5.2 → AC-2, AC-6
Story 5.3 → AC-3
Story 5.4 → AC-4
Story 5.5 → AC-5
```

## Risks, Assumptions, Open Questions

**技术风险：**
1. **R-001: 多协议兼容性风险**
   - 风险等级：高
   - 概率：40%
   - 影响：严重（设备无法接入）
   - 缓解措施：协议适配层设计，厂商SDK适配
   - 应急计划：重点支持WiFi协议，其他作为补充

2. **R-002: 设备联动复杂性风险**
   - 风险等级：中
   - 概率：30%
   - 影响：中等（联动失效）
   - 缓解措施：简化联动规则，冲突检测
   - 应急计划：提供手动控制替代方案

3. **R-003: 设备厂商API变化风险**
   - 风险等级：中
   - 概率：50%
   - 影响：中等（功能失效）
   - 缓解措施：版本兼容检测，定期更新
   - 应急计划：快速适配更新，临时禁用问题功能

**假设条件：**
1. **A-001:** 智能家居设备遵循标准协议
2. **A-002:** 设备网络环境稳定
3. **A-003:** 设备厂商提供必要文档和支持
4. **A-004:** 用户家庭网络带宽充足

**开放问题：**
1. **Q-001:** 是否需要支持云端设备？（当前本地设备）
2. **Q-002:** 设备数据存储策略？（本地vs云端）
3. **Q-003:** 第三方平台接入？（小米米家、华为智选等）
4. **Q-004:** 设备安全加固？（固件安全、证书管理）

## Test Strategy Summary

**测试级别和范围：**

1. **单元测试（Unit Testing）**
   - 覆盖率目标：>80%
   - 测试范围：协议适配、设备管理、场景引擎
   - 工具：pytest, mock设备
   - 频率：每次代码提交

2. **集成测试（Integration Testing）**
   - 测试范围：设备控制、场景执行、联动功能
   - 测试场景：真实设备接入
   - 工具：ROS2测试框架
   - 频率：每日构建

3. **系统测试（System Testing）**
   - 测试范围：完整IoT功能
   - 测试内容：性能、稳定性、兼容性
   - 测试环境：RDK X5 + 真实智能设备
   - 频率：版本发布前

4. **兼容性测试（Compatibility Testing）**
   - 测试范围：多品牌设备、多协议
   - 测试设备：小米、欧瑞博、涂鸦等
   - 工具：设备测试矩阵
   - 频率：每月一次

**测试方法：**

- **协议测试：** WiFi/蓝牙/Zigbee/Matter协议验证
- **设备测试：** 真实设备控制测试
- **性能测试：** 并发控制、响应时间
- **场景测试：** 自动化场景执行
- **联动测试：** 多设备联动
- **稳定性测试：** 长时间运行

**测试环境：**
- 硬件：RDK X5 + 智能设备（灯泡、插座、传感器等）
- 网络：WiFi路由器 + Zigbee网关
- 软件：Ubuntu 22.04 + ROS2 Humble
- 测试工具：协议分析仪、网络抓包工具

**验收测试矩阵：**
| 测试类型 | 目标指标 | 测试数据 | 验收标准 |
|----------|----------|----------|----------|
| 协议 | ≥3种 | 多协议设备 | 全部可用 |
| 管理 | 100+设备 | 批量设备 | 发现率≥90% |
| 语音 | 准确率>95% | 50次测试 | 达标 |
| 场景 | 成功率>95% | 20个场景 | 达标 |
| 联动 | 响应<3s | 10组联动 | 准确率≥90% |
| 发现 | 时间<30s | 10次测试 | 准确率≥95% |
| 稳定 | 24h运行 | 压力测试 | 0故障 |
| 兼容 | ≥5品牌 | 多品牌设备 | 全部可用 |

---
**文档版本：** v1.0
**最后更新：** 2025-11-02
**BMad Method** - 专业软件工程方法论
