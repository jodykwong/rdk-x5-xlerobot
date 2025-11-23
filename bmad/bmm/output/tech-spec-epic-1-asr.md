# Technical Specification: 语音识别模块 (ASR)

Date: 2025-11-02
Author: BMad Method Tech Architect
Epic ID: 1
Status: Draft

---

## Overview

语音识别模块（ASR）是XLeRobot智能语音机器人系统的核心组件，负责将用户的粤语语音实时转换为文本。该模块基于FunASR框架和SenseVoiceSmall模型，集成NPU/BPU硬件加速，实现高性能、低延迟的语音识别服务。模块支持粤语、普通话和英语三种语言，以粤语为主，优化粤语语音识别的准确性和响应速度。该设计遵循ROS2分布式架构，通过标准化的服务接口与系统其他模块（LLM、TTS、系统控制）协同工作，确保端到端交互时间<5秒的性能目标。

## Objectives and Scope

### In-Scope功能
- 粤语语音实时识别与文本转换（优先级P0）
- 语音活动检测（VAD）和语音段分割
- 噪声抑制和音频预处理
- 唤醒词检测（"傻强"、"喂傻强"）
- 连续语音识别和多语言支持（普通话、英语）
- NPU/BPU硬件加速集成
- 标准ROS2服务接口和API

### Out-of-Scope功能
- 离线语音识别模型训练
- 声纹识别和说话人验证
- 语音转文字后的语义理解（NLU部分）
- 音频数据的长期存储
- 多说话人分离（Speaker Diarization）

### 技术边界
- 硬件平台：RDK X5 (8GB RAM)
- 模型支持：iic/SenseVoiceSmall
- 音频格式：16kHz, 16bit, 单声道 PCM/WAV
- 加速要求：NPU/BPU优先，CPU降级
- 响应时间：<300ms (NPU), <2s (CPU)

## System Architecture Alignment

ASR模块在软件架构中的位置：
- 位于应用服务层，与LLM服务、TTS服务并列
- 通过ROS2中间件进行分布式通信
- 依赖硬件抽象层的音频设备驱动
- 遵循ROS2节点设计模式，发布/订阅模式

组件对齐：
- 与音频输入服务通过 `/audio/input` 主题通信
- 向LLM服务发布 `/asr/result` 消息
- 与系统控制模块共享 `/system/status` 状态信息
- 使用ROS2的DDS通信机制实现模块解耦

架构约束：
- 严格遵循模块化设计原则
- 与ROS_DOMAIN_ID=42网络域配置保持一致
- 支持服务发现和健康检查机制

## Detailed Design

### Services and Modules

| 服务/模块 | 职责 | 输入 | 输出 | 负责人 |
|----------|------|------|------|--------|
| ASRNPUAccelerator | NPU/BPU硬件加速推理 | 音频数据流 | 识别文本+置信度 | ASR工程师 |
| AudioPreprocessor | 音频预处理和增强 | 原始音频流 | 预处理后音频 | 音频工程师 |
| VADProcessor | 语音活动检测 | 音频流 | 语音段边界 | 音频工程师 |
| WakeWordDetector | 唤醒词检测 | 音频流 | 唤醒状态 | 系统工程师 |
| ASRServiceNode | ROS2服务节点 | 音频消息 | ASR结果消息 | ASR工程师 |
| LanguageDetector | 语言自动检测 | 文本片段 | 语言类型 | NLP工程师 |

**ASR核心引擎规格：**
- 基础引擎：FunASR 1.0+ (FunAudio)
- 语音模型：iic/SenseVoiceSmall (多语言模型)
- 模型大小：约180MB
- 上下文长度：60秒连续语音
- 支持语言：粤语(yue)、普通话(zh)、英语(en)
- NPU加速：ONNX → BPU格式转换

### Data Models and Contracts

```python
# 音频输入消息 (ROS2 std_msgs)
AudioData:
  - format: str  # "PCM", "WAV"
  - sample_rate: int  # 16000
  - channels: int  # 1 (单声道)
  - bit_depth: int  # 16
  - timestamp: int64  # 微秒级时间戳
  - data: bytes  # 音频数据 (二进制)

# ASR识别结果消息
ASRResult:
  - text: str  # 识别文本 (UTF-8编码)
  - confidence: float32  # 置信度 (0.0-1.0)
  - processing_time: float32  # 处理时间 (毫秒)
  - language: str  # "cantonese", "mandarin", "english"
  - is_final: bool  # 是否为最终结果
  - timestamp: int64  # 处理完成时间戳
  - vad_segments: List[VoiceSegment]  # VAD语音段信息

# 语音段定义
VoiceSegment:
  - start_time: float32  # 起始时间 (秒)
  - end_time: float32    # 结束时间 (秒)
  - confidence: float32  # 段置信度

# 唤醒词检测结果
WakeWordResult:
  - detected: bool  # 是否检测到唤醒词
  - wake_word: str  # 唤醒词内容
  - confidence: float32  # 检测置信度
  - timestamp: int64  # 检测时间
```

### APIs and Interfaces

#### ROS2服务接口
**服务名称：** `/xlerobot/asr_service`
**消息类型：** 自定义消息包

**发布主题：**
- `/audio/input` - 订阅音频输入流
- `/asr/result` - 发布识别结果
- `/system/asr_status` - 发布ASR状态

**服务调用：**
- `recognize_speech()` - 同步语音识别
- `start_continuous_recognition()` - 启动连续识别
- `stop_continuous_recognition()` - 停止连续识别
- `set_language_mode(mode: str)` - 设置语言模式
- `calibrate_vad(threshold: float)` - 校准VAD阈值

#### RESTful API接口（可选）
```
POST /api/v1/asr/recognize
Content-Type: application/json

Request Body:
{
  "audio_data": "base64编码的音频数据",
  "sample_rate": 16000,
  "language": "cantonese",  // 可选：指定语言
  "enable_vad": true
}

Response:
{
  "status": "success",
  "text": "识别出的文本",
  "confidence": 0.95,
  "processing_time_ms": 280,
  "language": "cantonese"
}
```

#### 错误码定义
| 错误码 | 说明 | 处理策略 |
|--------|------|----------|
| ASR_001 | 音频格式不支持 | 返回错误提示，支持格式转换 |
| ASR_002 | 音频数据损坏 | 记录日志，请求重发 |
| ASR_003 | VAD超时 | 自动结束识别，返回部分结果 |
| ASR_004 | NPU加速失败 | 降级到CPU模式 |
| ASR_005 | 模型加载失败 | 重试加载，错误告警 |

### Workflows and Sequencing

#### 主要识别流程
```
音频输入 → VAD检测 → 语音段提取 → NPU推理 →
文本生成 → 置信度计算 → 结果发布 → 等待下次输入
```

**详细时序：**
1. 音频捕获（持续流式）
2. VAD检测语音活动（<50ms延迟）
3. 提取有效语音段（静音截断）
4. NPU/BPU加速推理（<200ms）
5. 文本生成和语言检测
6. 置信度评估
7. 发布结果到ROS2主题
8. 等待下一个语音段

#### 连续识别流程
```
启动连续识别 → 循环VAD检测 → 增量识别 →
缓冲管理 → 结果聚合 → 流式输出
```

**状态机转换：**
- Standby → Listening → Processing → Speaking → Standby
- 错误状态：Error → Recovery → Standby

## Non-Functional Requirements

### Performance

**延迟要求：**
- 语音唤醒响应：<100ms
- VAD检测延迟：<50ms
- NPU推理延迟：<200ms（单次识别）
- 总体延迟：<300ms（NPU模式），<2s（CPU模式）
- 95%请求满足延迟要求

**吞吐量要求：**
- 并发用户：1个本地用户
- 处理频率：实时16kHz音频流
- 缓冲区大小：60秒音频
- 最大处理时长：单次10秒

**资源使用限制：**
- CPU使用率：<60% (平均)
- 内存使用：<2GB (模型+缓存)
- NPU利用率：<80%
- 磁盘使用：<1GB (模型文件)

### Security

**数据保护：**
- 音频数据本地处理为主，不上传云端
- 敏感语音数据不长期存储
- API密钥安全存储（环境变量/密钥管理）
- 传输加密：TLS 1.3（如使用云API）

**访问控制：**
- ROS2主题级权限控制
- API调用频率限制（<10 QPS）
- 本地服务绑定localhost

**隐私保护：**
- 语音数据最小化原则
- 自动清理临时音频文件
- 不保留用户语音历史记录

### Reliability/Availability

**可用性指标：**
- 系统可用性：>99.9%
- 平均故障间隔时间（MTBF）：>8760小时（1年）
- 平均修复时间（MTTR）：<30分钟

**容错机制：**
- NPU失败自动降级到CPU
- API重试机制（最大3次）
- 音频设备异常自动重连
- 节点故障自动重启（systemd）

**故障恢复：**
- 优雅降级：NPU → CPU → 离线模式
- 错误隔离：单个模块故障不影响整体
- 状态持久化：配置参数持久化存储
- 自动恢复：服务异常后自动重启

### Observability

**日志记录：**
- 结构化日志（JSON格式）
- 关键事件：启动、识别、错误、警告
- 日志级别：DEBUG, INFO, WARNING, ERROR, CRITICAL
- 日志轮转：每天轮转，保留30天

**监控指标：**
- 识别准确率（实时计算）
- 平均延迟（滑动窗口）
- CPU/内存/NPU使用率
- VAD检测率
- 错误率（每小时统计）

**告警机制：**
- 识别准确率<85%告警
- 平均延迟>500ms告警
- CPU使用率>80%告警
- NPU错误率>5%告警
- 音频设备丢失告警

**可观测性工具：**
- ROS2 rqt工具
- Prometheus + Grafana监控
- ELK Stack日志分析
- 自定义监控Dashboard

## Dependencies and Integrations

**内部依赖：**
- ROS2 Humble中间件
- FunASR语音识别框架
- SenseVoiceSmall模型 (iic/SenseVoiceSmall)
- ONNX Runtime推理引擎
- LibROSA音频处理库
- PyAudio音频I/O

**外部依赖：**
- BPU SDK（NPU加速）
- Ubuntu 22.04系统库
- ALSA/PulseAudio音频系统
- PyTorch 2.0+（模型加载）

**版本约束：**
```
Python: >=3.10,<3.11
FunASR: >=1.0.0
ONNX Runtime: >=1.15.0
torch: >=2.0.0,<3.0.0
librosa: >=0.10.0,<0.11.0
```

**集成点：**
- `/audio/input` - 音频输入服务（上游）
- `/asr/result` - LLM服务（下游）
- `/system/status` - 系统控制服务（状态同步）
- 配置服务 - ASR参数配置管理

## Acceptance Criteria (Authoritative)

**AC-1: 粤语语音识别准确性**
- 标准粤语测试集识别准确率≥90%
- 测试数据：1000条标准粤语语句
- 验收方法：WER（词错误率）计算
- 测试环境：安静环境（SNR>30dB）

**AC-2: 实时性能要求**
- NPU模式下端到端延迟<300ms（95%分位数）
- CPU模式下端到端延迟<2s
- 测试方法：100次连续测量，取平均值和95%分位数
- 验收标准：平均延迟和95%分位数均达标

**AC-3: VAD语音活动检测**
- VAD检测准确率≥95%
- 语音段边界误差<100ms
- 测试方法：人工标注语音段，对比VAD检测结果
- 验收标准：误检率<5%，漏检率<5%

**AC-4: 唤醒词检测**
- 安静环境下唤醒率=100%
- 噪声环境下唤醒率>95%
- 误唤醒率<0.1%（24小时测试）
- 唤醒词："傻强"、"喂傻强"

**AC-5: 噪声抑制效果**
- 噪声环境下识别准确率降低<5%（相比安静环境）
- SNR提升>10dB（处理后）
- 测试环境：办公室噪声、街道噪声、背景音乐
- 验收方法：对比处理前后识别准确率

**AC-6: 连续语音识别**
- 连续识别30分钟无错误累积
- 长语音自动分段（每段10秒）
- 测试方法：连续播放60分钟语音，记录识别质量
- 验收标准：无内存泄漏，无性能下降

**AC-7: 多语言支持**
- 语言自动识别准确率≥95%
- 粤语识别准确率≥90%
- 普通话识别准确率≥85%
- 英语识别准确率≥80%
- 测试方法：混合语言测试集

**AC-8: 服务稳定性**
- 连续运行24小时无崩溃
- 内存使用稳定，无泄漏
- CPU使用率波动<10%
- 测试方法：长时间压力测试
- 验收标准：所有稳定性指标达标

## Traceability Mapping

| AC编号 | 规格章节 | 组件/API | 测试用例 | 测试方法 |
|--------|----------|----------|----------|----------|
| AC-1 | 详细设计-服务模块 | ASRNPUAccelerator | TC-ASR-001 | 准确率基准测试 |
| AC-2 | 非功能性-性能 | ASRServiceNode | TC-ASR-002 | 延迟性能测试 |
| AC-3 | 详细设计-服务模块 | VADProcessor | TC-ASR-003 | VAD准确性测试 |
| AC-4 | 详细设计-服务模块 | WakeWordDetector | TC-ASR-004 | 唤醒率测试 |
| AC-5 | 详细设计-服务模块 | AudioPreprocessor | TC-ASR-005 | 噪声抑制测试 |
| AC-6 | 工作流-连续识别 | ASRServiceNode | TC-ASR-006 | 稳定性测试 |
| AC-7 | 详细设计-数据模型 | LanguageDetector | TC-ASR-007 | 多语言测试 |
| AC-8 | 非功能性-可靠性 | ASRServiceNode | TC-ASR-008 | 24小时压力测试 |

**需求追踪矩阵：**
```
PRD需求 ASR-001 → AC-1, AC-2
PRD需求 ASR-002 → AC-3
PRD需求 ASR-003 → AC-5
PRD需求 ASR-004 → AC-7
PRD需求 ASR-005 → AC-4
Story 1.1 → AC-1
Story 1.2 → AC-2
Story 1.3 → AC-5
Story 1.4 → AC-6
Story 1.5 → 接口测试
```

## Risks, Assumptions, Open Questions

**技术风险：**
1. **R-001: NPU SDK集成风险**
   - 风险等级：高
   - 概率：40%
   - 影响：严重（性能不达标）
   - 缓解措施：提前获取SDK评估板，测试兼容性，准备CPU备选方案
   - 应急计划：性能目标调整至延迟<2s

2. **R-002: 粤语模型准确性不足**
   - 风险等级：中
   - 概率：30%
   - 影响：中等（需要优化）
   - 缓解措施：扩充训练数据，模型微调，多模型融合
   - 应急计划：降低验收标准至85%，引入人工确认

3. **R-003: 音频设备兼容性问题**
   - 风险等级：中
   - 概率：30%
   - 影响：中等（功能受限）
   - 缓解措施：建立音频设备白名单，自动设备检测
   - 应急计划：要求用户使用推荐设备

**假设条件：**
1. **A-001:** RDK X5硬件平台和BPU SDK按计划提供
2. **A-002:** FunASR框架和SenseVoiceSmall模型稳定可用
3. **A-003:** Ubuntu 22.04和ROS2 Humble环境稳定
4. **A-004:** 粤语语音测试数据集充足且质量良好

**开放问题：**
1. **Q-001:** BPU SDK的实际性能提升比例？（待SDK评估确认）
2. **Q-002:** 是否需要支持离线模式？（当前设计为在线）
3. **Q-003:** 语音数据缓存策略？（隐私vs性能权衡）
4. **Q-004:** 是否需要声纹识别？（当前不在范围）

## Test Strategy Summary

**测试级别和范围：**

1. **单元测试（Unit Testing）**
   - 覆盖率目标：>80%
   - 测试范围：VAD检测、预处理、模型推理、NPU加速
   - 工具：pytest, unittest
   - 频率：每次代码提交

2. **集成测试（Integration Testing）**
   - 测试范围：ROS2主题通信、ASR→LLM端到端流程
   - 测试场景：音频输入 → ASR识别 → LLM处理
   - 工具：ROS2测试框架
   - 频率：每日构建

3. **系统测试（System Testing）**
   - 测试范围：完整ASR功能
   - 测试环境：RDK X5实际硬件
   - 测试内容：性能、准确率、稳定性、兼容性
   - 频率：版本发布前

4. **用户验收测试（UAT）**
   - 测试人员：Beta用户（粤语母语者）
   - 测试内容：真实场景语音识别
   - 验收标准：符合AC-1至AC-8所有标准
   - 频率：Beta测试阶段

**测试方法：**

- **准确率测试：** 使用标准粤语测试集（1000条），计算WER
- **性能测试：** 延迟测量（95%分位数）、吞吐量测试
- **噪声测试：** 在不同SNR环境下测试（办公室、街道等）
- **压力测试：** 24小时连续运行，监控资源使用
- **兼容性测试：** 多种音频设备（USB麦克风、3.5mm接口）

**测试环境：**
- 硬件：RDK X5开发套件
- 音频设备：USB麦克风、3.5mm音频接口
- 软件：Ubuntu 22.04 + ROS2 Humble + Python 3.10.12

**验收测试矩阵：**
| 测试类型 | 目标指标 | 测试数据 | 验收标准 |
|----------|----------|----------|----------|
| 准确率 | WER<10% | 1000条粤语 | ≥90%准确率 |
| 延迟 | P95<300ms | 实时测试 | 95%满足要求 |
| 稳定性 | 24h无故障 | 连续运行 | 100%稳定 |
| 噪声 | 准确率降幅<5% | 多场景 | 指标达标 |

---
**文档版本：** v1.0
**最后更新：** 2025-11-02
**BMad Method** - 专业软件工程方法论
