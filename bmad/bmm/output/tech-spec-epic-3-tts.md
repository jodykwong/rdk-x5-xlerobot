# Technical Specification: 语音合成模块 (TTS)

Date: 2025-11-02
Author: BMad Method Tech Architect
Epic ID: 3
Status: Draft

---

## Overview

语音合成模块（TTS）是XLeRobot智能语音机器人系统的重要输出组件，负责将LLM生成的文本转换为自然流畅的粤语语音。该模块基于Piper VITS语音合成技术，集成zh_CN-huayan-medium模型，支持NPU/BPU硬件加速，实现高质量、低延迟的语音合成服务。模块支持多种音色选择、语速控制、音量调节和情感表达，为用户提供个性化的语音体验。该设计遵循ROS2分布式架构，通过标准化的服务接口与LLM模块和音频输出系统协同工作，确保端到端交互体验流畅自然。

## Objectives and Scope

### In-Scope功能
- 粤语语音合成和输出（优先级P0）
- 多种语音音色支持（至少3种）
- 语速控制（0.5x-2.0x可调）
- 音量控制（0-100级可调）
- 情感语音合成（开心、悲伤、愤怒、平静、兴奋）
- NPU/BPU硬件加速集成

### Out-of-Scope功能
- 实时变声和声音克隆
- 多语言混合语音合成
- 3D空间音效处理
- 语音情感识别（输入端）
- 自定义音色训练

### 技术边界
- 硬件平台：RDK X5 (8GB RAM)
- 合成引擎：Piper VITS
- 模型支持：zh_CN-huayan-medium
- 音频格式：22.05kHz, 16bit, WAV/PCM
- 加速要求：NPU/BPU优先，CPU降级
- 合成延迟：<1ms (NPU)，<1s (CPU)

## System Architecture Alignment

TTS模块在软件架构中的位置：
- 位于应用服务层，与ASR、LLM服务并列
- 作为下游服务，订阅LLM响应
- 作为上游服务，发布音频到音频输出系统
- 通过ROS2中间件进行异步通信

组件对齐：
- 订阅 `/llm/response` 主题（来自LLM模块）
- 发布 `/tts/output` 主题（到音频输出系统）
- 与系统控制模块共享 `/system/status` 状态
- 使用ROS2服务调用进行同步控制

架构约束：
- 必须支持NPU加速降级到CPU
- 音频缓冲区管理，避免音频卡顿
- 合成结果缓存机制
- 支持实时流式合成和批量合成

## Detailed Design

### Services and Modules

| 服务/模块 | 职责 | 输入 | 输出 | 负责人 |
|----------|------|------|------|--------|
| TTSNPUAccelerator | NPU/BPU硬件加速合成 | 文本参数 | 音频数据 | TTS工程师 |
| VoiceRenderer | 语音渲染和合成 | 文本+参数 | 音频流 | TTS工程师 |
| AudioPostProcessor | 音频后处理 | 原始音频流 | 处理后音频 | 音频工程师 |
| VoiceManager | 音色管理 | 音色请求 | 音色模型 | TTS工程师 |
| EmotionEngine | 情感表达引擎 | 文本情感 | 情感参数 | NLP工程师 |
| TTSServiceNode | ROS2服务节点 | LLM消息 | 音频消息 | TTS工程师 |
| AudioBufferManager | 音频缓冲区管理 | 音频片段 | 完整音频 | 系统工程师 |

**TTS核心引擎规格：**
- 合成引擎：Piper VITS 1.2+
- 主模型：zh_CN-huayan-medium
- 音频质量：22.05kHz采样率
- 位深：16bit
- 音色数量：≥3种（默认、温和、活泼）
- 合成速度：>10倍实时

### Data Models and Contracts

```python
# TTS服务输入消息（订阅LLM响应）
LLMResponse:
  - text: str  # 待合成文本
  - confidence: float32  # 置信度
  - processing_time: float32  # 处理时间
  - intent: str  # 意图类型
  - emotion: str  # 期望情感（可选）

# TTS输入参数消息
TTSInput:
  - text: str  # 待合成文本
  - voice: str  # 音色选择：default, gentle, lively
  - speed: float32  # 语速：0.5-2.0
  - volume: float32  # 音量：0-100
  - emotion: str  # 情感：neutral, happy, sad, angry, excited
  - pitch: float32  # 音调：0.5-2.0

# TTS输出音频消息
TTSAudioData:
  - data: bytes  # 音频数据 (WAV格式)
  - sample_rate: int32  # 22050
  - channels: int32  # 1 (单声道)
  - bit_depth: int32  # 16
  - duration: float32  # 音频时长（秒）
  - format: str  # "wav", "pcm"
  - timestamp: int64  # 合成完成时间

# 音色定义
VoiceProfile:
  - voice_id: str  # 音色ID
  - name: str  # 音色名称
  - model_path: str  # 模型文件路径
  - sample_rate: int  # 22050
  - characteristics: Dict[str, float]  # 音色特征
  - gender: str  # "male", "female", "neutral"
  - age_range: str  # "child", "adult", "senior"

# 情感参数
EmotionParameters:
  - emotion: str  # 情感类型
  - intensity: float32  # 强度：0.0-1.0
  - duration_factor: float32  # 时长因子
  - pitch_shift: float32  # 音调变化
  - speed_factor: float32  # 语速因子
```

### APIs and Interfaces

#### ROS2服务接口
**服务名称：** `/xlerobot/tts_service`
**消息类型：** 自定义消息包

**发布主题：**
- `/tts/output` - 发布音频数据（到音频输出）
- `/tts/status` - 发布TTS服务状态

**订阅主题：**
- `/llm/response` - 订阅LLM响应

**服务调用：**
- `synthesize_speech()` - 同步语音合成
- `synthesize_stream()` - 流式语音合成
- `set_voice_profile()` - 设置音色
- `adjust_parameters()` - 调整合成参数
- `cache_lookup()` - 查找缓存

#### RESTful API接口（可选）
```
POST /api/v1/tts/synthesize
Content-Type: application/json

Request Body:
{
  "text": "待合成文本",
  "voice": "default",
  "speed": 1.0,
  "volume": 70,
  "emotion": "neutral"
}

Response:
{
  "status": "success",
  "audio_url": "http://localhost:8080/audio/12345.wav",
  "duration": 3.2,
  "format": "wav",
  "sample_rate": 22050
}
```

#### 音频格式支持
| 格式 | 编码 | 采样率 | 位深 | 声道 | 说明 |
|------|------|--------|------|------|------|
| WAV | PCM | 22050 | 16bit | 单声道 | 默认格式 |
| PCM | RAW | 22050 | 16bit | 单声道 | 无压缩 |
| MP3 | MP3 | 22050 | - | 单声道 | 压缩格式（可选） |

#### 错误码定义
| 错误码 | 说明 | 处理策略 |
|--------|------|----------|
| TTS_001 | 文本过长 | 分段合成 |
| TTS_002 | 音色不存在 | 使用默认音色 |
| TTS_003 | 参数无效 | 使用默认值 |
| TTS_004 | NPU加速失败 | 降级到CPU |
| TTS_005 | 音频输出错误 | 重试合成 |

### Workflows and Sequencing

#### 主合成流程
```
接收LLM文本 → 参数解析 → 音色加载 →
NPU推理 → 音频生成 → 后处理 → 缓存存储 →
发布音频 → 等待下次输入
```

**详细时序：**
1. 接收LLM响应（异步）
2. 文本预处理（<10ms）
3. 加载音色模型（首次加载，后续缓存）
4. NPU/BPU加速合成（<1000ms for 10s audio）
5. 音频后处理（音量、音调调整）
6. 格式转换（如果需要）
7. 存储到缓存（避免重复合成）
8. 发布音频到 `/tts/output`

#### 流式合成流程
```
长文本输入 → 文本分段 → 逐段合成 →
音频拼接 → 流式输出 → 实时播放
```

**分段策略：**
- 按句子分割（标点符号）
- 最大段长度：100字符
- 段间重叠：<5字符
- 实时合成：段合成完成立即输出

#### 情感合成流程
```
文本情感分析 → 情感参数映射 →
情感特征提取 → 情感模型推理 →
音频生成 → 情感验证
```

**情感参数映射：**
- 开心：语速+10%，音调+5%，时长×0.9
- 悲伤：语速-15%，音调-10%，时长×1.2
- 愤怒：语速+20%，音调+15%，音量+10%
- 平静：语速-5%，音调-5%，音量-5%

## Non-Functional Requirements

### Performance

**延迟要求：**
- 音色加载时间：<2秒（首次）
- 合成启动延迟：<50ms
- 合成速度：>10倍实时（10秒音频<1秒合成）
- NPU模式：<1ms per character
- CPU模式：<1秒 per sentence

**吞吐量要求：**
- 并发合成：支持3个并发请求
- 音频输出：实时流式输出
- 缓冲区：支持5分钟音频缓存
- 缓存容量：100个文本片段

**资源使用限制：**
- CPU使用率：<50% (平均)
- 内存使用：<1GB (模型+缓存)
- NPU利用率：<80%
- 磁盘使用：<500MB (模型文件)

### Security

**数据保护：**
- 音频数据不长期存储（默认）
- 临时文件自动清理（1小时内）
- API密钥安全存储
- 传输加密：TLS 1.3（如使用网络API）

**访问控制：**
- ROS2主题级权限
- API调用频率限制
- 音色模型访问控制

**隐私保护：**
- 合成内容不记录日志
- 缓存数据加密存储（可选）
- 用户音频数据最小化

### Reliability/Availability

**可用性指标：**
- 系统可用性：>99.5%
- 平均故障间隔时间（MTBF）：>4380小时（半年）
- 平均修复时间（MTTR）：<30分钟

**容错机制：**
- NPU失败自动降级到CPU
- 音色加载失败使用默认音色
- 音频输出异常自动重试
- 节点故障自动重启

**故障恢复：**
- 优雅降级：NPU → CPU → 简化模式
- 错误隔离：单个音色失败不影响整体
- 状态持久化：合成参数持久化
- 自动恢复：服务重启后恢复状态

### Observability

**日志记录：**
- 合成任务日志（文本、参数、时长）
- 性能日志（延迟、吞吐量）
- 错误日志（失败原因、频率）
- 资源使用日志（CPU、内存、NPU）

**监控指标：**
- 合成延迟（平均、95%分位数）
- 音频质量（MOS评分，抽样评估）
- NPU使用率
- 缓存命中率
- 音色使用分布

**告警机制：**
- 合成延迟>2s告警
- CPU使用率>70%告警
- NPU错误率>5%告警
- 音频质量下降告警

**可观测性工具：**
- ROS2 rqt工具
- Prometheus指标采集
- Grafana监控Dashboard
- 自定义音频质量监控

## Dependencies and Integrations

**内部依赖：**
- ROS2 Humble中间件
- LLM模块响应结果
- 音频输出系统
- 系统控制模块

**外部依赖：**
- Piper VITS语音合成引擎
- zh_CN-huayan-medium模型
- BPU SDK（NPU加速）
- 音频处理库（librosa, soundfile）
- NumPy/SciPy数值计算

**版本约束：**
```
Python: >=3.10,<3.11
piper-tts: >=1.2.0
librosa: >=0.10.0,<0.11.0
soundfile: >=0.12.0
numpy: >=1.21.0
scipy: >=1.7.0
```

**集成点：**
- `/llm/response` - LLM服务（上游）
- `/tts/output` - 音频输出系统（下游）
- `/system/status` - 系统控制（状态同步）
- 音频设备接口 - ALSA/PulseAudio

## Acceptance Criteria (Authoritative)

**AC-1: 粤语语音合成质量**
- 语音自然度评分>4.0/5.0
- 测试方法：10名粤语母语者主观评分
- 测试内容：100段标准粤语文本
- 验收标准：平均评分≥4.0

**AC-2: 合成延迟性能**
- NPU模式：<1ms per character
- CPU模式：<1秒 per sentence
- 测试方法：100次合成任务测量
- 验收标准：平均延迟达标

**AC-3: 语速控制精度**
- 语速控制范围：0.5x-2.0x
- 控制精度误差：<5%
- 测试方法：多语速对比测试
- 验收标准：实测语速与设定值误差<5%

**AC-4: 音量控制精度**
- 音量控制范围：0-100级
- 控制精度：±2dB
- 测试方法：音频分析仪测量
- 验收标准：音量误差<2dB

**AC-5: 多种音色支持**
- 支持至少3种音色
- 音色切换响应时间：<100ms
- 测试方法：音色切换测试
- 验收标准：3种音色全部可用

**AC-6: 情感表达效果**
- 情感识别准确率>80%
- 情感表达自然度评分>3.5/5.0
- 测试方法：情感语音主观评分
- 验收标准：准确率和自然度达标

**AC-7: 音频质量指标**
- 采样率：22.05kHz
- 位深：16bit
- THD<0.1%（总谐波失真）
- SNR>60dB（信噪比）
- 测试方法：音频质量分析

**AC-8: 系统稳定性**
- 连续运行24小时无崩溃
- 内存使用稳定，无泄漏
- 合成质量无衰减
- 测试方法：长时间压力测试

## Traceability Mapping

| AC编号 | 规格章节 | 组件/API | 测试用例 | 测试方法 |
|--------|----------|----------|----------|----------|
| AC-1 | 详细设计-语音渲染 | VoiceRenderer | TC-TTS-001 | 音质主观评分 |
| AC-2 | 非功能性-性能 | TTSNPUAccelerator | TC-TTS-002 | 合成延迟测试 |
| AC-3 | 详细设计-音频后处理 | AudioPostProcessor | TC-TTS-003 | 语速控制测试 |
| AC-4 | 详细设计-音频后处理 | AudioPostProcessor | TC-TTS-004 | 音量控制测试 |
| AC-5 | 详细设计-音色管理 | VoiceManager | TC-TTS-005 | 音色切换测试 |
| AC-6 | 详细设计-情感引擎 | EmotionEngine | TC-TTS-006 | 情感表达测试 |
| AC-7 | 详细设计-数据模型 | TTSAudioData | TC-TTS-007 | 音频质量测试 |
| AC-8 | 非功能性-可靠性 | TTSServiceNode | TC-TTS-008 | 稳定性测试 |

**需求追踪矩阵：**
```
PRD需求 TTS-001 → AC-1, AC-2, AC-7
PRD需求 TTS-002 → AC-3
PRD需求 TTS-003 → AC-4
PRD需求 TTS-004 → AC-6
Story 3.1 → AC-2, AC-7
Story 3.2 → AC-1
Story 3.3 → AC-5
Story 3.4 → AC-6
Story 3.5 → 接口测试
```

## Risks, Assumptions, Open Questions

**技术风险：**
1. **R-001: 语音模型训练和优化风险**
   - 风险等级：高
   - 概率：30%
   - 影响：严重（音质不达标）
   - 缓解措施：模型微调、质量调优、多模型对比
   - 应急计划：降低验收标准至3.5分

2. **R-002: NPU集成性能优化风险**
   - 风险等级：中
   - 概率：40%
   - 影响：中等（性能提升不足）
   - 缓解措施：ONNX优化、算子融合、性能调优
   - 应急计划：纯CPU模式，性能目标调整

3. **R-003: 情感表达准确性风险**
   - 风险等级：中
   - 概率：50%
   - 影响：中等（情感效果不佳）
   - 缓解措施：情感参数优化、人工标注数据
   - 应急计划：简化情感模型，降低期望

**假设条件：**
1. **A-001:** Piper VITS引擎稳定可用，支持NPU加速
2. **A-002:** zh_CN-huayan-medium模型质量满足要求
3. **A-003:** 音频输出设备（扬声器）质量良好
4. **A-004:** NPU加速能够显著提升性能

**开放问题：**
1. **Q-001:** 是否需要支持自定义音色？（当前预置3种）
2. **Q-002:** 音频缓存策略？（内存vs磁盘）
3. **Q-003:** 实时变声的必要性？（当前不支持）
4. **Q-004:** 离线语音合成的范围？（当前设计为离线）

## Test Strategy Summary

**测试级别和范围：**

1. **单元测试（Unit Testing）**
   - 覆盖率目标：>80%
   - 测试范围：音色管理、参数调整、缓存机制
   - 工具：pytest, mock音频
   - 频率：每次代码提交

2. **集成测试（Integration Testing）**
   - 测试范围：LLM→TTS→Audio输出端到端
   - TTS引擎集成测试
   - 工具：ROS2测试框架
   - 频率：每日构建

3. **系统测试（System Testing）**
   - 测试范围：完整TTS功能
   - 测试内容：性能、质量、稳定性、兼容性
   - 测试环境：RDK X5 + 真实音频设备
   - 频率：版本发布前

4. **用户验收测试（UAT）**
   - 测试人员：Beta用户（粤语用户）
   - 测试内容：真实语音场景
   - 验收标准：符合AC-1至AC-8所有标准
   - 频率：Beta测试阶段

**测试方法：**

- **音质测试：** MOS评分（主观评价）
- **性能测试：** 合成延迟测量（平均、95%分位数）
- **语速测试：** 多语速对比（0.5x-2.0x）
- **音量测试：** 音频分析仪测量
- **情感测试：** 情感语音主观评分
- **稳定性测试：** 24小时连续合成
- **兼容性测试：** 多种音频设备

**测试环境：**
- 硬件：RDK X5开发套件
- 音频设备：USB扬声器、3.5mm音频接口
- 软件：Ubuntu 22.04 + ROS2 Humble
- 音频分析工具：Audacity、FFmpeg

**验收测试矩阵：**
| 测试类型 | 目标指标 | 测试数据 | 验收标准 |
|----------|----------|----------|----------|
| 音质 | 评分>4.0 | 100段文本 | 平均≥4.0 |
| 性能 | <1ms/字 | 100次合成 | 95%分位达标 |
| 语速 | 误差<5% | 多语速测试 | 实测达标 |
| 音量 | ±2dB | 多音量测试 | 误差达标 |
| 音色 | 3种音色 | 切换测试 | 全部可用 |
| 情感 | 准确率>80% | 情感样本 | 达标 |
| 质量 | 指标达标 | 音频分析 | 全部达标 |
| 稳定 | 24h运行 | 压力测试 | 无故障 |

---
**文档版本：** v1.0
**最后更新：** 2025-11-02
**BMad Method** - 专业软件工程方法论
