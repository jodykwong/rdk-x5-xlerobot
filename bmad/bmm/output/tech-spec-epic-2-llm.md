# Technical Specification: 智能对话模块 (LLM)

Date: 2025-11-02
Author: BMad Method Tech Architect
Epic ID: 2
Status: Draft

---

## Overview

智能对话模块（LLM）是XLeRobot智能语音机器人系统的核心大脑，负责自然语言理解、对话生成和上下文管理。该模块基于通义千问API（DashScope）构建，支持粤语对话模式，能够理解用户意图并生成自然、流畅的回复。通过集成先进的大语言模型（qwen-plus/qwen-max），模块实现了4000 tokens的上下文窗口，支持多轮对话记忆，提供个性化的对话体验。该设计严格遵循ROS2分布式架构，提供标准化的服务接口，确保与ASR、TTS和系统控制模块的高效协同工作。

## Objectives and Scope

### In-Scope功能
- 通义千问API集成和调用管理（优先级P0）
- 自然语言理解和意图分类（支持粤语）
- 对话生成和响应优化（上下文感知）
- 多轮对话上下文管理和记忆
- 个性化对话定制（角色、偏好）
- 内容安全过滤和审核

### Out-of-Scope功能
- 自主大语言模型训练
- 私有化部署的LLM服务
- 图像理解（多模态）
- 复杂知识图谱推理
- 实时网络信息查询（外部搜索）

### 技术边界
- API依赖：通义千问API（DashScope）
- 模型选择：qwen-plus / qwen-max
- 响应时间：<3秒（网络正常）
- 上下文窗口：4000 tokens
- 并发能力：<10 QPS
- 语言支持：粤语为主，普通话、英语为辅

## System Architecture Alignment

LLM模块在软件架构中的位置：
- 位于应用服务层，与ASR、TTS服务并列
- 作为下游服务，订阅ASR结果
- 作为上游服务，发布响应给TTS
- 通过ROS2中间件进行异步通信

组件对齐：
- 订阅 `/asr/result` 主题（来自ASR模块）
- 发布 `/llm/response` 主题（到TTS模块）
- 与系统控制模块共享 `/system/status` 状态
- 使用ROS2服务调用进行同步查询

架构约束：
- 必须支持API失败降级机制
- 缓存机制减少API调用
- 异步处理提高并发能力
- 符合GDPR/PIPL数据保护要求

## Detailed Design

### Services and Modules

| 服务/模块 | 职责 | 输入 | 输出 | 负责人 |
|----------|------|------|------|--------|
| QwenAPIClient | 通义千问API调用管理 | ASR结果 | LLM响应 | 后端工程师 |
| NLUProcessor | 自然语言理解 | 用户文本 | 意图+实体 | NLP工程师 |
| DialogueManager | 对话上下文管理 | 历史对话 | 上下文窗口 | 后端工程师 |
| ContentFilter | 内容安全过滤 | LLM回复 | 过滤后内容 | 安全工程师 |
| PromptBuilder | 提示词构建 | 上下文+用户输入 | 完整Prompt | NLP工程师 |
| LLMServiceNode | ROS2服务节点 | ASR消息 | LLM消息 | 后端工程师 |
| ResponseOptimizer | 响应优化 | LLM回复 | 优化后回复 | NLP工程师 |

**LLM核心引擎规格：**
- 主要模型：qwen-plus（标准版）
- 高级模型：qwen-max（复杂查询）
- 最大上下文：4000 tokens
- 最大输出：2048 tokens
- 支持语言：中文、英文、粤语
- API调用方式：HTTP RESTful

### Data Models and Contracts

```python
# LLM服务输入消息（订阅ASR结果）
ASRResult:
  - text: str  # 用户输入文本
  - confidence: float32  # ASR置信度
  - processing_time: float32  # ASR处理时间
  - language: str  # "cantonese", "mandarin", "english"
  - is_final: bool  # 是否为最终结果
  - timestamp: int64  # 识别完成时间

# LLM响应消息（发布给TTS）
LLMResponse:
  - text: str  # 生成的回复文本
  - confidence: float32  # 回复置信度
  - processing_time: float32  # LLM处理时间
  - tokens_used: int  # 使用的token数
  - intent: str  # 意图分类结果
  - entities: List[str]  # 提取的实体
  - is_error: bool  # 是否为错误回复
  - context_id: int64  # 对话上下文ID

# 对话上下文管理
DialogueContext:
  - context_id: int64  # 上下文ID
  - user_id: str  # 用户标识
  - messages: List[Message]  # 历史消息列表
  - created_time: int64  # 创建时间
  - last_access: int64  # 最后访问时间
  - ttl: int  # 生存时间（秒）

Message:
  - role: str  # "system", "user", "assistant"
  - content: str  # 消息内容
  - timestamp: int64  # 时间戳
  - tokens: int  # token数

# 意图分类结果
IntentClassification:
  - intent: str  # 主要意图
  - confidence: float32  # 置信度
  - sub_intents: List[str]  # 子意图
  - entities: Dict[str, Any]  # 实体提取结果
  - sentiment: str  # 情感分析

# 内容过滤结果
ContentFilterResult:
  - is_safe: bool  # 内容是否安全
  - risk_level: str  # "low", "medium", "high"
  - filtered_words: List[str]  # 过滤的敏感词
  - action: str  # 处理动作：allow, filter, block
```

### APIs and Interfaces

#### ROS2服务接口
**服务名称：** `/xlerobot/llm_service`
**消息类型：** 自定义消息包

**发布主题：**
- `/llm/response` - 发布LLM响应（到TTS）
- `/llm/status` - 发布LLM服务状态

**订阅主题：**
- `/asr/result` - 订阅ASR识别结果

**服务调用：**
- `generate_response()` - 同步生成回复
- `manage_context()` - 管理对话上下文
- `classify_intent()` - 意图分类
- `filter_content()` - 内容过滤
- `set_persona()` - 设置对话角色

#### RESTful API接口（可选）
```
POST /api/v1/llm/chat
Content-Type: application/json

Request Body:
{
  "user_input": "用户输入文本",
  "context_id": 12345,
  "persona": "friendly_assistant",
  "language": "cantonese"
}

Response:
{
  "status": "success",
  "response": "LLM生成的回复",
  "intent": "smart_home_control",
  "confidence": 0.92,
  "processing_time_ms": 2500,
  "context_id": 12345
}
```

#### 通义千问API集成
**API端点：** https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation
**认证方式：** API Key (DashScope)

```python
# API请求格式
request_payload = {
  "model": "qwen-plus",
  "input": {
    "messages": [
      {
        "role": "system",
        "content": cantonese_system_prompt
      },
      {
        "role": "user",
        "content": user_input
      }
    ]
  },
  "parameters": {
    "max_tokens": 4000,
    "temperature": 0.7,
    "top_p": 0.8,
    "repetition_penalty": 1.1
  }
}

# API响应格式
response_payload = {
  "output": {
    "choices": [
      {
        "message": {
          "role": "assistant",
          "content": "回复文本"
        }
      }
    ]
  },
  "usage": {
    "input_tokens": 150,
    "output_tokens": 200
  }
}
```

**重试策略：**
- 最大重试次数：3次
- 退避策略：指数退避（1s, 2s, 4s）
- 错误分类：网络错误重试，API限额不重试
- 降级方案：API失败时返回预设回复

#### 错误码定义
| 错误码 | 说明 | 处理策略 |
|--------|------|----------|
| LLM_001 | API Key无效或过期 | 返回配置错误，提示用户配置 |
| LLM_002 | API调用超时 | 重试机制，退避策略 |
| LLM_003 | 请求频率超限 | 降级处理，返回缓存结果 |
| LLM_004 | 服务不可用 | 离线模式，预设回复 |
| LLM_005 | 上下文超长 | 自动截断，保持最新上下文 |

### Workflows and Sequencing

#### 主对话流程
```
ASR识别文本 → 意图分类 → 上下文管理 →
Prompt构建 → API调用 → 响应生成 → 内容过滤 →
上下文更新 → 发布结果
```

**详细时序：**
1. 接收ASR识别结果（异步）
2. 意图分类和实体提取（<100ms）
3. 检索对话上下文（上下文ID）
4. 构建完整Prompt（系统提示+历史+用户输入）
5. 调用通义千问API（<3s）
6. 解析API响应
7. 内容安全过滤（<50ms）
8. 更新对话上下文
9. 发布结果到 `/llm/response`

#### 上下文管理流程
```
新对话 → 创建Context → 存储Context →
消息处理 → 添加历史 → 检查长度 →
截断处理 → 持久化 → 返回上下文
```

**上下文窗口管理：**
- 窗口大小：4000 tokens
- 截断策略：保持最新对话，删除最老消息
- 缓存策略：本地缓存（Redis） + 持久化存储
- 清理机制：TTL 24小时未访问自动删除

## Non-Functional Requirements

### Performance

**延迟要求：**
- 意图分类延迟：<100ms
- Prompt构建延迟：<50ms
- API调用时间：<3秒（网络正常）
- 内容过滤延迟：<50ms
- 总体响应时间：<3.5秒

**吞吐量要求：**
- 并发用户：1个本地用户
- API调用频率：<10 QPS
- 上下文管理：支持100个并发上下文
- 缓存命中：>80%（重复问题）

**资源使用限制：**
- CPU使用率：<40% (平均)
- 内存使用：<1GB (上下文缓存)
- 网络带宽：<100KB/请求
- API调用成本：<0.01元/次

### Security

**API安全：**
- API Key安全存储（环境变量/密钥管理）
- API调用频率限制（防滥用）
- 请求签名验证（如适用）
- 传输加密：TLS 1.3

**内容安全：**
- 输入内容过滤（敏感词检测）
- 输出内容审核（不当内容拦截）
- 隐私保护：不存储敏感对话内容
- 数据脱敏：日志中隐藏敏感信息

**访问控制：**
- 服务访问权限控制
- 上下文隔离（多用户）
- API配额管理
- 审计日志记录

### Reliability/Availability

**可用性指标：**
- 系统可用性：>99%
- 平均故障间隔时间（MTBF）：>720小时（1月）
- 平均修复时间（MTTR）：<1小时

**容错机制：**
- API失败自动重试（最多3次）
- 服务降级：API失败 → 缓存回复 → 预设回复
- 上下文持久化：防止丢失
- 健康检查：定期检查API状态

**故障恢复：**
- API不可用：切换到离线模式
- 网络中断：请求重试 + 本地缓存
- 数据损坏：从持久化存储恢复
- 错误隔离：单个上下文失败不影响整体

### Observability

**日志记录：**
- API调用日志（响应时间、成功率）
- 错误日志（分类统计）
- 上下文管理日志（创建、更新、清理）
- 内容过滤日志（拦截统计）

**监控指标：**
- API调用成功率（目标>99%）
- 平均响应时间（<3s）
- 上下文命中率（>80%）
- 缓存使用率
- 内容过滤拦截率

**告警机制：**
- API成功率<95%告警
- 响应时间>5s告警
- 错误率>5%告警
- API调用成本超预算告警

**可观测性工具：**
- Prometheus指标采集
- Grafana监控Dashboard
- ELK Stack日志分析
- 自定义业务监控

## Dependencies and Integrations

**内部依赖：**
- ROS2 Humble中间件
- ASR模块识别结果
- TTS模块语音合成
- 系统控制模块状态管理

**外部依赖：**
- 通义千问API（DashScope）
- Python DashScope SDK
- Redis（缓存，可选）
- requests/HTTP库
- 敏感词过滤库

**版本约束：**
```
Python: >=3.10,<3.11
dashscope: >=1.0.0
requests: >=2.28.0,<3.0.0
redis: >=4.0.0 (可选)
```

**集成点：**
- `/asr/result` - ASR服务（上游）
- `/llm/response` - TTS服务（下游）
- `/system/status` - 系统控制服务（状态同步）
- DashScope API - 外部LLM服务

## Acceptance Criteria (Authoritative)

**AC-1: API集成稳定性**
- API调用成功率≥99%
- 网络超时重试成功率≥90%
- 测试方法：连续1000次API调用
- 验收标准：成功率≥99%

**AC-2: 响应时间性能**
- 平均响应时间<3秒
- 95%分位数<3.5秒
- 测试方法：100次连续调用测量
- 验收标准：平均时间和95%分位数均达标

**AC-3: 自然语言理解准确性**
- 意图识别准确率≥90%
- 实体抽取准确率≥85%
- 测试数据：500条粤语指令
- 验收方法：人工标注对比

**AC-4: 粤语对话质量**
- 粤语回复自然度评分>4.0/5.0
- 粤语文化适配性评分>4.2/5.0
- 测试方法：10名粤语母语者评分
- 验收标准：平均分≥4.0

**AC-5: 上下文记忆能力**
- 支持连续20轮对话
- 上下文一致性≥95%
- 测试方法：20轮连续对话测试
- 验收标准：所有回合上下文保持准确

**AC-6: 个性化对话效果**
- 个性化响应满意度>4.0/5.0
- 角色一致性评分>4.2/5.0
- 测试方法：用户主观评价
- 验收标准：评分达标

**AC-7: 内容安全过滤**
- 不当内容拦截率100%
- 误拦截率<5%
- 测试数据：1000条敏感内容样本
- 验收标准：拦截率和误拦截率达标

**AC-8: 并发处理能力**
- 支持10 QPS并发调用
- 响应时间退化<20%
- 测试方法：压力测试（10并发×100请求）
- 验收标准：平均响应时间<3.6s

## Traceability Mapping

| AC编号 | 规格章节 | 组件/API | 测试用例 | 测试方法 |
|--------|----------|----------|----------|----------|
| AC-1 | 详细设计-API集成 | QwenAPIClient | TC-LLM-001 | API稳定性测试 |
| AC-2 | 非功能性-性能 | LLMServiceNode | TC-LLM-002 | 响应时间测试 |
| AC-3 | 详细设计-服务模块 | NLUProcessor | TC-LLM-003 | NLU准确性测试 |
| AC-4 | 详细设计-响应优化 | ResponseOptimizer | TC-LLM-004 | 粤语质量测试 |
| AC-5 | 详细设计-对话管理 | DialogueManager | TC-LLM-005 | 上下文记忆测试 |
| AC-6 | 详细设计-提示构建 | PromptBuilder | TC-LLM-006 | 个性化测试 |
| AC-7 | 详细设计-内容过滤 | ContentFilter | TC-LLM-007 | 内容安全测试 |
| AC-8 | 非功能性-性能 | LLMServiceNode | TC-LLM-008 | 并发性能测试 |

**需求追踪矩阵：**
```
PRD需求 LLM-001 → AC-3
PRD需求 LLM-002 → AC-4
PRD需求 LLM-003 → AC-5
PRD需求 LLM-004 → AC-4, AC-6
PRD需求 LLM-005 → AC-7
Story 2.1 → AC-1, AC-2
Story 2.2 → AC-5
Story 2.3 → AC-3
Story 2.4 → AC-6
Story 2.5 → AC-7
```

## Risks, Assumptions, Open Questions

**技术风险：**
1. **R-001: 通义千问API稳定性风险**
   - 风险等级：高
   - 概率：30%
   - 影响：严重（核心功能不可用）
   - 缓解措施：重试机制、降级方案、多API提供商
   - 应急计划：切换到OpenAI API或本地LLM

2. **R-002: API成本和限额风险**
   - 风险等级：中
   - 概率：40%
   - 影响：中等（运营成本增加）
   - 缓解措施：缓存机制、频率限制、成本监控
   - 应急计划：优化提示词减少token消耗

3. **R-003: 粤语理解准确性不足**
   - 风险等级：中
   - 概率：30%
   - 影响：中等（用户体验下降）
   - 缓解措施：定制粤语提示词、训练数据扩充
   - 应急计划：降低验收标准，人工确认机制

**假设条件：**
1. **A-001:** 通义千问API在项目周期内保持稳定和政策不变
2. **A-002:** 网络环境稳定，API调用延迟<3s
3. **A-003:** API配额充足，满足正常使用需求
4. **A-004:** 粤语prompt优化能够达到预期效果

**开放问题：**
1. **Q-001:** 是否需要支持多个LLM提供商？（当前仅通义千问）
2. **Q-002:** 本地LLM部署的可行性？（成本vs隐私权衡）
3. **Q-003:** 敏感内容过滤策略？（本地vs云端）
4. **Q-004:** 上下文缓存存储位置？（内存vs磁盘）

## Test Strategy Summary

**测试级别和范围：**

1. **单元测试（Unit Testing）**
   - 覆盖率目标：>80%
   - 测试范围：意图分类、提示构建、内容过滤、上下文管理
   - 工具：pytest, mock API
   - 频率：每次代码提交

2. **集成测试（Integration Testing）**
   - 测试范围：ASR→LLM→TTS端到端流程
   - API集成测试：DashScope API调用
   - 工具：ROS2测试框架，API模拟器
   - 频率：每日构建

3. **系统测试（System Testing）**
   - 测试范围：完整LLM功能
   - 测试内容：性能、准确率、稳定性、安全性
   - 测试环境：RDK X5 + 真实API
   - 频率：版本发布前

4. **用户验收测试（UAT）**
   - 测试人员：Beta用户（粤语用户）
   - 测试内容：真实对话场景
   - 验收标准：符合AC-1至AC-8所有标准
   - 频率：Beta测试阶段

**测试方法：**

- **API稳定性测试：** 连续调用1000次，记录成功率
- **性能测试：** 响应时间测量（平均、95%分位数）
- **准确性测试：** 意图分类准确率、实体提取准确率
- **对话质量测试：** 人工评分（自然度、文化适配性）
- **安全测试：** 内容过滤测试（敏感词拦截率）
- **压力测试：** 并发调用（10 QPS），资源使用监控

**测试环境：**
- 硬件：RDK X5开发套件
- 软件：Ubuntu 22.04 + ROS2 Humble
- 外部服务：DashScope API（测试环境）
- 网络：稳定的互联网连接

**验收测试矩阵：**
| 测试类型 | 目标指标 | 测试数据 | 验收标准 |
|----------|----------|----------|----------|
| API稳定 | 成功率>99% | 1000次调用 | ≥99%成功率 |
| 响应时间 | 平均<3s | 100次调用 | 95%分位<3.5s |
| NLU准确 | 意图>90% | 500条指令 | ≥90%准确率 |
| 对话质量 | 评分>4.0 | 10人评分 | 平均≥4.0 |
| 上下文记忆 | 20轮对话 | 连续对话 | 95%一致性 |
| 内容安全 | 拦截率100% | 1000样本 | 拦截≥99% |
| 并发性能 | 10 QPS | 压力测试 | 退化<20% |

---
**文档版本：** v1.0
**最后更新：** 2025-11-02
**BMad Method** - 专业软件工程方法论
