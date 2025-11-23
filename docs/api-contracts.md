# XleRobot API契约文档

**文档编号**: XLR-API-P0-20251107-001
**项目名称**: XleRobot 家用机器人控制系统
**文档类型**: API契约文档
**生成日期**: 2025-11-07
**工作流**: Phase 0 Documentation - document-project

---

## 📋 概述

本文档定义XleRobot系统的完整API契约，包括ROS2话题接口、服务接口、Web API接口和内部Python API。为系统集成、模块开发和API维护提供标准化的接口规范。

### API分类
- **ROS2 API**: 话题、服务、动作接口
- **Web API**: HTTP REST接口
- **WebSocket API**: 实时通信接口
- **Python API**: 内部模块接口

---

## 🤖 ROS2 API接口

### 1. 话题接口 (Topics)

#### 1.1 音频输入话题
**话题名称**: `/audio_input`
**消息类型**: `audio_common_msgs/AudioData`
**方向**: 发布 (Publisher)
**频率**: 16kHz
**描述**: 原始音频输入数据流

```yaml
消息定义 (AudioData):
  data:
    type: uint8[]
    description: 音频数据字节流
    format: PCM 16-bit
  format:
    type: string
    description: 音频格式 (wav/pcm/flac)
  sample_rate:
    type: uint32
    description: 采样率 (Hz)
  channels:
    type: uint8
    description: 声道数
  frame_size:
    type: uint32
    description: 帧大小
```

**使用示例**:
```python
from audio_common_msgs.msg import AudioData

# 发布音频数据
audio_msg = AudioData()
audio_msg.data = audio_bytes
audio_msg.format = "wav"
audio_msg.sample_rate = 16000
audio_msg.channels = 1

publisher.publish(audio_msg)
```

#### 1.2 ASR结果话题
**话题名称**: `/asr_result`
**消息类型**: `std_msgs/String`
**方向**: 发布 (Publisher)
**频率**: 按需发布
**描述**: 语音识别结果文本

```yaml
消息定义 (String):
  data:
    type: string
    description: 识别出的文本内容
```

#### 1.3 LLM响应话题
**话题名称**: `/llm_response`
**消息类型**: `std_msgs/String`
**方向**: 发布 (Publisher)
**频率**: 按需发布
**描述**: 大语言模型生成的回复

#### 1.4 系统状态话题
**话题名称**: `/system_status`
**消息类型**: `xlerobot_msgs/SystemStatus`
**方向**: 发布 (Publisher)
**频率**: 1Hz
**描述**: 系统运行状态信息

```yaml
消息定义 (SystemStatus):
  state:
    type: string
    description: 当前系统状态
    enum: [idle, listening, processing, speaking, error, maintenance]
  battery_level:
    type: float32
    description: 电池电量百分比
  cpu_usage:
    type: float32
    description: CPU使用率
  memory_usage:
    type: float32
    description: 内存使用率
  active_components:
    type: string[]
    description: 活跃组件列表
  error_count:
    type: uint32
    description: 错误计数
```

#### 1.5 命令话题
**话题名称**: `/command`
**消息类型**: `xlerobot_msgs/Command`
**方向**: 订阅 (Subscriber)
**频率**: 按需接收
**描述**: 系统控制命令

```yaml
消息定义 (Command):
  command:
    type: string
    description: 命令名称
    enum: [start, stop, pause, resume, shutdown, restart, configure]
  parameters:
    type: string
    description: 命令参数 (JSON格式)
  timestamp:
    type: time
    description: 命令时间戳
  request_id:
    type: string
    description: 请求唯一标识
```

### 2. 服务接口 (Services)

#### 2.1 语音识别服务
**服务名称**: `/asr_service`
**服务类型**: `xlerobot_srvs/ASRService`
**描述**: 同步语音识别服务

```yaml
请求定义 (ASRRequest):
  audio_data:
    type: uint8[]
    description: 音频数据
  language:
    type: string
    description: 识别语言 (cantonese/mandarin/english)
  provider:
    type: string
    description: ASR提供商 (alibaba/local)

响应定义 (ASRResponse):
  success:
    type: bool
    description: 识别是否成功
  text:
    type: string
    description: 识别结果文本
  confidence:
    type: float32
    description: 识别置信度 (0-1)
  processing_time:
    type: float32
    description: 处理时间 (秒)
  error_message:
    type: string
    description: 错误信息 (如果失败)
```

#### 2.2 语音合成服务
**服务名称**: `/tts_service`
**服务类型**: `xlerobot_srvs/TTSService`
**描述**: 同步语音合成服务

```yaml
请求定义 (TTSRequest):
  text:
    type: string
    description: 要合成的文本
  voice:
    type: string
    description: 语音类型
  speed:
    type: float32
    description: 语速 (0.5-2.0)
  pitch:
    type: float32
    description: 音调 (0.5-2.0)

响应定义 (TTSResponse):
  success:
    type: bool
    description: 合成是否成功
  audio_data:
    type: uint8[]
    description: 合成的音频数据
  format:
    type: string
    description: 音频格式
  duration:
    type: float32
    description: 音频时长 (秒)
  error_message:
    type: string
    description: 错误信息 (如果失败)
```

#### 2.3 对话服务
**服务名称**: `/conversation_service`
**服务类型**: `xlerobot_srvs/ConversationService`
**描述**: 对话交互服务

```yaml
请求定义 (ConversationRequest):
  message:
    type: string
    description: 用户消息
  conversation_id:
    type: string
    description: 对话ID (可选)
  user_id:
    type: string
    description: 用户ID
  context:
    type: string
    description: 上下文信息 (JSON格式)

响应定义 (ConversationResponse):
  success:
    type: bool
    description: 处理是否成功
  response:
    type: string
    description: 系统回复
  conversation_id:
    type: string
    description: 对话ID
  suggested_actions:
    type: string[]
    description: 建议的后续动作
  emotion:
    type: string
    description: 情感识别结果
  error_message:
    type: string
    description: 错误信息 (如果失败)
```

### 3. 动作接口 (Actions)

#### 3.1 长时间ASR动作
**动作名称**: `/long_asr`
**动作类型**: `xlerobot_actions/LongASRAction`
**描述**: 长时间语音识别动作

```yaml
目标定义 (LongASRGoal):
  duration:
    type: float32
    description: 识别时长 (秒)
  language:
    type: string
    description: 识别语言
  sensitivity:
    type: float32
    description: 识别灵敏度

结果定义 (LongASRResult):
  final_text:
    type: string
    description: 最终识别结果
  intermediate_results:
    type: string[]
    description: 中间结果列表
  confidence:
    type: float32
    description: 置信度
  total_duration:
    type: float32
    description: 总识别时长

反馈定义 (LongASRFeedback):
  current_text:
    type: string
    description: 当前识别结果
  is_speaking:
    type: bool
    description: 是否正在说话
  volume_level:
    type: float32
    description: 音量级别
```

---

## 🌐 Web API接口

### 1. 认证接口

#### 1.1 用户认证
**端点**: `POST /api/v1/auth/login`
**描述**: 用户登录认证

```http
请求体:
{
  "username": "string",
  "password": "string"
}

响应体 (200 OK):
{
  "success": true,
  "token": "jwt_token_string",
  "user_id": "user_uuid",
  "expires_in": 3600,
  "permissions": ["speak", "listen", "control"]
}

错误响应 (401 Unauthorized):
{
  "success": false,
  "error": "invalid_credentials",
  "message": "用户名或密码错误"
}
```

#### 1.2 Token刷新
**端点**: `POST /api/v1/auth/refresh`
**描述**: 刷新访问令牌

```http
请求头:
  Authorization: Bearer <refresh_token>

响应体 (200 OK):
{
  "success": true,
  "token": "new_jwt_token_string",
  "expires_in": 3600
}
```

### 2. 语音处理接口

#### 2.1 语音识别
**端点**: `POST /api/v1/speech/recognize`
**描述**: 上传音频文件进行语音识别

```http
请求体 (multipart/form-data):
  audio_file: <audio_file>
  language: cantonese (可选)
  provider: alibaba (可选)

响应体 (200 OK):
{
  "success": true,
  "text": "识别到的文本内容",
  "confidence": 0.95,
  "processing_time": 1.2,
  "timestamp": "2025-11-07T10:30:00Z"
}

错误响应 (400 Bad Request):
{
  "success": false,
  "error": "invalid_audio_format",
  "message": "不支持的音频格式"
}
```

#### 2.2 语音合成
**端点**: `POST /api/v1/speech/synthesize`
**描述**: 文本转语音

```http
请求体:
{
  "text": "要合成的文本内容",
  "voice": "cantonese_female",
  "speed": 1.0,
  "pitch": 1.0,
  "format": "wav"
}

响应体 (200 OK):
{
  "success": true,
  "audio_url": "https://api.example.com/audio/generated_file.wav",
  "duration": 3.5,
  "file_size": 56032,
  "expires_at": "2025-11-08T10:30:00Z"
}

错误响应 (400 Bad Request):
{
  "success": false,
  "error": "text_too_long",
  "message": "文本长度超过限制"
}
```

### 3. 对话接口

#### 3.1 发送消息
**端点**: `POST /api/v1/conversation/message`
**描述**: 发送消息进行对话

```http
请求头:
  Authorization: Bearer <access_token>

请求体:
{
  "message": "用户输入的消息",
  "conversation_id": "conversation_uuid" (可选),
  "context": {
    "emotion": "happy",
    "location": "living_room"
  }
}

响应体 (200 OK):
{
  "success": true,
  "response": "系统回复内容",
  "conversation_id": "conversation_uuid",
  "message_id": "message_uuid",
  "emotion": "friendly",
  "suggested_actions": [
    "播放音乐",
    "查询天气",
    "设置闹钟"
  ],
  "timestamp": "2025-11-07T10:30:00Z"
}
```

#### 3.2 获取对话历史
**端点**: `GET /api/v1/conversation/{conversation_id}/history`
**描述**: 获取对话历史记录

```http
查询参数:
  limit: 20 (可选)
  offset: 0 (可选)

响应体 (200 OK):
{
  "success": true,
  "conversation_id": "conversation_uuid",
  "messages": [
    {
      "id": "message_uuid",
      "role": "user",
      "content": "用户消息",
      "timestamp": "2025-11-07T10:25:00Z"
    },
    {
      "id": "message_uuid",
      "role": "assistant",
      "content": "系统回复",
      "timestamp": "2025-11-07T10:26:00Z"
    }
  ],
  "total_count": 15,
  "has_more": true
}
```

### 4. 系统控制接口

#### 4.1 系统状态查询
**端点**: `GET /api/v1/system/status`
**描述**: 获取系统状态

```http
响应体 (200 OK):
{
  "success": true,
  "status": {
    "state": "idle",
    "battery_level": 0.85,
    "cpu_usage": 0.25,
    "memory_usage": 0.45,
    "active_components": ["asr", "llm", "tts"],
    "uptime": 3600,
    "error_count": 0
  },
  "capabilities": {
    "speech_recognition": true,
    "text_to_speech": true,
    "conversation": true,
    "vision": false,
    "smart_home": true
  }
}
```

#### 4.2 系统控制
**端点**: `POST /api/v1/system/control`
**描述**: 控制系统状态

```http
请求体:
{
  "command": "restart",
  "target": "asr_service",
  "parameters": {
    "force": true
  }
}

响应体 (200 OK):
{
  "success": true,
  "message": "ASR服务重启成功",
  "execution_time": 2.5,
  "new_status": "running"
}
```

---

## 🔌 WebSocket API接口

### 1. 实时语音通信
**端点**: `ws://localhost:8080/ws/speech`
**描述**: 实时语音数据流通信

#### 1.1 连接认证
```javascript
// 连接时发送认证消息
{
  "type": "auth",
  "token": "jwt_token_string"
}

// 认证响应
{
  "type": "auth_response",
  "success": true,
  "session_id": "session_uuid"
}
```

#### 1.2 音频数据传输
```javascript
// 发送音频数据
{
  "type": "audio_data",
  "data": "base64_encoded_audio",
  "format": "wav",
  "sample_rate": 16000
}

// 接收识别结果
{
  "type": "recognition_result",
  "text": "识别到的文本",
  "confidence": 0.95,
  "is_final": true
}
```

### 2. 实时对话通信
**端点**: `ws://localhost:8080/ws/conversation`
**描述**: 实时对话消息通信

#### 2.1 消息格式
```javascript
// 发送用户消息
{
  "type": "user_message",
  "content": "用户输入的消息",
  "conversation_id": "conversation_uuid"
}

// 接收系统回复
{
  "type": "assistant_message",
  "content": "系统回复内容",
  "conversation_id": "conversation_uuid",
  "message_id": "message_uuid",
  "emotion": "friendly"
}

// 接收状态更新
{
  "type": "status_update",
  "status": "processing",
  "message": "正在处理您的请求..."
}
```

---

## 🐍 Python API接口

### 1. 核心组件接口

#### 1.1 ASR核心接口
```python
# src/modules/asr/asr_core.py
from abc import ABC, abstractmethod
from typing import Optional, Callable
from dataclasses import dataclass

@dataclass
class ASRResult:
    text: str
    confidence: float
    processing_time: float
    provider: str

class ASRProvider(ABC):
    @abstractmethod
    def recognize(self, audio_data: bytes, **kwargs) -> ASRResult:
        """识别音频数据"""
        pass

class ASRCore:
    def __init__(self, config: ASRConfig):
        """初始化ASR核心组件"""

    def recognize_audio(self, audio_data: bytes, format: str = "wav") -> ASRResult:
        """识别音频数据，返回识别结果"""

    def start_streaming(self, callback: Callable[[ASRResult], None]) -> None:
        """启动流式识别"""

    def stop_streaming(self) -> None:
        """停止流式识别"""

    def set_provider(self, provider: ASRProvider) -> None:
        """设置ASR提供商"""
```

#### 1.2 LLM核心接口
```python
# src/modules/llm/llm_core.py
from dataclasses import dataclass
from typing import List, Dict, Optional
from abc import ABC, abstractmethod

@dataclass
class LLMResponse:
    text: str
    conversation_id: str
    message_id: str
    emotion: Optional[str] = None
    suggested_actions: Optional[List[str]] = None
    processing_time: Optional[float] = None

@dataclass
class Context:
    conversation_history: List[Dict]
    user_id: str
    session_data: Dict
    current_state: str

class LLMProvider(ABC):
    @abstractmethod
    def generate(self, prompt: str, context: Optional[Context] = None) -> LLMResponse:
        """生成回复"""
        pass

class LLMCore:
    def __init__(self, config: LLMConfig):
        """初始化LLM核心组件"""

    def generate_response(self, prompt: str, context: Optional[Context] = None) -> LLMResponse:
        """生成回复"""

    def start_conversation(self, user_id: str) -> str:
        """开始新对话，返回对话ID"""

    def continue_conversation(self, conversation_id: str, message: str) -> LLMResponse:
        """继续对话"""

    def set_system_prompt(self, prompt: str) -> None:
        """设置系统提示词"""
```

#### 1.3 TTS核心接口
```python
# src/modules/tts/tts_engine.py
from dataclasses import dataclass
from typing import Optional
from abc import ABC, abstractmethod

@dataclass
class AudioData:
    data: bytes
    format: str
    sample_rate: int
    channels: int
    duration: float

@dataclass
class Voice:
    name: str
    language: str
    gender: str
    age_group: str

class TTSProvider(ABC):
    @abstractmethod
    def synthesize(self, text: str, voice: str = None) -> AudioData:
        """合成语音"""
        pass

class TTSEngine:
    def __init__(self, config: TTSConfig):
        """初始化TTS引擎"""

    def synthesize(self, text: str, voice: str = None) -> AudioData:
        """合成语音，返回音频数据"""

    def synthesize_to_file(self, text: str, file_path: str, voice: str = None) -> None:
        """合成语音并保存到文件"""

    def get_available_voices(self) -> List[Voice]:
        """获取可用语音列表"""

    def preload_voice(self, voice: str) -> None:
        """预加载语音模型"""
```

### 2. 系统控制接口

#### 2.1 状态机接口
```python
# src/modules/system_control/state_machine.py
from enum import Enum
from typing import Callable, Optional

class SystemState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    PROCESSING = "processing"
    SPEAKING = "speaking"
    ERROR = "error"
    MAINTENANCE = "maintenance"

class StateMachine:
    def __init__(self, initial_state: SystemState):
        """初始化状态机"""

    def transition_to(self, new_state: SystemState) -> bool:
        """转换到新状态，返回是否成功"""

    def get_current_state(self) -> SystemState:
        """获取当前状态"""

    def can_transition_to(self, new_state: SystemState) -> bool:
        """检查是否可以转换到新状态"""

    def add_transition_callback(self, callback: Callable[[SystemState, SystemState], None]) -> None:
        """添加状态转换回调"""

    def get_valid_transitions(self) -> List[SystemState]:
        """获取当前状态的有效转换列表"""
```

#### 2.2 命令分发器接口
```python
# src/modules/system_control/command_dispatcher.py
from typing import Callable, Dict, List, Optional
from dataclasses import dataclass

@dataclass
class CommandResult:
    success: bool
    message: str
    data: Optional[Dict] = None
    execution_time: Optional[float] = None

@dataclass
class CommandInfo:
    name: str
    description: str
    parameters: Dict[str, type]
    handler: Callable

class CommandDispatcher:
    def __init__(self):
        """初始化命令分发器"""

    def register_command(self, name: str, handler: Callable, description: str = "",
                        parameters: Dict[str, type] = None) -> None:
        """注册命令处理器"""

    def unregister_command(self, name: str) -> None:
        """注销命令"""

    def execute_command(self, command: str, args: Dict = None) -> CommandResult:
        """执行命令"""

    def get_available_commands(self) -> List[CommandInfo]:
        """获取可用命令列表"""

    def validate_command(self, command: str, args: Dict = None) -> bool:
        """验证命令参数"""
```

---

## 📝 API使用指南

### 1. ROS2 API使用
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from xlerobot_srvs.srv import ConversationService

class XleRobotClient(Node):
    def __init__(self):
        super().__init__('xlerobot_client')

        # 创建服务客户端
        self.conversation_client = self.create_client(
            ConversationService, '/conversation_service'
        )

        # 创建订阅者
        self.response_sub = self.create_subscription(
            String, '/llm_response', self.response_callback, 10
        )

    def send_message(self, message: str) -> str:
        """发送消息到对话服务"""
        request = ConversationService.Request()
        request.message = message
        request.user_id = "user_001"

        future = self.conversation_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        return response.response if response.success else None

    def response_callback(self, msg: String):
        """处理LLM响应回调"""
        self.get_logger().info(f"收到回复: {msg.data}")
```

### 2. Web API使用
```python
import requests
import json

class XleRobotWebClient:
    def __init__(self, base_url: str, token: str):
        self.base_url = base_url
        self.headers = {
            'Authorization': f'Bearer {token}',
            'Content-Type': 'application/json'
        }

    def send_message(self, message: str, conversation_id: str = None) -> dict:
        """发送消息"""
        url = f"{self.base_url}/api/v1/conversation/message"
        data = {
            "message": message,
            "conversation_id": conversation_id
        }

        response = requests.post(url, json=data, headers=self.headers)
        return response.json() if response.status_code == 200 else None

    def get_system_status(self) -> dict:
        """获取系统状态"""
        url = f"{self.base_url}/api/v1/system/status"
        response = requests.get(url, headers=self.headers)
        return response.json() if response.status_code == 200 else None
```

### 3. WebSocket API使用
```python
import asyncio
import websockets
import json

class XleobotWebSocketClient:
    def __init__(self, uri: str, token: str):
        self.uri = uri
        self.token = token
        self.websocket = None

    async def connect(self):
        """连接WebSocket"""
        self.websocket = await websockets.connect(self.uri)

        # 发送认证消息
        auth_message = {
            "type": "auth",
            "token": self.token
        }
        await self.websocket.send(json.dumps(auth_message))

        # 等待认证响应
        response = await self.websocket.recv()
        auth_response = json.loads(response)

        if not auth_response.get("success"):
            raise Exception("WebSocket认证失败")

    async def send_message(self, message: str, conversation_id: str = None):
        """发送消息"""
        msg = {
            "type": "user_message",
            "content": message,
            "conversation_id": conversation_id
        }
        await self.websocket.send(json.dumps(msg))

    async def listen(self, callback):
        """监听消息"""
        async for message in self.websocket:
            data = json.loads(message)
            await callback(data)
```

---

## 🔧 API错误处理

### 1. 错误代码定义
```python
class XleRobotErrorCodes:
    # 认证错误 (1000-1099)
    INVALID_TOKEN = 1001
    TOKEN_EXPIRED = 1002
    INSUFFICIENT_PERMISSIONS = 1003

    # 语音处理错误 (2000-2099)
    AUDIO_FORMAT_UNSUPPORTED = 2001
    SPEECH_NOT_RECOGNIZED = 2002
    TTS_SYNTHESIS_FAILED = 2003

    # 对话错误 (3000-3099)
    CONVERSATION_NOT_FOUND = 3001
    MESSAGE_TOO_LONG = 3002
    CONTEXT_INVALID = 3003

    # 系统错误 (4000-4099)
    SERVICE_UNAVAILABLE = 4001
    RESOURCE_EXHAUSTED = 4002
    INTERNAL_ERROR = 4003
```

### 2. 错误响应格式
```json
{
  "success": false,
  "error": {
    "code": 2002,
    "type": "speech_not_recognized",
    "message": "无法识别语音内容",
    "details": {
      "confidence": 0.3,
      "audio_duration": 2.5
    }
  },
  "timestamp": "2025-11-07T10:30:00Z",
  "request_id": "req_uuid"
}
```

### 3. 重试策略
```python
import time
from typing import Callable, Optional

class RetryConfig:
    max_attempts: int = 3
    base_delay: float = 1.0
    max_delay: float = 30.0
    backoff_factor: float = 2.0

async def retry_api_call(callable_func: Callable, *args, **kwargs):
    """API调用重试机制"""
    config = kwargs.pop('retry_config', RetryConfig())
    last_exception = None

    for attempt in range(config.max_attempts):
        try:
            return await callable_func(*args, **kwargs)
        except Exception as e:
            last_exception = e
            if attempt < config.max_attempts - 1:
                delay = min(
                    config.base_delay * (config.backoff_factor ** attempt),
                    config.max_delay
                )
                await asyncio.sleep(delay)

    raise last_exception
```

---

## 📊 API性能指标

### 1. 响应时间要求
```yaml
API性能要求:
  语音识别:
    - 平均响应时间: < 2秒
    - 95%分位响应时间: < 3秒
    - 最大响应时间: < 5秒

  语音合成:
    - 平均响应时间: < 1秒
    - 95%分位响应时间: < 2秒
    - 最大响应时间: < 3秒

  对话响应:
    - 平均响应时间: < 3秒
    - 95%分位响应时间: < 5秒
    - 最大响应时间: < 10秒
```

### 2. 并发处理能力
```yaml
并发要求:
  Web API:
    - 最大并发用户: 100
    - 每秒请求数: 50
    - 连接池大小: 20

  WebSocket:
    - 最大并发连接: 50
    - 消息吞吐量: 1000条/秒
    - 心跳间隔: 30秒

  ROS2:
    - 消息发布频率: 1000Hz
    - 话题缓存大小: 10
    - 服务并发处理: 10个
```

### 3. 可用性指标
```yaml
可用性要求:
  系统可用性: > 99.5%
  API响应成功率: > 99%
  错误恢复时间: < 30秒
  数据一致性: 100%
```

---

## 🔄 API版本管理

### 1. 版本策略
- **主版本**: 不兼容的API变更
- **次版本**: 向后兼容的功能新增
- **修订版本**: 向后兼容的问题修正

### 2. 版本控制
```http
# API版本通过URL路径控制
/api/v1/speech/recognize
/api/v2/speech/recognize

# 支持版本协商
Accept: application/vnd.xlerobot.v1+json
```

### 3. 弃用策略
```http
# 废用通知响应头
Deprecation: true
Sunset: 2025-12-31T00:00:00Z
Link: </api/v2/speech/recognize>; rel="successor-version"
```

---

*本API契约文档遵循Brownfield Level 4企业级标准，为XleRobot系统提供完整的接口规范。文档随API变更持续更新。*