# XleRobot 组件清单文档

**文档编号**: XLR-COMP-P0-20251107-001
**项目名称**: XleRobot 家用机器人控制系统
**文档类型**: 组件清单文档
**生成日期**: 2025-11-07
**工作流**: Phase 0 Documentation - document-project

---

## 📋 概述

本文档详细列出XleRobot项目中所有可重用组件，包括组件功能、接口定义、使用方法、依赖关系等。为AI辅助开发和人工开发提供组件复用指导，提高开发效率和代码一致性。

### 组件分类
- **音频处理组件**: ASR、TTS、音频输入输出
- **AI模型组件**: LLM集成、对话管理
- **系统控制组件**: 状态管理、命令执行
- **通信组件**: ROS2节点、消息处理
- **工具组件**: 配置管理、日志记录、缓存

---

## 🎤 音频处理组件

### 1. ASR核心组件
**组件名称**: `ASRCore`
**位置**: `src/modules/asr/asr_core.py`
**类型**: 核心处理组件

#### 功能描述
提供统一的语音识别接口，支持多种ASR引擎和音频预处理功能。

#### 接口定义
```python
class ASRCore:
    def __init__(self, config: ASRConfig):
        """初始化ASR核心组件"""

    def recognize_audio(self, audio_data: bytes, format: AudioFormat) -> ASRResult:
        """识别音频数据，返回识别结果"""

    def start_streaming(self, callback: Callable[[str], None]) -> None:
        """启动流式识别"""

    def stop_streaming(self) -> None:
        """停止流式识别"""

    def set_provider(self, provider: ASRProvider) -> None:
        """设置ASR提供商"""
```

#### 配置参数
```python
@dataclass
class ASRConfig:
    provider: str = "alibaba"  # alibaba, local
    sample_rate: int = 16000
    channels: int = 1
    format: str = "wav"
    language: str = "cantonese"
    timeout: float = 5.0
    confidence_threshold: float = 0.8
```

#### 使用示例
```python
from src.modules.asr.asr_core import ASRCore, ASRConfig

# 创建ASR组件
config = ASRConfig(provider="alibaba", language="cantonese")
asr = ASRCore(config)

# 识别音频
result = asr.recognize_audio(audio_data, AudioFormat.WAV)
print(f"识别结果: {result.text}")
```

#### 依赖组件
- `AudioPreprocessor`: 音频预处理
- `ASRProvider`: ASR服务提供商
- `WakeWordDetector`: 唤醒词检测

---

### 2. TTS核心组件
**组件名称**: `TTSEngine`
**位置**: `src/modules/tts/tts_engine.py`
**类型**: 核心处理组件

#### 功能描述
提供文本转语音服务，支持多种TTS引擎和音频缓存功能。

#### 接口定义
```python
class TTSEngine:
    def __init__(self, config: TTSConfig):
        """初始化TTS引擎"""

    def synthesize(self, text: str, voice: str = None) -> AudioData:
        """合成语音，返回音频数据"""

    def synthesize_to_file(self, text: str, file_path: str) -> None:
        """合成语音并保存到文件"""

    def get_available_voices(self) -> List[Voice]:
        """获取可用语音列表"""

    def preload_voice(self, voice: str) -> None:
        """预加载语音模型"""
```

#### 配置参数
```python
@dataclass
class TTSConfig:
    provider: str = "alibaba"  # alibaba, local_vits
    voice: str = "cantonese_female"
    sample_rate: int = 22050
    format: str = "wav"
    cache_enabled: bool = True
    cache_size: int = 100
    speed: float = 1.0
    pitch: float = 1.0
```

#### 使用示例
```python
from src.modules.tts.tts_engine import TTSEngine, TTSConfig

# 创建TTS组件
config = TTSConfig(provider="alibaba", voice="cantonese_female")
tts = TTSEngine(config)

# 合成语音
audio_data = tts.synthesize("你好，我是XleRobot")
tts.synthesize_to_file("你好，我是XleRobot", "output.wav")
```

#### 依赖组件
- `AudioPlayer`: 音频播放器
- `TTSCache`: TTS缓存管理
- `TTSProvider`: TTS服务提供商

---

### 3. 音频播放器组件
**组件名称**: `AudioPlayer`
**位置**: `src/modules/asr/audio/audio_player.py`
**类型**: 工具组件

#### 功能描述
提供音频播放功能，支持多种音频格式和设备管理。

#### 接口定义
```python
class AudioPlayer:
    def __init__(self, config: AudioConfig):
        """初始化音频播放器"""

    def play(self, audio_data: AudioData) -> None:
        """播放音频数据"""

    def play_file(self, file_path: str) -> None:
        """播放音频文件"""

    def stop(self) -> None:
        """停止播放"""

    def set_volume(self, volume: float) -> None:
        """设置音量 (0.0-1.0)"""

    def get_devices(self) -> List[AudioDevice]:
        """获取可用音频设备列表"""
```

#### 使用示例
```python
from src.modules.asr.audio.audio_player import AudioPlayer

# 创建音频播放器
player = AudioPlayer()

# 播放音频
player.play(audio_data)
player.set_volume(0.8)
```

---

## 🧠 AI模型组件

### 1. LLM核心组件
**组件名称**: `LLMCore`
**位置**: `src/modules/llm/llm_core.py`
**类型**: 核心处理组件

#### 功能描述
提供大语言模型集成接口，支持多种LLM提供商和对话管理。

#### 接口定义
```python
class LLMCore:
    def __init__(self, config: LLMConfig):
        """初始化LLM核心组件"""

    def generate_response(self, prompt: str, context: Context = None) -> LLMResponse:
        """生成回复"""

    def start_conversation(self, user_id: str) -> Conversation:
        """开始新对话"""

    def continue_conversation(self, conversation_id: str, message: str) -> LLMResponse:
        """继续对话"""

    def set_system_prompt(self, prompt: str) -> None:
        """设置系统提示词"""
```

#### 配置参数
```python
@dataclass
class LLMConfig:
    provider: str = "qwen"  # qwen, openai, local
    model: str = "qwen-plus"
    api_key: str = ""
    api_base: str = ""
    max_tokens: int = 1000
    temperature: float = 0.7
    system_prompt: str = "你是XleRobot助手"
```

#### 使用示例
```python
from src.modules.llm.llm_core import LLMCore, LLMConfig

# 创建LLM组件
config = LLMConfig(provider="qwen", model="qwen-plus")
llm = LLMCore(config)

# 生成回复
response = llm.generate_response("今天天气怎么样？")
print(f"回复: {response.text}")
```

#### 依赖组件
- `ConversationManager`: 对话管理器
- `ContextManager`: 上下文管理器
- `LLMProvider`: LLM服务提供商

---

### 2. 对话管理器组件
**组件名称**: `ConversationManager`
**位置**: `src/modules/llm/conversation_manager.py`
**类型**: 核心处理组件

#### 功能描述
管理多用户对话会话，维护对话历史和上下文信息。

#### 接口定义
```python
class ConversationManager:
    def __init__(self, max_history: int = 10):
        """初始化对话管理器"""

    def create_conversation(self, user_id: str) -> str:
        """创建新对话，返回对话ID"""

    def add_message(self, conversation_id: str, role: str, content: str) -> None:
        """添加消息到对话"""

    def get_history(self, conversation_id: str, limit: int = None) -> List[Message]:
        """获取对话历史"""

    def clear_conversation(self, conversation_id: str) -> None:
        """清空对话"""

    def get_context(self, conversation_id: str) -> Context:
        """获取对话上下文"""
```

#### 使用示例
```python
from src.modules.llm.conversation_manager import ConversationManager

# 创建对话管理器
manager = ConversationManager()

# 创建对话
conversation_id = manager.create_conversation("user_001")

# 添加消息
manager.add_message(conversation_id, "user", "你好")
manager.add_message(conversation_id, "assistant", "你好！我是XleRobot")

# 获取历史
history = manager.get_history(conversation_id)
```

---

## 🤖 系统控制组件

### 1. 状态机组件
**组件名称**: `StateMachine`
**位置**: `src/modules/system_control/state_machine.py`
**类型**: 核心控制组件

#### 功能描述
管理系统状态转换，确保系统状态的一致性和可靠性。

#### 接口定义
```python
class StateMachine:
    def __init__(self, initial_state: SystemState):
        """初始化状态机"""

    def transition_to(self, new_state: SystemState) -> bool:
        """转换到新状态"""

    def get_current_state(self) -> SystemState:
        """获取当前状态"""

    def can_transition_to(self, new_state: SystemState) -> bool:
        """检查是否可以转换到新状态"""

    def add_transition_callback(self, callback: Callable[[SystemState, SystemState], None]) -> None:
        """添加状态转换回调"""
```

#### 状态定义
```python
class SystemState(Enum):
    IDLE = "idle"                    # 空闲状态
    LISTENING = "listening"          # 监听状态
    PROCESSING = "processing"        # 处理状态
    SPEAKING = "speaking"            # 说话状态
    ERROR = "error"                  # 错误状态
    MAINTENANCE = "maintenance"      # 维护状态
```

#### 使用示例
```python
from src.modules.system_control.state_machine import StateMachine, SystemState

# 创建状态机
state_machine = StateMachine(SystemState.IDLE)

# 状态转换
if state_machine.can_transition_to(SystemState.LISTENING):
    state_machine.transition_to(SystemState.LISTENING)
```

---

### 2. 命令分发器组件
**组件名称**: `CommandDispatcher`
**位置**: `src/modules/system_control/command_dispatcher.py`
**类型**: 核心控制组件

#### 功能描述
分发和执行系统命令，提供命令注册和执行机制。

#### 接口定义
```python
class CommandDispatcher:
    def __init__(self):
        """初始化命令分发器"""

    def register_command(self, name: str, handler: Callable, description: str = "") -> None:
        """注册命令处理器"""

    def execute_command(self, command: str, args: dict = None) -> CommandResult:
        """执行命令"""

    def get_available_commands(self) -> List[CommandInfo]:
        """获取可用命令列表"""

    def unregister_command(self, name: str) -> None:
        """注销命令"""
```

#### 使用示例
```python
from src.modules.system_control.command_dispatcher import CommandDispatcher

# 创建命令分发器
dispatcher = CommandDispatcher()

# 注册命令
def hello_handler(args):
    return CommandResult(success=True, message="你好！")

dispatcher.register_command("hello", hello_handler, "打招呼命令")

# 执行命令
result = dispatcher.execute_command("hello")
```

---

## 📡 通信组件

### 1. ROS2节点基类
**组件名称**: `BaseNode`
**位置**: `src/xlerobot_llm/base_node.py`
**类型**: 基础组件

#### 功能描述
提供ROS2节点的基础功能，包括日志记录、参数管理、生命周期管理。

#### 接口定义
```python
class BaseNode(Node):
    def __init__(self, node_name: str):
        """初始化基础节点"""

    def create_publisher(self, topic: str, msg_type: Type) -> Publisher:
        """创建发布者"""

    def create_subscription(self, topic: str, msg_type: Type, callback: Callable) -> Subscription:
        """创建订阅者"""

    def get_parameter(self, name: str, default_value=None):
        """获取参数值"""

    def declare_parameter(self, name: str, default_value):
        """声明参数"""

    def log_info(self, message: str) -> None:
        """记录信息日志"""

    def log_error(self, message: str) -> None:
        """记录错误日志"""
```

#### 使用示例
```python
from src.xlerobot_llm.base_node import BaseNode

class ASRNode(BaseNode):
    def __init__(self):
        super().__init__("xlerobot_asr_node")

        # 创建发布者
        self.result_publisher = self.create_publisher(
            "/asr_result", String
        )

        # 创建订阅者
        self.audio_subscription = self.create_subscription(
            "/audio_input", AudioData, self.audio_callback
        )
```

---

### 2. 消息转换器组件
**组件名称**: `MessageConverter`
**位置**: `src/modules/integration/message_converter.py`
**类型**: 工具组件

#### 功能描述
提供ROS2消息和Python对象之间的转换功能。

#### 接口定义
```python
class MessageConverter:
    @staticmethod
    def audio_data_to_ros(audio_data: AudioData) -> AudioMsg:
        """音频数据转ROS消息"""

    @staticmethod
    def ros_to_audio_data(msg: AudioMsg) -> AudioData:
        """ROS消息转音频数据"""

    @staticmethod
    def llm_response_to_ros(response: LLMResponse) -> StringMsg:
        """LLM响应转ROS消息"""

    @staticmethod
    def text_to_ros(text: str) -> StringMsg:
        """文本转ROS消息"""
```

---

## 🔧 工具组件

### 1. 配置管理器组件
**组件名称**: `ConfigManager`
**位置**: `src/modules/common/config_manager.py`
**类型**: 工具组件

#### 功能描述
统一管理项目配置，支持环境变量覆盖和配置验证。

#### 接口定义
```python
class ConfigManager:
    def __init__(self, config_path: str):
        """初始化配置管理器"""

    def get(self, key: str, default=None):
        """获取配置值"""

    def set(self, key: str, value) -> None:
        """设置配置值"""

    def reload(self) -> None:
        """重新加载配置"""

    def validate(self) -> List[str]:
        """验证配置，返回错误列表"""

    def get_all(self) -> dict:
        """获取所有配置"""
```

#### 使用示例
```python
from src.modules.common.config_manager import ConfigManager

# 创建配置管理器
config = ConfigManager("config/config.yaml")

# 获取配置
api_key = config.get("llm.api_key")
sample_rate = config.get("asr.sample_rate", 16000)
```

---

### 2. 日志管理器组件
**组件名称**: `LogManager`
**位置**: `src/modules/common/log_manager.py`
**类型**: 工具组件

#### 功能描述
提供统一的日志记录功能，支持多种输出格式和日志级别。

#### 接口定义
```python
class LogManager:
    def __init__(self, config: LogConfig):
        """初始化日志管理器"""

    def get_logger(self, name: str) -> Logger:
        """获取日志记录器"""

    def set_level(self, level: str) -> None:
        """设置日志级别"""

    def add_handler(self, handler: logging.Handler) -> None:
        """添加日志处理器"""

    def log_performance(self, operation: str, duration: float) -> None:
        """记录性能日志"""
```

#### 使用示例
```python
from src.modules.common.log_manager import LogManager

# 获取日志记录器
log_manager = LogManager(log_config)
logger = log_manager.get_logger("xlerobot.asr")

# 记录日志
logger.info("ASR组件初始化完成")
logger.error("音频处理失败", exc_info=True)
```

---

### 3. 缓存管理器组件
**组件名称**: `CacheManager`
**位置**: `src/modules/common/cache_manager.py`
**类型**: 工具组件

#### 功能描述
提供内存缓存功能，支持LRU策略和自动过期。

#### 接口定义
```python
class CacheManager:
    def __init__(self, max_size: int = 1000, ttl: int = 3600):
        """初始化缓存管理器"""

    def get(self, key: str) -> Any:
        """获取缓存值"""

    def set(self, key: str, value: Any, ttl: int = None) -> None:
        """设置缓存值"""

    def delete(self, key: str) -> bool:
        """删除缓存"""

    def clear(self) -> None:
        """清空缓存"""

    def stats(self) -> CacheStats:
        """获取缓存统计信息"""
```

#### 使用示例
```python
from src.modules.common.cache_manager import CacheManager

# 创建缓存管理器
cache = CacheManager(max_size=100, ttl=300)

# 使用缓存
result = cache.get("asr_result")
if result is None:
    result = expensive_operation()
    cache.set("asr_result", result)
```

---

## 🔄 组件依赖关系图

```
XleRobot组件依赖关系:

ASRCore
├── AudioPreprocessor
├── ASRProvider
│   ├── AlibabaASRProvider
│   └── LocalASRProvider
├── WakeWordDetector
└── AudioPlayer

TTSEngine
├── TTSProvider
│   ├── AlibabaTTSProvider
│   └── VITSProvider
├── TTSCache
└── AudioPlayer

LLMCore
├── LLMProvider
│   ├── QwenProvider
│   └── OpenAIProvider
├── ConversationManager
├── ContextManager
└── CacheManager

SystemController
├── StateMachine
├── CommandDispatcher
├── MonitoringService
└── HealthCheckService

BaseNode
├── ConfigManager
├── LogManager
└── MessageConverter
```

---

## 🎯 组件使用指南

### 1. 组件选择建议
- **语音处理**: 使用ASRCore + TTSEngine + AudioPlayer
- **对话功能**: 使用LLMCore + ConversationManager
- **系统控制**: 使用StateMachine + CommandDispatcher
- **ROS2集成**: 继承BaseNode类
- **工具功能**: 使用ConfigManager + LogManager + CacheManager

### 2. 组件组合模式
```python
# 标准语音处理管道
class VoicePipeline:
    def __init__(self):
        self.asr = ASRCore(ASRConfig())
        self.llm = LLMCore(LLMConfig())
        self.tts = TTSEngine(TTSConfig())
        self.player = AudioPlayer()

    def process_voice(self, audio_data):
        # ASR处理
        text = self.asr.recognize_audio(audio_data)

        # LLM处理
        response = self.llm.generate_response(text)

        # TTS处理
        audio_output = self.tts.synthesize(response.text)

        # 播放音频
        self.player.play(audio_output)
```

### 3. 组件扩展指导
- **新ASR提供商**: 实现ASRProvider接口
- **新TTS引擎**: 实现TTSProvider接口
- **新LLM模型**: 实现LLMProvider接口
- **自定义命令**: 注册到CommandDispatcher
- **新ROS2节点**: 继承BaseNode类

---

## 📊 组件性能指标

### 1. 响应时间要求
- **ASR识别**: < 2秒
- **LLM生成**: < 5秒
- **TTS合成**: < 1秒
- **状态转换**: < 100ms

### 2. 资源使用限制
- **内存使用**: < 2GB
- **CPU使用**: < 80%
- **网络带宽**: < 1Mbps
- **存储空间**: < 10GB

### 3. 可靠性指标
- **可用性**: > 99%
- **错误恢复时间**: < 10秒
- **数据一致性**: 100%
- **并发支持**: 10个用户

---

## 🔍 组件质量标准

### 1. 代码质量
- **单元测试覆盖率**: > 90%
- **集成测试覆盖**: 核心流程100%
- **代码规范**: 遵循PEP8标准
- **文档覆盖**: 所有公共接口

### 2. 性能标准
- **响应时间**: 满足实时性要求
- **资源使用**: 在硬件限制内
- **并发处理**: 支持多用户并发
- **错误处理**: 优雅的异常处理

### 3. 兼容性标准
- **Python版本**: 3.10+
- **ROS2版本**: Humble
- **硬件平台**: RDK X5
- **操作系统**: Ubuntu 22.04+

---

## 📝 组件维护指南

### 1. 添加新组件
1. **定义接口**: 设计清晰的组件接口
2. **实现功能**: 遵循现有设计模式
3. **编写测试**: 单元测试和集成测试
4. **更新文档**: 接口文档和使用示例
5. **注册组件**: 添加到组件清单

### 2. 修改现有组件
1. **保持兼容**: 确保向后兼容性
2. **更新测试**: 修改相关测试用例
3. **更新文档**: 同步更新接口文档
4. **验证影响**: 测试相关组件功能

### 3. 组件弃用
1. **标记弃用**: 在文档中标记弃用状态
2. **提供替代**: 推荐替代组件
3. **渐进迁移**: 支持平滑迁移过程
4. **最终移除**: 在合适时机移除组件

---

*本组件清单文档遵循Brownfield Level 4企业级标准，为XleRobot项目提供完整的组件复用指导。文档随组件变更持续更新。*