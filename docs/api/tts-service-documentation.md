# XleRobot TTS服务技术文档

## 概述

XleRobot Story 1.4 TTS服务是基于阿里云TTS API的纯在线语音合成解决方案，专门为粤语语音交互设计。本文档提供完整的API接口说明、配置参数指南和故障排除方法。

## 目录

- [系统架构](#系统架构)
- [API接口文档](#api接口文档)
- [配置参数说明](#配置参数说明)
- [故障排除指南](#故障排除指南)
- [性能指标](#性能指标)
- [部署指南](#部署指南)

## 系统架构

### 组件架构
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   ROS2 Topics   │    │   TTS Service   │    │  Alibaba Cloud  │
│                 │    │     Node        │    │     TTS API     │
│ /tts/text_input├────┤                 ├────┤                 │
│ /tts/audio     │    │  AliyunTTSClient│    │   jiajia voice  │
│ /tts/status    │    │  AudioProcessor │    │   16kHz WAV     │
│ /audio_player/ │    │  ErrorHandler   │    │                 │
│ status         │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                    ┌─────────────────┐
                    │ Audio Player    │
                    │      Node       │
                    │                 │
                    │ ALSA Playback  │
                    │ Stream Control  │
                    └─────────────────┘
```

### 设计原则
- **纯在线服务**: 100%依赖阿里云TTS API，无本地TTS模型
- **高可靠性**: 完整的错误处理和重试机制
- **高质量音频**: 16kHz/16bit/单声道WAV输出
- **企业级标准**: 全面的日志记录和监控支持

## API接口文档

### 1. AliyunTTSClient类

#### 构造函数
```python
AliyunTTSClient(config: Dict[str, Any])
```

**参数:**
- `config`: 配置字典，包含以下必需字段：
  - `app_key`: 阿里云应用密钥
  - `token`: 访问令牌
  - `region`: 服务区域 (默认: cn-shanghai)
  - `timeout`: 请求超时时间 (默认: 10秒)
  - `max_retries`: 最大重试次数 (默认: 3)

**示例:**
```python
config = {
    'app_key': 'your_app_key',
    'token': 'your_token',
    'region': 'cn-shanghai',
    'timeout': 10,
    'max_retries': 3
}
client = AliyunTTSClient(config)
```

#### 主要方法

##### synthesize_speech
```python
def synthesize_speech(self, text: str, **kwargs) -> Optional[bytes]
```

语音合成主接口，返回WAV格式音频数据。

**参数:**
- `text`: 要合成的文本 (必需)
- `voice`: 发音人 (可选, 默认: 'jiajia')
- `speech_rate`: 语速 (0-100, 可选, 默认: 50)
- `pitch_rate`: 音调 (0-100, 可选, 默认: 50)
- `volume`: 音量 (0-100, 可选, 默认: 50)

**返回:**
- `bytes`: WAV格式音频数据，失败时返回None

**示例:**
```python
# 基础语音合成
audio_data = client.synthesize_speech("你好世界")

# 带参数的语音合成
audio_data = client.synthesize_speech(
    "你好世界",
    voice='jiajia',
    speech_rate=60,
    pitch_rate=50,
    volume=80
)
```

##### validate_parameters
```python
def validate_parameters(self, **kwargs) -> Dict[str, Any]
```

验证和调整参数范围。

**返回:**
- `Dict`: 包含调整后参数和警告信息

##### test_connection
```python
def test_connection(self) -> bool
```

测试TTS服务连接。

**返回:**
- `bool`: 连接是否成功

##### get_supported_voices
```python
def get_supported_voices() -> Dict[str, str]
```

获取支持的发音人列表。

**返回:**
- `Dict`: 发音人映射字典

### 2. AudioProcessor类

#### 音频质量控制方法

##### adjust_speech_rate
```python
def adjust_speech_rate(self, audio_data: bytes, rate_factor: float = 1.0) -> Optional[bytes]
```

调节语音播放速度 (0.8x - 1.2x)。

**参数:**
- `audio_data`: 原始WAV音频数据
- `rate_factor`: 速度因子 (0.8-1.2)

##### adjust_pitch_rate
```python
def adjust_pitch_rate(self, audio_data: bytes, pitch_factor: float = 1.0) -> Optional[bytes]
```

调节语音音调 (±20%)。

**参数:**
- `audio_data`: 原始WAV音频数据
- `pitch_factor`: 音调因子 (0.8-1.2)

##### adjust_volume
```python
def adjust_volume(self, audio_data: bytes, volume_factor: float = 1.0) -> Optional[bytes]
```

调节语音音量 (50% - 150%)。

**参数:**
- `audio_data`: 原始WAV音频数据
- `volume_factor`: 音量因子 (0.5-1.5)

##### apply_emotion_style
```python
def apply_emotion_style(self, audio_data: bytes, emotion: str = "friendly") -> Optional[bytes]
```

应用情感语音风格。

**参数:**
- `audio_data`: 原始WAV音频数据
- `emotion`: 情感类型 ("friendly", "confirm", "error")

##### enhance_audio_quality
```python
def enhance_audio_quality(self, audio_data: bytes) -> Optional[bytes]
```

音频质量增强处理。

**参数:**
- `audio_data`: 原始WAV音频数据

##### evaluate_audio_quality
```python
def evaluate_audio_quality(self, audio_data: bytes) -> Dict[str, Any]
```

评估音频质量。

**返回:**
- `Dict`: 包含质量评分、信噪比、动态范围等指标

### 3. ROS2服务接口

#### TextToSpeech服务
**服务名称**: `/tts/synthesize`
**服务类型**: `audio_msg/srv/TextToSpeech`

**请求字段:**
- `text`: 要合成的文本 (string)
- `voice`: 发音人 (string, 可选)
- `speech_rate`: 语速 (int32, 可选)
- `pitch_rate`: 音调 (int32, 可选)
- `volume`: 音量 (int32, 可选)
- `emotion`: 情感类型 (string, 可选)
- `enhance_quality`: 是否增强质量 (bool, 可选)

**响应字段:**
- `success`: 合成是否成功 (bool)
- `duration`: 处理时长 (float64)
- `message`: 状态消息 (string)

#### 话题接口

##### /tts/audio
**消息类型**: `audio_msg/msg/AudioData`
**方向**: 发布
**描述**: 发布合成后的音频数据

**字段:**
- `data`: 音频数据 (uint8[])
- `format`: 音频格式 (string)
- `sample_rate`: 采样率 (int32)
- `channels`: 声道数 (int32)
- `stamp`: 时间戳 (time)

##### /tts/status
**消息类型**: `audio_msg/msg/TTSStatus`
**方向**: 发布
**描述**: 发布TTS服务状态

**字段:**
- `status`: 状态 (string)
- `text_hash`: 文本哈希 (string)
- `duration`: 处理时长 (float64)
- `error_code`: 错误码 (int32)
- `stamp`: 时间戳 (time)

##### /tts/text_input
**消息类型**: `std_msgs/msg/String`
**方向**: 订阅
**描述**: 接收文本输入进行合成

## 配置参数说明

### 配置文件 (config/tts_config.yaml)

```yaml
# 阿里云API配置
app_key: "your_app_key"           # 应用密钥 (必需)
token: "your_token"               # 访问令牌 (可通过环境变量ALIBABA_CLOUD_TOKEN设置)
region: "cn-shanghai"             # 服务区域
endpoint: "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts"
timeout: 10                       # 请求超时时间(秒)
max_retries: 3                    # 最大重试次数

# 默认语音参数
voice: "jiajia"                   # 粤语女声发音人
volume: 50                        # 音量 (50-150)
speech_rate: 0                    # 语速调节 (0-100)
pitch_rate: 0                     # 音调调节 (0-100)

# 音频格式配置
format: "wav"                     # 固定WAV格式
sample_rate: 16000                # 固定16kHz采样率
channels: 1                       # 固定单声道
bits_per_sample: 16               # 固定16位深度

# 错误处理配置
retry_delays: [1.0, 2.0, 4.0]     # 重试延迟(秒)
retry_backoff: 2.0                # 退避倍数
max_retry_delay: 20.0             # 最大重试延迟(秒)

# 日志配置
log_level: "INFO"                 # 日志级别
log_file: "logs/tts.log"          # 日志文件路径
enable_file_logging: true         # 是否启用文件日志

# 性能配置
max_concurrent_requests: 1        # 最大并发请求数
audio_buffer_size: 8192           # 音频缓冲区大小
cleanup_temp_files: true          # 是否清理临时文件

# 支持的发音人
supported_voices:
  jiajia: "佳佳 (粤语女声)"
  xiaoxiao: "晓晓 (标准女声)"
  xiaoyun: "晓云 (知性女声)"

# 粤语特定配置
cantonese_settings:
  default_voice: "jiajia"
  preferred_speech_rate: 0
  preferred_pitch_rate: 0
  cultural_adaptations: true

# 安全配置
api_key_validation: true          # API密钥验证
request_logging: false            # 请求日志记录
sensitive_data_masking: true      # 敏感数据掩码
```

### 环境变量

| 变量名 | 描述 | 默认值 | 必需 |
|--------|------|--------|------|
| `ALIBABA_CLOUD_TOKEN` | 阿里云访问令牌 | - | 是 |
| `TTS_LOG_LEVEL` | 日志级别 | INFO | 否 |
| `TTS_CONFIG_PATH` | 配置文件路径 | config/tts_config.yaml | 否 |
| `TTS_VOICE_DEFAULT` | 默认发音人 | jiajia | 否 |

## 故障排除指南

### 常见问题及解决方案

#### 1. 认证失败

**症状:**
```
❌ TTS请求失败: HTTP 400 - Meta:ACCESS_DENIED:The token 'xxx' is invalid!
```

**解决方案:**
1. 检查`ALIBABA_CLOUD_TOKEN`环境变量是否设置正确
2. 验证配置文件中的`app_key`和`token`是否有效
3. 确认阿里云账户已开通TTS服务且有足够余额

**检查命令:**
```bash
echo $ALIBABA_CLOUD_TOKEN
env | grep ALIBABA
```

#### 2. 网络连接问题

**症状:**
```
❌ TTS请求失败: Connection timeout
❌ TTS请求失败: Network unreachable
```

**解决方案:**
1. 检查网络连接是否正常
2. 验证防火墙设置，确保HTTPS出站连接允许
3. 检查DNS解析是否正常

**诊断命令:**
```bash
ping nls-gateway.cn-shanghai.aliyuncs.com
curl -I https://nls-gateway.cn-shanghai.aliyuncs.com
```

#### 3. 音频质量问题

**症状:**
- 生成的音频质量评分过低
- 音频播放有杂音或断断续续

**解决方案:**
1. 使用`enhance_quality`参数启用音频增强
2. 调整音频参数：适当降低语速、调整音量和音调
3. 检查ALSA音频设备配置

**质量检查代码:**
```python
processor = AudioProcessor()
quality_result = processor.evaluate_audio_quality(audio_data)
print(f"质量评分: {quality_result['quality_score']} ({quality_result['quality_rating']})")
```

#### 4. 内存泄漏问题

**症状:**
- 长时间运行后内存持续增长
- 系统响应变慢

**解决方案:**
1. 确保正确清理临时文件
2. 检查音频数据处理是否及时释放资源
3. 重启TTS服务节点

**监控命令:**
```bash
# 监控TTS节点内存使用
ps aux | grep tts_service
top -p $(pgrep tts_service)
```

#### 5. ROS2节点启动失败

**症状:**
```
Failed to load entry point for xlerobot
ImportError: No module named 'audio_msg'
```

**解决方案:**
1. 确保ROS2环境正确配置
2. 检查消息包是否正确安装
3. 验证Python路径设置

**修复命令:**
```bash
source /opt/ros/humble/setup.bash
cd /home/sunrise/xlerobot
colcon build --packages-select xlerobot
source install/setup.bash
```

### 调试模式

启用详细日志记录：

```bash
# 设置环境变量
export TTS_LOG_LEVEL=DEBUG

# 或修改配置文件
log_level: "DEBUG"
enable_file_logging: true
```

查看详细日志：
```bash
tail -f logs/tts.log
```

### 性能监控

使用内置的性能监控功能：

```python
# 在TTS服务节点中查看状态
status = tts_node.get_node_status()
print(f"处理状态: {status['is_processing']}")
print(f"连接状态: {status['tts_connected']}")
```

## 性能指标

### 基准性能

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 响应时间 | < 3秒 | 从文本输入到音频输出的时间 |
| 并发处理 | 5个请求 | 同时处理的最大请求数 |
| 成功率 | > 95% | 语音合成成功率 |
| 内存使用 | < 100MB | 稳定运行时的内存占用 |
| CPU使用率 | < 10% | 正常负载下的CPU占用 |

### 质量指标

| 指标 | 优秀 | 良好 | 一般 | 较差 |
|------|------|------|------|------|
| 质量评分 | 80-100 | 60-79 | 40-59 | 0-39 |
| 信噪比(dB) | >30 | 20-30 | 10-20 | <10 |
| 动态范围(dB) | 10-40 | 5-50 | 其他 | 其他 |

## 部署指南

### 系统要求

- **操作系统**: Ubuntu 20.04/22.04 LTS
- **Python版本**: 3.10+
- **ROS2版本**: Humble
- **内存**: 最少2GB，推荐4GB+
- **存储**: 最少1GB可用空间
- **网络**: 稳定的互联网连接

### 快速部署

```bash
# 1. 克隆项目
cd /home/sunrise
git clone <repository_url>
cd xlerobot

# 2. 安装依赖
./scripts/install_dependencies.sh

# 3. 配置环境
export ALIBABA_CLOUD_TOKEN="your_token_here"

# 4. 构建项目
colcon build --packages-select xlerobot

# 5. 启动服务
source install/setup.bash
ros2 launch xlerobot tts_service.launch.py
```

### 生产部署

详细的生产环境部署步骤请参考 `deployment/production-deployment.md`。

---

**文档版本**: v1.4
**最后更新**: 2024年
**维护团队**: XleRobot开发团队