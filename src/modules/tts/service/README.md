# TTS服务接口

TTS（文本转语音）服务提供完整的RESTful API和WebSocket流式接口，支持多种音色、情感语音合成和实时流式传输。

## 特性

- ✅ **RESTful API** - 标准HTTP接口，易于集成
- ✅ **WebSocket流式接口** - 实时音频流传输
- ✅ **多音色支持** - 支持多种音色和语言
- ✅ **情感语音合成** - 支持9种情感类型
- ✅ **高性能缓存** - LRU缓存机制，提升响应速度
- ✅ **并发支持** - 支持10并发请求，响应时间<100ms
- ✅ **实时监控** - 性能指标收集和健康检查
- ✅ **异步处理** - 基于FastAPI和异步IO
- ✅ **完整文档** - 自动生成OpenAPI文档

## 目录结构

```
service/
├── api/                    # RESTful API
│   ├── __init__.py
│   └── tts_api.py
├── websocket/              # WebSocket接口
│   ├── __init__.py
│   └── websocket_server.py
├── models/                 # 数据模型
│   ├── __init__.py
│   ├── request_models.py   # 请求模型
│   ├── response_models.py  # 响应模型
│   └── websocket_models.py # WebSocket模型
├── utils/                  # 工具类
│   ├── __init__.py
│   ├── cache_manager.py    # 缓存管理
│   ├── performance_monitor.py  # 性能监控
│   └── voice_loader.py     # 音色预加载
├── examples/               # 使用示例
│   ├── api_client_example.py
│   └── websocket_client_example.py
├── tests/                  # 测试套件
├── tts_service.py          # 主服务入口
└── README.md
```

## 快速开始

### 1. 启动服务

```python
from tts_service import TTSService
from tts_system import TTSSystem
from voice_manager import VoiceManager

# 初始化组件
tts_system = TTSSystem()
voice_manager = VoiceManager()

# 创建服务
service = TTSService(tts_system, voice_manager, config={
    'cache_max_size': 1000,
    'cache_ttl': 3600,
    'common_voices': ['default', 'female', 'male']
})

# 启动服务
await service.start(host="0.0.0.0", port=8000)
```

### 2. 使用RESTful API

#### 健康检查

```bash
curl -X GET http://localhost:8000/api/v1/tts/health?detailed=true
```

#### 获取音色列表

```bash
curl -X GET http://localhost:8000/api/v1/tts/voices
```

#### 文本转语音

```bash
curl -X POST http://localhost:8000/api/v1/tts/synthesize \
  -H "Content-Type: application/json" \
  -d '{
    "text": "你好，欢迎使用TTS服务！",
    "voice_id": "default",
    "emotion": "happy",
    "emotion_intensity": 0.8
  }' \
  --output output.wav
```

### 3. 使用WebSocket

```python
import asyncio
import websockets
import json

async def tts_stream():
    uri = "ws://localhost:8000/ws/v1/tts/stream"
    async with websockets.connect(uri) as websocket:
        # 发送请求
        request = {
            "type": "request",
            "data": {
                "text": "这是流式TTS测试",
                "voice_id": "default",
                "emotion": "happy"
            }
        }
        await websocket.send(json.dumps(request))

        # 接收音频分块
        async for message in websocket:
            data = json.loads(message)
            if data.get('type') == 'response':
                # 处理音频数据
                audio_chunk = base64.b64decode(data['audio_data'])
                # ...
                if data.get('is_final'):
                    break

asyncio.run(tts_stream())
```

## API文档

### RESTful API端点

| 方法 | 路径 | 描述 |
|------|------|------|
| GET | `/api/v1/tts/health` | 健康检查 |
| GET | `/api/v1/tts/voices` | 获取音色列表 |
| POST | `/api/v1/tts/synthesize` | 文本转语音 |
| POST | `/api/v1/tts/synthesize-stream` | 流式文本转语音 |

### WebSocket端点

| 协议 | 路径 | 描述 |
|------|------|------|
| WS | `/ws/v1/tts/stream` | 流式TTS WebSocket接口 |

## 请求参数

### TTSRequest

| 参数 | 类型 | 必填 | 默认值 | 描述 |
|------|------|------|--------|------|
| text | string | 是 | - | 要转换的文本 |
| voice_id | string | 否 | "default" | 音色ID |
| language | string | 否 | "zh-CN" | 语言代码 |
| emotion | string | 否 | "neutral" | 情感类型 |
| emotion_intensity | float | 否 | 0.5 | 情感强度 (0.0-1.0) |
| speed | float | 否 | 1.0 | 语速 (0.5-2.0) |
| pitch | float | 否 | 1.0 | 音调 (0.5-2.0) |
| volume | float | 否 | 1.0 | 音量 (0.0-2.0) |
| audio_format | string | 否 | "wav" | 音频格式 (wav/mp3/ogg/flac) |
| audio_quality | string | 否 | "medium" | 音频质量 (low/medium/high/ultra) |
| sample_rate | int | 否 | 22050 | 采样率 |
| cache_enabled | bool | 否 | true | 是否启用缓存 |

### 情感类型

- `neutral` - 中性
- `happy` - 快乐
- `sad` - 悲伤
- `angry` - 愤怒
- `excited` - 兴奋
- `calm` - 平静
- `surprised` - 惊讶
- `fear` - 恐惧
- `disgust` - 厌恶

## 响应格式

### TTSResponse

```json
{
  "success": true,
  "audio_data": "UklGRnoGAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQoGAACBhYqFbF1fdJivrJBhNjVgodDbq2EcBj+a2/LDciUFLIHO8tiJNwgZaLvt559NEAxQp+PwtmMcBjiR1/LMeSwFJHfH8N2QQAoUXrTp66hVFApGn+DyvmUdBzaJ0fPYeTcFJHfH8N2QQAoUXrTp66hVFApGn+DyvmUdBzaJ0fPYeTcF...",
  "audio_format": "wav",
  "audio_size": 102400,
  "duration": 3.5,
  "sample_rate": 22050,
  "request_id": "req-1234567890",
  "processing_time": 0.125,
  "cache_hit": false,
  "message": "音频合成成功"
}
```

## 性能指标

- **并发支持**: 10并发请求
- **响应时间**: <100ms
- **缓存命中率**: >80%
- **音频合成延迟**: <50ms
- **支持并发WebSocket连接**: 100+

## 错误代码

| 错误代码 | 描述 |
|----------|------|
| TTS_001 | TTS合成失败 |
| TTS_002 | 流式TTS合成失败 |
| WS_001 | WebSocket连接错误 |
| INVALID_REQUEST | 请求参数无效 |
| INTERNAL_ERROR | 内部服务器错误 |

## 配置选项

```python
config = {
    'cache_max_size': 1000,          # 缓存最大条目数
    'cache_ttl': 3600,               # 缓存生存时间（秒）
    'voice_loader_workers': 4,       # 音色加载线程数
    'cors_allow_origins': ["*"],     # CORS允许的源
    'common_voices': ['default']     # 预加载的音色列表
}
```

## 使用示例

参见 `examples/` 目录下的示例代码：

- `api_client_example.py` - RESTful API客户端示例
- `websocket_client_example.py` - WebSocket客户端示例

运行示例：

```bash
# RESTful API示例
python examples/api_client_example.py

# WebSocket示例
python examples/websocket_client_example.py basic
python examples/websocket_client_example.py emotion
python examples/websocket_client_example.py batch
python examples/websocket_client_example.py monitor
```

## 监控和调试

### 获取服务统计信息

```python
stats = service.get_stats()
print(json.dumps(stats, indent=2))
```

### 查看性能指标

```python
from utils.performance_monitor import PerformanceMonitor

monitor = PerformanceMonitor()
stats = monitor.get_stats()

print(f"平均响应时间: {stats['avg_response_time']}s")
print(f"QPS: {stats['qps']}")
print(f"缓存命中率: {stats['cache_hit_rate_percent']}%")
```

### 健康检查

```bash
# 基本健康检查
curl http://localhost:8000/api/v1/tts/health

# 详细健康检查
curl http://localhost:8000/api/v1/tts/health?detailed=true
```

## 最佳实践

1. **启用缓存** - 建议启用缓存以提升性能
2. **预加载常用音色** - 配置`common_voices`预加载常用音色
3. **合理设置分块大小** - WebSocket流式接口建议使用1024字节分块
4. **监控性能指标** - 定期检查性能指标和健康状态
5. **错误处理** - 妥善处理API错误和异常情况
6. **连接管理** - WebSocket连接使用心跳机制保持活跃

## 常见问题

### Q: 如何选择合适的音频格式？
A: WAV格式音质最好但文件较大；MP3格式压缩率高，适合网络传输；OGG格式开源兼容性好。

### Q: 如何优化TTS性能？
A: 启用缓存、预加载常用音色、使用异步处理、合理设置并发数。

### Q: WebSocket连接断开怎么办？
A: 实现重连机制，发送心跳保持连接，处理异常并重新连接。

### Q: 如何处理大文本？
A: 对于长文本，建议分段处理或使用流式接口。

## 许可证

本项目采用 MIT 许可证。

## 贡献

欢迎提交 Issue 和 Pull Request！

## 联系方式

如有问题，请联系开发团队。
