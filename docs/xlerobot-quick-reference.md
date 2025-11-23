# XLeRobot 快速参考卡片

> **版本**: Epic 1 (全在线架构)
> **更新日期**: 2025-11-16
> **目标环境**: RDK X5 + ROS2 Humble + Python 3.10.12

本文档为XLeRobot语音交互系统的快速参考指南，提供常用命令、故障排查清单和关键指标。

---

## 📋 目录

- [系统启动命令](#系统启动命令)
- [常用检查命令](#常用检查命令)
- [故障排查清单](#故障排查清单)
- [关键日志消息解读](#关键日志消息解读)
- [性能指标正常范围](#性能指标正常范围)
- [紧急恢复步骤](#紧急恢复步骤)
- [环境变量速查](#环境变量速查)
- [ROS2节点速查](#ros2节点速查)
- [API服务状态检查](#api服务状态检查)

---

## 🚀 系统启动命令

### 标准启动流程

```bash
# 1. 清理环境（移除conda）
source ~/xlerobot/xlerobot_env.sh

# 2. 验证Python版本（必须是3.10.12）
python3.10 --version

# 3. 验证ROS2环境
echo $ROS_DISTRO  # 应输出: humble

# 4. 启动语音助手服务
cd ~/xlerobot
./start_voice_assistant.sh

# 5. 查看启动日志
./start_voice_assistant.sh logs
```

### 快捷启动命令

```bash
# 完整环境检查后启动
./start_voice_assistant.sh

# 强制启动（跳过检查）
./start_voice_assistant.sh --force

# 重启服务
./start_voice_assistant.sh restart

# 停止服务
./start_voice_assistant.sh stop

# 查看服务状态
./start_voice_assistant.sh status
```

### 测试命令

```bash
# Epic 1完整链路测试
python3.10 tests/test_epic1_complete_integration.py

# 快速功能验证
python3.10 tests/verify_epic1_complete_functionality.py

# 真实环境验证
python3.10 tests/real_epic1_verification.py

# 音频设备测试
arecord -d 3 -f cd test.wav && aplay test.wav
```

---

## 🔍 常用检查命令

### 环境检查

```bash
# Python版本检查
python3.10 --version  # 应显示: Python 3.10.12
which python3.10      # 应显示: /usr/bin/python3.10

# ROS2环境检查
echo $ROS_DISTRO                    # 应显示: humble
echo $ROS_DOMAIN_ID                 # 应显示: 42
ros2 --version                      # 验证ROS2可用

# Python路径检查
echo $PYTHONPATH                    # 应包含: ~/xlerobot/src
python3.10 -c "import sys; print('\n'.join(sys.path))"

# 环境变量检查
env | grep -E 'ALIBABA|ALIYUN|QWEN'  # 验证API密钥
```

### ROS2节点检查

```bash
# 列出所有运行中的节点
ros2 node list

# 应该看到以下4个节点:
# - /asr_bridge
# - /llm_service
# - /tts_service
# - /voice_assistant_coordinator

# 查看特定节点信息
ros2 node info /asr_bridge
ros2 node info /llm_service
ros2 node info /tts_service

# 列出所有话题
ros2 topic list

# 关键话题:
# - /voice_command (ASR → Coordinator)
# - /llm_request (Coordinator → LLM)
# - /llm_response (LLM → Coordinator)
# - /tts_request (Coordinator → TTS)
# - /system_status (系统状态)

# 监控话题消息
ros2 topic echo /voice_command
ros2 topic echo /llm_response
ros2 topic echo /system_status
```

### 音频设备检查

```bash
# 列出录音设备
arecord -l

# 列出播放设备
aplay -l

# 测试录音（3秒）
arecord -d 3 -f cd -t wav test_recording.wav

# 测试播放
aplay test_recording.wav

# 检查ALSA配置
cat ~/.asoundrc  # 如果存在
```

### 网络与API检查

```bash
# 测试阿里云连接
ping nls-gateway-cn-shanghai.aliyuncs.com

# DNS解析检查
nslookup nls-gateway-cn-shanghai.aliyuncs.com

# 测试Token获取
python3.10 -c "from aliyun_nls_token_manager import get_valid_token; print(get_valid_token())"

# 检查API密钥配置
cat ~/xlerobot/.env | grep -E 'ALIBABA|ALIYUN|QWEN'
```

---

## 🔧 故障排查清单

### 启动失败

**症状**: 服务无法启动或启动后立即退出

**排查步骤**:

1. ✅ **检查Python版本**
   ```bash
   python3.10 --version  # 必须是3.10.12
   which python3.10      # 必须是/usr/bin/python3.10
   ```

2. ✅ **检查ROS2环境**
   ```bash
   echo $ROS_DISTRO  # 必须输出: humble
   source /opt/ros/humble/setup.bash
   ```

3. ✅ **清理conda环境**
   ```bash
   source ~/xlerobot/xlerobot_env.sh
   echo $PATH | grep -i conda  # 不应有输出
   ```

4. ✅ **检查环境变量**
   ```bash
   cat ~/xlerobot/.env
   # 确认以下变量存在:
   # - ALIBABA_CLOUD_ACCESS_KEY_ID
   # - ALIBABA_CLOUD_ACCESS_KEY_SECRET
   # - ALIYUN_NLS_APPKEY
   # - QWEN_API_KEY
   ```

5. ✅ **查看启动日志**
   ```bash
   ./start_voice_assistant.sh logs
   # 查找ERROR或CRITICAL级别日志
   ```

### ASR识别失败

**症状**: 检测不到唤醒词或无法识别语音

**排查步骤**:

1. ✅ **检查麦克风**
   ```bash
   arecord -l  # 确认设备存在
   arecord -d 3 -f cd test.wav  # 测试录音
   aplay test.wav  # 确认有声音
   ```

2. ✅ **检查ASR服务**
   ```bash
   ros2 node list | grep asr_bridge  # 确认节点运行
   ros2 topic echo /voice_command    # 监控识别结果
   ```

3. ✅ **检查Token状态**
   ```bash
   python3.10 -c "
   from aliyun_nls_token_manager import get_token_manager
   manager = get_token_manager()
   info = manager.get_token_info()
   print(f'Token有效: {info[\"is_valid\"]}')
   print(f'剩余时间: {info[\"remaining_hours\"]}小时')
   "
   ```

4. ✅ **查看ASR日志**
   ```bash
   grep "ASR" ~/xlerobot/logs/*.log
   # 查找识别结果和错误信息
   ```

5. ✅ **音频参数验证**
   - 采样率: 16000 Hz
   - 声道: 单声道 (mono)
   - 位深: 16-bit
   - 格式: WAV/PCM

### LLM推理失败

**症状**: ASR识别成功但没有LLM响应

**排查步骤**:

1. ✅ **检查API密钥**
   ```bash
   echo $QWEN_API_KEY  # 确认已设置
   ```

2. ✅ **检查LLM节点**
   ```bash
   ros2 node list | grep llm_service
   ros2 topic echo /llm_response
   ```

3. ✅ **测试API连接**
   ```bash
   curl -X POST https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions \
     -H "Authorization: Bearer $QWEN_API_KEY" \
     -H "Content-Type: application/json" \
     -d '{"model":"qwen-plus","messages":[{"role":"user","content":"你好"}]}'
   ```

4. ✅ **查看LLM日志**
   ```bash
   grep "LLM\|Qwen" ~/xlerobot/logs/*.log
   # 查找401错误、超时等
   ```

5. ✅ **检查请求格式**
   - 模型: `qwen3-vl-plus` (多模态) 或 `qwen-plus` (纯文本)
   - 消息格式: 符合OpenAI Chat Completion API
   - 超时设置: 30秒

### TTS合成失败

**症状**: 有LLM响应但无语音播放

**排查步骤**:

1. ✅ **检查扬声器**
   ```bash
   aplay -l  # 确认播放设备存在
   aplay /path/to/test.wav  # 测试播放
   ```

2. ✅ **检查TTS节点**
   ```bash
   ros2 node list | grep tts_service
   ros2 topic echo /tts_request
   ```

3. ✅ **检查Token状态**（同ASR）

4. ✅ **查看TTS日志**
   ```bash
   grep "TTS" ~/xlerobot/logs/*.log
   # 查找DNS错误、超时等
   ```

5. ✅ **音频播放测试**
   ```bash
   # 测试pygame播放
   python3.10 -c "
   import pygame
   pygame.mixer.init()
   pygame.mixer.music.load('test.wav')
   pygame.mixer.music.play()
   import time; time.sleep(3)
   "
   ```

### 模块导入错误

**症状**: 日志显示 `ModuleNotFoundError` 或 `ImportError`

**已知问题**:

以下模块名称在24个文件中存在错误引用:

| 错误引用 | 正确引用 |
|---------|---------|
| `websocket_asr_service_final` | `websocket_asr_service` |
| `aliyun_tts_final` | `aliyun_tts_client` |
| `QwenClient` | `QwenAPIClient` |

**排查步骤**:

1. ✅ **检查导入路径**
   ```bash
   grep -r "websocket_asr_service_final" ~/xlerobot/src/
   grep -r "aliyun_tts_final" ~/xlerobot/src/
   grep -r "from.*QwenAPIClient" ~/xlerobot/src/
   ```

2. ✅ **验证模块存在**
   ```bash
   ls -la ~/xlerobot/src/modules/asr/websocket_asr_service.py
   ls -la ~/xlerobot/src/modules/tts/aliyun_tts_client.py
   ls -la ~/xlerobot/src/modules/llm/qwen_client.py
   ```

3. ✅ **检查PYTHONPATH**
   ```bash
   echo $PYTHONPATH  # 应包含: ~/xlerobot/src
   ```

---

## 📊 关键日志消息解读

### 正常启动日志（成功标志）

```
✅ 阿里云NLS Token管理器初始化完成
✅ Token获取成功: [token_preview]
✅ ASR服务初始化成功
✅ LLM服务初始化成功
✅ TTS服务初始化成功
🎯 开始监听唤醒词: 傻强
```

### ASR识别日志

| 日志消息 | 含义 | 状态 |
|---------|------|------|
| `🎯 开始监听唤醒词: 傻强` | 唤醒词监听已启动 | ✅ 正常 |
| `🔔 检测到唤醒词：傻强` | 唤醒成功 | ✅ 正常 |
| `🔊 播放欢迎语...` | 播放"傻强系度,老细有乜可以帮到你!" | ✅ 正常 |
| `🎤 ASR识别成功: [文本]` | 语音识别完成 | ✅ 正常 |
| `❌ ASR识别失败: [错误]` | 识别失败 | ⚠️ 异常 |
| `⏰ Token即将过期，开始自动刷新` | Token自动刷新 | ✅ 正常 |

### LLM推理日志

| 日志消息 | 含义 | 状态 |
|---------|------|------|
| `📤 发送LLM请求: [文本]` | 向通义千问发送请求 | ✅ 正常 |
| `📥 收到LLM响应: [响应]` | 获得LLM回复 | ✅ 正常 |
| `❌ LLM请求失败 (401)` | API密钥无效或过期 | ❌ 严重 |
| `⏱️ LLM请求超时` | 超过30秒无响应 | ⚠️ 异常 |
| `🔄 重试LLM请求 (尝试 X/3)` | 自动重试机制 | ⚠️ 警告 |

### TTS合成日志

| 日志消息 | 含义 | 状态 |
|---------|------|------|
| `🎵 开始TTS合成: [文本]` | 开始语音合成 | ✅ 正常 |
| `✅ TTS合成成功，音频长度: X字节` | 合成完成 | ✅ 正常 |
| `🔊 播放TTS音频...` | 开始播放 | ✅ 正常 |
| `❌ TTS合成失败 (DNS error)` | DNS解析失败 | ⚠️ 异常 |
| `❌ TTS合成失败 (timeout)` | 网络超时 | ⚠️ 异常 |
| `🔄 降级到备用语音` | 使用备用TTS引擎 | ⚠️ 警告 |

### 错误级别日志

| 级别 | 日志前缀 | 严重程度 | 处理建议 |
|------|---------|---------|---------|
| DEBUG | `🔍` | 调试信息 | 仅开发时关注 |
| INFO | `✅ 🎯 📤 📥` | 正常信息 | 记录正常流程 |
| WARNING | `⚠️ 🔄` | 警告 | 需要关注，但不影响核心功能 |
| ERROR | `❌` | 错误 | 需要修复，可能影响功能 |
| CRITICAL | `🔥` | 严重错误 | 立即处理，系统无法正常工作 |

---

## 📈 性能指标正常范围

### 延迟指标（单次交互）

| 阶段 | 正常范围 | 最大可接受 | 说明 |
|------|---------|-----------|------|
| 唤醒词检测 | < 500ms | 1000ms | 从说"傻强"到检测成功 |
| 欢迎语播放 | 1-2s | 3s | 播放"傻强系度,老细有乜可以帮到你!" |
| ASR识别 | 1-3s | 5s | 从用户说话到识别完成 |
| LLM推理 | 2-5s | 10s | 通义千问响应时间 |
| TTS合成 | 1-3s | 5s | 文本转语音 |
| 音频播放 | 取决于内容长度 | - | 实时播放 |
| **总体延迟** | **5-13s** | **20s** | 从唤醒到回复播放完成 |

### 成功率指标

| 指标 | 正常范围 | 最低可接受 | 说明 |
|------|---------|-----------|------|
| 唤醒词检测成功率 | > 90% | 80% | 粤语"傻强"识别准确率 |
| ASR识别成功率 | > 95% | 85% | 清晰环境下识别率 |
| LLM响应成功率 | > 98% | 95% | API调用成功率 |
| TTS合成成功率 | > 98% | 95% | 语音合成成功率 |
| **端到端成功率** | **> 85%** | **70%** | 完整交互链路成功率 |

### 资源使用指标（RDK X5）

| 资源 | 正常范围 | 最大可接受 | 说明 |
|------|---------|-----------|------|
| CPU使用率 | 30-50% | 80% | 4核心平均使用率 |
| 内存使用 | 500MB-1GB | 2GB | 包括ROS2和Python进程 |
| 网络带宽 | 50-200 KB/s | 1 MB/s | 峰值在TTS合成时 |
| 麦克风采样率 | 16000 Hz | - | 固定值 |
| 音频缓冲延迟 | < 100ms | 200ms | 音频处理缓冲 |

### Token管理指标

| 指标 | 正常范围 | 说明 |
|------|---------|------|
| Token有效期 | 24小时 | 阿里云NLS Token标准有效期 |
| Token刷新间隔 | 每23.5小时 | 提前30分钟刷新（buffer_seconds=300） |
| Token缓存命中率 | > 99% | 大部分时间使用缓存Token |
| Token刷新失败率 | < 1% | 网络正常情况下 |

---

## 🚨 紧急恢复步骤

### 场景1: 服务完全无响应

```bash
# 1. 强制停止所有服务
pkill -f start_epic1_services
pkill -f ros2

# 2. 清理ROS2缓存
rm -rf ~/.ros/log/*
rm -rf /tmp/ros*

# 3. 重置环境
source ~/xlerobot/xlerobot_env.sh

# 4. 重启服务
cd ~/xlerobot
./start_voice_assistant.sh restart

# 5. 验证启动
./start_voice_assistant.sh status
ros2 node list
```

### 场景2: Token失效无法刷新

```bash
# 1. 删除过期Token缓存
rm -f /tmp/aliyun_nls_token.cache

# 2. 验证API密钥
cat ~/xlerobot/.env | grep ALIBABA_CLOUD

# 3. 手动测试Token获取
python3.10 -c "
from aliyun_nls_token_manager import AliyunNLSTokenManager
manager = AliyunNLSTokenManager()
token = manager.get_token()
print(f'Token: {token[:20]}...' if token else 'Failed')
"

# 4. 重启服务
./start_voice_assistant.sh restart
```

### 场景3: 音频设备无法访问

```bash
# 1. 检查设备状态
arecord -l
aplay -l

# 2. 重启ALSA服务
sudo systemctl restart alsa-restore
sudo alsactl restore

# 3. 测试音频设备
arecord -d 3 -f cd test.wav
aplay test.wav

# 4. 如果仍失败，重启系统
sudo reboot
```

### 场景4: ROS2节点启动失败

```bash
# 1. 检查ROS2环境
echo $ROS_DISTRO  # 应为: humble
source /opt/ros/humble/setup.bash

# 2. 清理ROS2进程
pkill -f ros2
rm -rf /tmp/ros*

# 3. 检查端口占用
netstat -tunlp | grep -E ':(111|7400|7401)'

# 4. 重启ROS2 daemon
ros2 daemon stop
ros2 daemon start

# 5. 重新启动服务
./start_voice_assistant.sh restart
```

### 场景5: 模块导入错误

```bash
# 1. 验证PYTHONPATH
echo $PYTHONPATH
export PYTHONPATH="~/xlerobot/src:$PYTHONPATH"

# 2. 检查模块文件存在
ls -la ~/xlerobot/src/modules/asr/websocket_asr_service.py
ls -la ~/xlerobot/src/modules/tts/aliyun_tts_client.py
ls -la ~/xlerobot/src/modules/llm/qwen_client.py

# 3. 测试导入
python3.10 -c "
import sys
sys.path.insert(0, '~/xlerobot/src')
from modules.asr.websocket_asr_service import WebSocketASRService
from modules.tts.aliyun_tts_client import AliyunTTSClient
from modules.llm.qwen_client import QwenAPIClient
print('All imports successful')
"

# 4. 如果失败，检查是否有错误的模块名引用
grep -r "websocket_asr_service_final" ~/xlerobot/src/
grep -r "aliyun_tts_final" ~/xlerobot/src/
```

### 场景6: 系统完全重置

```bash
# ⚠️ 警告：这将清除所有运行状态

# 1. 停止所有服务
./start_voice_assistant.sh stop
pkill -f python3.10
pkill -f ros2

# 2. 清理所有缓存
rm -rf ~/.ros/log/*
rm -rf /tmp/ros*
rm -f /tmp/aliyun_nls_token.cache

# 3. 重新加载环境
source ~/xlerobot/xlerobot_env.sh

# 4. 验证环境
python3.10 --version
echo $ROS_DISTRO
echo $PYTHONPATH

# 5. 重新安装依赖（如有必要）
pip3.10 install -r ~/xlerobot/requirements.txt

# 6. 完整启动
cd ~/xlerobot
./start_voice_assistant.sh

# 7. 运行完整测试
python3.10 tests/verify_epic1_complete_functionality.py
```

---

## 🔑 环境变量速查

### 必需环境变量

```bash
# ROS2环境
export ROS_DISTRO=humble
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1  # 可选：限制本地通信

# Python路径
export PYTHONPATH="~/xlerobot/src:$PYTHONPATH"

# 阿里云NLS认证（ASR + TTS共用，从.env加载）
# 说明: ASR和TTS都使用阿里云NLS服务，共用同一套Token认证系统
# 认证流程: AccessKey ID/Secret → 获取Token → 调用ASR/TTS服务
export ALIBABA_CLOUD_ACCESS_KEY_ID="LTAI5tQ4E2YNzZkGn9g1JqeY"      # 用于: ASR + TTS
export ALIBABA_CLOUD_ACCESS_KEY_SECRET="Hr1xZdcdz3D9OgFnH1nvWz5rldXVeI" # 用于: ASR + TTS
export ALIYUN_NLS_APPKEY="4G5BCMccTCW8nC8w"                        # 用于: ASR + TTS

# 通义千问LLM认证（独立）
export QWEN_API_KEY="sk-600a739fb3f54f338616254c1c69c1f6"          # 用于: LLM

# 可选配置
export ALIYUN_CONFIG_PATH="~/xlerobot/config/aliyun_nls_config.yaml"
export LOG_LEVEL="INFO"  # DEBUG, INFO, WARNING, ERROR, CRITICAL
```

### 验证环境变量

```bash
# 快速验证所有关键变量
env | grep -E 'ROS_|PYTHON|ALIBABA|ALIYUN|QWEN' | sort

# 或使用脚本验证
bash ~/xlerobot/complete_env_check.sh
```

---

## 🤖 ROS2节点速查

### 节点架构

```
┌─────────────────────────────────────────────────────────┐
│           voice_assistant_coordinator                    │
│          (系统协调器 - 中央控制节点)                      │
└─────────────────────────────────────────────────────────┘
             ↑ /voice_command           ↓ /tts_request
             |                          |
┌────────────┴──────┐       ┌──────────┴─────────┐
│   asr_bridge      │       │   tts_service      │
│  (ASR服务节点)     │       │  (TTS服务节点)      │
└───────────────────┘       └────────────────────┘
             ↑
             | /llm_request
             ↓ /llm_response
┌───────────────────┐
│   llm_service     │
│  (LLM服务节点)     │
└───────────────────┘
```

### 节点职责

| 节点名称 | 职责 | 发布话题 | 订阅话题 |
|---------|------|---------|---------|
| `asr_bridge` | 音频采集、唤醒检测、语音识别 | `/voice_command` | - |
| `llm_service` | 自然语言理解、对话管理 | `/llm_response` | `/llm_request` |
| `tts_service` | 文本预处理、语音合成、音频播放 | - | `/tts_request` |
| `voice_assistant_coordinator` | 系统协调、流程控制、状态管理 | `/llm_request`, `/tts_request`, `/system_status` | `/voice_command`, `/llm_response` |

### 消息类型

```python
# /voice_command (String)
{
  "data": "今日天气点样？"
}

# /llm_request (String - JSON格式)
{
  "text": "今日天气点样？",
  "session_id": "session_12345",
  "context": {}
}

# /llm_response (String - JSON格式)
{
  "response": "今日广州天气晴朗...",
  "session_id": "session_12345"
}

# /tts_request (String - JSON格式)
{
  "text": "今日广州天气晴朗...",
  "voice": "xiaoyun",
  "speed": 0
}

# /system_status (String - JSON格式)
{
  "state": "LISTENING_COMMAND",
  "timestamp": 1234567890,
  "nodes_active": ["asr", "llm", "tts"]
}
```

### 节点操作命令

```bash
# 列出所有节点
ros2 node list

# 查看节点详细信息
ros2 node info /asr_bridge
ros2 node info /llm_service
ros2 node info /tts_service
ros2 node info /voice_assistant_coordinator

# 监控话题
ros2 topic echo /voice_command
ros2 topic echo /llm_response
ros2 topic echo /system_status

# 查看话题信息
ros2 topic info /voice_command
ros2 topic hz /voice_command  # 消息频率

# 手动发布测试消息
ros2 topic pub /llm_request std_msgs/msg/String \
  'data: "{\"text\": \"你好\", \"session_id\": \"test_123\"}"'
```

---

## 🌐 API服务状态检查

### 阿里云NLS服务

```bash
# 1. 检查网络连接
ping nls-gateway-cn-shanghai.aliyuncs.com

# 2. 检查DNS解析
nslookup nls-gateway-cn-shanghai.aliyuncs.com

# 3. 测试Token获取
python3.10 -c "
from aliyun_nls_token_manager import get_token_manager
manager = get_token_manager()
token = manager.get_token()
if token:
    print(f'✅ Token获取成功: {token[:20]}...')
    info = manager.get_token_info()
    print(f'剩余时间: {info[\"remaining_hours\"]}小时')
else:
    print('❌ Token获取失败')
"

# 4. 测试ASR服务
python3.10 tests/test_aliyun_api_integration.py --test asr

# 5. 测试TTS服务
python3.10 tests/test_aliyun_api_integration.py --test tts
```

### 通义千问API服务

```bash
# 1. 检查API密钥
echo $QWEN_API_KEY

# 2. 测试API连接
curl -X POST https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions \
  -H "Authorization: Bearer $QWEN_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{
    "model": "qwen-plus",
    "messages": [
      {"role": "system", "content": "你是一个友好的粤语语音助手"},
      {"role": "user", "content": "你好"}
    ]
  }'

# 3. 测试LLM服务
python3.10 tests/test_aliyun_api_integration.py --test llm

# 4. 检查LLM响应时间
python3.10 -c "
import time
from modules.llm.qwen_client import QwenAPIClient
client = QwenAPIClient()
start = time.time()
response = client.chat('你好')
elapsed = time.time() - start
print(f'响应时间: {elapsed:.2f}秒')
print(f'响应内容: {response}')
"
```

### API健康状态判断

| 服务 | 健康指标 | 异常阈值 |
|------|---------|---------|
| **阿里云NLS ASR** | Token获取成功 + 识别延迟 < 3s | 识别失败率 > 10% 或延迟 > 5s |
| **阿里云NLS TTS** | Token获取成功 + 合成延迟 < 3s | 合成失败率 > 10% 或延迟 > 5s |
| **通义千问API** | HTTP 200 + 响应延迟 < 5s | 401/429错误 或延迟 > 10s |

### 常见API错误码

| 错误码 | 含义 | 解决方案 |
|-------|------|---------|
| **401** | 认证失败 | 检查API密钥是否正确、Token是否过期 |
| **403** | 权限不足 | 确认账号已开通相应服务 |
| **429** | 请求限流 | 降低请求频率，或升级API配额 |
| **500** | 服务器错误 | 稍后重试，或联系阿里云客服 |
| **503** | 服务不可用 | 检查阿里云服务状态页 |
| **DNS Error** | 域名解析失败 | 检查网络连接、DNS配置 |
| **Timeout** | 请求超时 | 检查网络速度、增加超时时间 |

---

## 📚 相关文档

- **详细架构文档**: [xlerobot-system-architecture-dataflow.md](./xlerobot-system-architecture-dataflow.md)
- **可视化流程图**: [xlerobot-dataflow-diagrams.md](./xlerobot-dataflow-diagrams.md)
- **项目概览**: [project-overview.md](./project-overview.md)
- **技术栈文档**: [tech-stack-documentation.md](./tech-stack-documentation.md)
- **Epic 1 PRD**: [prd-epic1-multimodal-services.md](./prd-epic1-multimodal-services.md)

---

## 🆘 获取帮助

### 日志位置

```bash
# 系统日志
~/xlerobot/logs/

# ROS2日志
~/.ros/log/

# 启动脚本日志
./start_voice_assistant.sh logs
```

### 诊断报告生成

```bash
# 生成完整诊断报告
bash ~/xlerobot/scripts/generate_diagnostic_report.sh

# 报告将包含:
# - 环境变量状态
# - ROS2节点状态
# - API服务连通性
# - 音频设备状态
# - 最近错误日志
```

### 问题报告模板

当遇到问题时，请收集以下信息：

1. **环境信息**
   ```bash
   python3.10 --version
   echo $ROS_DISTRO
   uname -a
   ```

2. **错误日志**
   ```bash
   ./start_voice_assistant.sh logs | tail -100
   ```

3. **节点状态**
   ```bash
   ros2 node list
   ros2 topic list
   ```

4. **测试结果**
   ```bash
   python3.10 tests/verify_epic1_complete_functionality.py
   ```

---

**最后更新**: 2025-11-16
**文档版本**: v1.0
**适用范围**: XLeRobot Epic 1 全在线架构
