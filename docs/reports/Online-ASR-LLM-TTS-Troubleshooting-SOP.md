# 在线服务 ASR→LLM→TTS 完整流程排查 SOP

> **架构**：全在线服务（阿里云ASR + 通义千问LLM + 阿里云TTS）  
> **设备**：RDK X5  
> **版本**：v2.0（在线服务版）  
> **日期**：2025-11-14

---

## 📋 目录

1. [排查原则](#排查原则)
2. [Phase 1: 网络和基础环境](#phase-1-网络和基础环境)
3. [Phase 2: API凭证验证](#phase-2-api凭证验证)
4. [Phase 3: ASR在线服务测试](#phase-3-asr在线服务测试)
5. [Phase 4: LLM在线服务测试](#phase-4-llm在线服务测试)
6. [Phase 5: TTS在线服务测试](#phase-5-tts在线服务测试)
7. [Phase 6: 唤醒词和麦克风](#phase-6-唤醒词和麦克风)
8. [Phase 7: 程序逻辑和流程](#phase-7-程序逻辑和流程)
9. [Phase 8: 完整链路测试](#phase-8-完整链路测试)
10. [常见问题快速定位](#常见问题快速定位)

---

## 排查原则

### 🎯 核心思路

在线服务架构的排查重点：

```
网络层（连接性、DNS、代理）
    ↓
凭证层（AccessKey、Token、API Key）
    ↓
服务层（ASR、LLM、TTS API调用）
    ↓
设备层（麦克风、音频格式）
    ↓
逻辑层（程序流程、状态管理）
    ↓
集成层（完整端到端测试）
```

### ✅ 与离线服务的关键区别

**在线服务特点**：
- ✅ 无需本地模型加载
- ✅ 无需NPU/GPU加速
- ❌ 强依赖网络连接
- ❌ 受限于API配额和速率
- ❌ 有网络延迟
- ❌ 需要管理Token/API Key

---

## Phase 1: 网络和基础环境

**目的**：确认设备能正常访问阿里云服务

### 1.1 基础网络检查

**检查内容**：
```bash
# 检查网络接口状态
ip addr show

# 测试外网连通性
ping -c 3 8.8.8.8

# 测试DNS解析
nslookup nls-gateway.cn-shanghai.aliyuncs.com
nslookup dashscope.aliyuncs.com

# 检查默认路由
ip route show

# 检查防火墙状态
sudo iptables -L -n
```

**判断标准**：
- ✅ **Pass**: 
  - 网络接口有IP
  - 能ping通外网
  - DNS解析正常
  - 无阻断防火墙规则
- ❌ **Fail**:
  - 无网络 → 检查物理连接或DHCP
  - DNS失败 → 修改/etc/resolv.conf
  - 防火墙阻断 → 调整规则

### 1.2 阿里云服务端点测试

**检查内容**：
```bash
# 测试ASR服务端点
curl -I https://nls-gateway.cn-shanghai.aliyuncs.com

# 测试Token获取端点
curl -I https://nls-meta.cn-shanghai.aliyuncs.com

# 测试通义千问端点
curl -I https://dashscope.aliyuncs.com

# 如果有代理，检查代理设置
echo $http_proxy
echo $https_proxy
```

**判断标准**：
- ✅ **Pass**: 所有端点返回HTTP 200/301/302（说明能连通）
- ❌ **Fail**:
  - 连接超时 → 网络问题或被墙
  - 证书错误 → 时间不同步（检查`date`命令）
  - 代理设置错误 → 取消代理或修正配置

### 1.3 系统时间检查

**检查内容**：
```bash
# 查看系统时间
date

# 查看时区
timedatectl

# 如果时间不对，同步时间
sudo ntpdate ntp.aliyun.com
# 或
sudo timedatectl set-ntp true
```

**判断标准**：
- ✅ **Pass**: 系统时间准确（误差<5分钟）
- ❌ **Fail**: 时间严重偏差 → API签名会失败

### 1.4 Python环境和依赖

**检查内容**：
```bash
# 确认Python版本
python3 --version

# 检查关键库
python3 -c "import requests; print('requests OK')"
python3 -c "import pyaudio; print('pyaudio OK')"
python3 -c "import json; print('json OK')"

# 如果缺少requests
pip3 install requests
```

**判断标准**：
- ✅ **Pass**: Python 3.8+，requests库可用
- ❌ **Fail**: 缺少库 → 使用pip安装

---

## Phase 2: API凭证验证

**目的**：确认所有API密钥和凭证正确有效

### 2.1 阿里云AccessKey检查

**检查内容**：
```bash
# 查看AccessKey配置位置
echo "检查环境变量:"
echo $ALIYUN_AK_ID
echo $ALIYUN_AK_SECRET

# 或检查配置文件
cat ~/.aliyun/config.json
cat ~/xlerobot/config/alibaba_config.json

# 确认AppKey
echo $ALIYUN_ASR_APPKEY
```

**判断标准**：
- ✅ **Pass**: AccessKey ID和Secret存在且格式正确（ID通常是LT开头）
- ❌ **Fail**: 凭证缺失或格式错误 → 重新配置

### 2.2 阿里云Token获取测试

**检查内容**：
```bash
# 手动测试Token获取
python3 << 'EOF'
import requests
import json

ACCESS_KEY_ID = "你的AccessKey_ID"
ACCESS_KEY_SECRET = "你的AccessKey_Secret"

response = requests.post(
    "https://nls-meta.cn-shanghai.aliyuncs.com/pop/2018-05-18/tokens",
    json={
        "AccessKeyId": ACCESS_KEY_ID,
        "Action": "CreateToken"
    },
    timeout=10
)

print(f"状态码: {response.status_code}")
if response.status_code == 200:
    token_data = response.json()
    print(f"✅ Token获取成功")
    print(f"Token: {token_data['Token']['Id'][:30]}...")
    print(f"过期时间: {token_data['Token']['ExpireTime']}")
else:
    print(f"❌ Token获取失败")
    print(f"响应: {response.text}")
EOF
```

**判断标准**：
- ✅ **Pass**: 返回200，包含Token和ExpireTime
- ❌ **Fail**:
  - 401/403 → AccessKey错误或无权限
  - 超时 → 网络问题
  - 400 → 请求格式错误

### 2.3 通义千问API Key验证

**检查内容**：
```bash
# 检查API Key
echo $DASHSCOPE_API_KEY

# 测试API Key有效性
python3 << 'EOF'
import requests

API_KEY = "你的通义千问API_KEY"

response = requests.get(
    "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation",
    headers={
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }
)

print(f"状态码: {response.status_code}")
if response.status_code in [200, 400]:  # 400说明密钥有效但请求参数不对
    print("✅ API Key有效")
else:
    print(f"❌ API Key可能无效")
    print(f"响应: {response.text}")
EOF
```

**判断标准**：
- ✅ **Pass**: 返回200或400（说明认证通过）
- ❌ **Fail**: 401/403 → API Key错误或过期

### 2.4 配额和余额检查

**检查内容**：
```bash
# 登录阿里云控制台检查：
# 1. 智能语音交互 - 用量统计
# 2. 通义千问 - API额度
# 3. 确认没有欠费

# 可以通过curl查看API响应中的quota信息
```

**判断标准**：
- ✅ **Pass**: 有足够配额，账户无欠费
- ❌ **Fail**: 配额用尽或欠费 → 充值或购买资源包

---

## Phase 3: ASR在线服务测试

**目的**：单独验证阿里云ASR识别功能

### 3.1 录制测试音频

**检查内容**：
```bash
# 录制3秒测试音频（16kHz单声道）
arecord -D hw:0,0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_asr.wav

# 播放验证
aplay /tmp/test_asr.wav

# 查看文件信息
ls -lh /tmp/test_asr.wav
file /tmp/test_asr.wav
```

**判断标准**：
- ✅ **Pass**: 
  - 文件大小约96KB（3秒×16kHz×2字节）
  - 播放能听到清晰录音
- ❌ **Fail**: 返回Phase 1.4或检查麦克风硬件

### 3.2 转换为PCM格式

**检查内容**：
```bash
# 阿里云ASR要求PCM格式，转换wav到pcm
ffmpeg -i /tmp/test_asr.wav -f s16le -ar 16000 -ac 1 /tmp/test_asr.pcm

# 或使用sox
sox /tmp/test_asr.wav -r 16000 -c 1 -b 16 /tmp/test_asr.pcm

# 验证文件大小
ls -lh /tmp/test_asr.pcm
```

**判断标准**：
- ✅ **Pass**: PCM文件大小约96KB
- ❌ **Fail**: 安装ffmpeg或sox

### 3.3 调用ASR API测试

**检查内容**：
```bash
python3 << 'EOF'
import requests
import base64

# 配置
TOKEN = "你在Phase2获取的Token"
APPKEY = "你的ASR_APPKEY"

# 读取PCM文件
with open("/tmp/test_asr.pcm", "rb") as f:
    audio_data = base64.b64encode(f.read()).decode('utf-8')

# 调用ASR API
response = requests.post(
    "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/asr",
    headers={
        "Content-Type": "application/json",
        "X-NLS-Token": TOKEN  # 关键：必须用X-NLS-Token
    },
    json={
        "appkey": APPKEY,
        "format": "pcm",
        "sample_rate": 16000,
        "enable_intermediate_result": False,
        "enable_punctuation_prediction": True,
        "enable_inverse_text_normalization": True,
        "audio": audio_data
    },
    timeout=30
)

print(f"ASR响应状态码: {response.status_code}")
if response.status_code == 200:
    result = response.json()
    print("✅ ASR识别成功")
    print(f"识别文本: {result.get('result', 'No result')}")
else:
    print("❌ ASR识别失败")
    print(f"响应: {response.text}")
EOF
```

**判断标准**：
- ✅ **Pass**: 返回200，能识别出文本（即使不完全准确）
- ❌ **Fail**:
  - 400 → 检查请求格式、Token、AppKey
  - 401 → Token无效或过期
  - 超时 → 音频过大或网络慢

### 3.4 粤语ASR测试

**检查内容**：
```bash
# 录制粤语音频
echo "请说粤语：'你好呀' 或 '而家几点'"
arecord -D hw:0,0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_cantonese.wav

# 转换格式
ffmpeg -i /tmp/test_cantonese.wav -f s16le -ar 16000 -ac 1 /tmp/test_cantonese.pcm

# 调用ASR（修改上面脚本中的文件路径）
# 关键：可能需要设置language参数为"yue-CN"（粤语）
```

**判断标准**：
- ✅ **Pass**: 能识别粤语文本
- ⚠️ **Warning**: 识别为普通话拼音 → 检查是否支持粤语或调整模型
- ❌ **Fail**: 完全无法识别 → 音频质量或格式问题

---

## Phase 4: LLM在线服务测试

**目的**：验证通义千问API调用

### 4.1 简单对话测试

**检查内容**：
```bash
python3 << 'EOF'
import requests
import json

API_KEY = "你的通义千问API_KEY"

response = requests.post(
    "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation",
    headers={
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    },
    json={
        "model": "qwen-max",
        "input": {
            "messages": [
                {"role": "user", "content": "你好"}
            ]
        },
        "parameters": {
            "result_format": "message"
        }
    },
    timeout=30
)

print(f"LLM响应状态码: {response.status_code}")
if response.status_code == 200:
    result = response.json()
    print("✅ LLM回复成功")
    output = result['output']['choices'][0]['message']['content']
    print(f"回复: {output}")
else:
    print("❌ LLM调用失败")
    print(f"响应: {response.text}")
EOF
```

**判断标准**：
- ✅ **Pass**: 返回200，有合理的中文回复
- ❌ **Fail**:
  - 401 → API Key错误
  - 429 → 请求频率超限
  - 超时 → 网络或服务繁忙

### 4.2 粤语理解测试

**检查内容**：
```bash
python3 << 'EOF'
import requests
import json

API_KEY = "你的通义千问API_KEY"

# 测试粤语输入
cantonese_prompts = [
    "你好呀",
    "而家几点？",
    "今日天气点样？"
]

for prompt in cantonese_prompts:
    response = requests.post(
        "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation",
        headers={
            "Authorization": f"Bearer {API_KEY}",
            "Content-Type": "application/json"
        },
        json={
            "model": "qwen-max",
            "input": {
                "messages": [
                    {"role": "user", "content": prompt}
                ]
            }
        },
        timeout=30
    )
    
    if response.status_code == 200:
        result = response.json()
        output = result['output']['choices'][0]['message']['content']
        print(f"提示: {prompt}")
        print(f"回复: {output[:100]}")
        print()
    else:
        print(f"❌ 失败: {prompt}")
EOF
```

**判断标准**：
- ✅ **Pass**: LLM能理解粤语输入并给出合理回复
- ⚠️ **Warning**: 只能用普通话回复 → 可接受
- ❌ **Fail**: 完全无法理解 → 考虑添加system prompt指导

### 4.3 响应速度测试

**检查内容**：
```bash
python3 << 'EOF'
import requests
import time

API_KEY = "你的通义千问API_KEY"

prompts = ["你好", "介绍一下自己", "解释什么是人工智能"]

for prompt in prompts:
    start = time.time()
    response = requests.post(
        "https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation",
        headers={
            "Authorization": f"Bearer {API_KEY}",
            "Content-Type": "application/json"
        },
        json={
            "model": "qwen-max",
            "input": {
                "messages": [
                    {"role": "user", "content": prompt}
                ]
            }
        },
        timeout=30
    )
    elapsed = time.time() - start
    
    if response.status_code == 200:
        result = response.json()
        output = result['output']['choices'][0]['message']['content']
        print(f"提示: {prompt}")
        print(f"耗时: {elapsed:.2f}秒")
        print(f"回复长度: {len(output)}字符")
        print()
EOF
```

**判断标准**：
- ✅ **Pass**: 响应时间<10秒
- ⚠️ **Warning**: 10-20秒（可接受但较慢）
- ❌ **Fail**: >20秒或超时 → 网络问题或服务繁忙

---

## Phase 5: TTS在线服务测试

**目的**：验证阿里云TTS语音合成

### 5.1 简单文本合成测试

**检查内容**：
```bash
python3 << 'EOF'
import requests

TOKEN = "你在Phase2获取的Token"
APPKEY = "你的TTS_APPKEY"

response = requests.post(
    "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts",
    headers={
        "Content-Type": "application/json",
        "X-NLS-Token": TOKEN
    },
    json={
        "appkey": APPKEY,
        "text": "你好，我是小强",
        "format": "wav",
        "sample_rate": 16000,
        "voice": "sijia"  # 粤语女声
    },
    timeout=30
)

print(f"TTS响应状态码: {response.status_code}")
if response.status_code == 200:
    # 保存音频
    with open("/tmp/test_tts_output.wav", "wb") as f:
        f.write(response.content)
    print("✅ TTS合成成功")
    print(f"文件大小: {len(response.content)} 字节")
    print("播放: aplay /tmp/test_tts_output.wav")
else:
    print("❌ TTS合成失败")
    print(f"响应: {response.text}")
EOF

# 播放合成的语音
aplay /tmp/test_tts_output.wav
```

**判断标准**：
- ✅ **Pass**: 
  - 返回200
  - 文件大小合理（几KB到几十KB）
  - 播放能听到清晰语音
- ❌ **Fail**:
  - 400 → Token或参数错误
  - 文件太小或播放无声 → 格式问题

### 5.2 粤语TTS测试

**检查内容**：
```bash
python3 << 'EOF'
import requests

TOKEN = "你在Phase2获取的Token"
APPKEY = "你的TTS_APPKEY"

# 粤语文本
cantonese_texts = [
    "你好呀",
    "而家几点？",
    "今日天气好好"
]

for i, text in enumerate(cantonese_texts):
    response = requests.post(
        "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts",
        headers={
            "Content-Type": "application/json",
            "X-NLS-Token": TOKEN
        },
        json={
            "appkey": APPKEY,
            "text": text,
            "format": "wav",
            "sample_rate": 16000,
            "voice": "sijia"  # 或其他粤语voice
        },
        timeout=30
    )
    
    if response.status_code == 200:
        output_file = f"/tmp/tts_cantonese_{i}.wav"
        with open(output_file, "wb") as f:
            f.write(response.content)
        print(f"✅ 合成: {text} -> {output_file}")
    else:
        print(f"❌ 合成失败: {text}")
EOF

# 逐个播放
for i in {0..2}; do
    echo "播放: 粤语测试 $i"
    aplay /tmp/tts_cantonese_$i.wav
done
```

**判断标准**：
- ✅ **Pass**: 合成的语音是粤语发音
- ⚠️ **Warning**: 普通话发音 → voice参数错误，查阅文档选择正确的粤语voice
- ❌ **Fail**: 无法合成 → Token或AppKey问题

### 5.3 TTS性能测试

**检查内容**：
```bash
python3 << 'EOF'
import requests
import time

TOKEN = "你在Phase2获取的Token"
APPKEY = "你的TTS_APPKEY"

texts = [
    "短句",
    "这是一个中等长度的句子",
    "这是一个很长的句子包含了很多字用来测试TTS在处理长文本时的表现"
]

for text in texts:
    start = time.time()
    response = requests.post(
        "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts",
        headers={
            "Content-Type": "application/json",
            "X-NLS-Token": TOKEN
        },
        json={
            "appkey": APPKEY,
            "text": text,
            "format": "wav",
            "sample_rate": 16000,
            "voice": "sijia"
        },
        timeout=30
    )
    elapsed = time.time() - start
    
    if response.status_code == 200:
        print(f"文本: {text[:20]}...")
        print(f"长度: {len(text)}字符")
        print(f"耗时: {elapsed:.2f}秒")
        print(f"音频大小: {len(response.content)} 字节")
        print()
EOF
```

**判断标准**：
- ✅ **Pass**: 合成耗时<3秒（短句），<5秒（长句）
- ❌ **Fail**: 耗时过长 → 网络问题或服务繁忙

---

## Phase 6: 唤醒词和麦克风

**目的**：验证本地唤醒词检测功能

### 6.1 麦克风基本功能

**检查内容**：
```bash
# 列出音频设备
arecord -l
aplay -l

# 检查权限
ls -l /dev/snd/*
groups $USER | grep audio

# 录音测试
arecord -D hw:0,0 -f S16_LE -r 16000 -c 1 -d 3 /tmp/test_mic.wav
aplay /tmp/test_mic.wav
```

**判断标准**：
- ✅ **Pass**: 能录制和播放
- ❌ **Fail**: 返回Phase 1检查硬件

### 6.2 唤醒词进程检查

**检查内容**：
```bash
# 检查唤醒词相关进程
ps aux | grep -i wake
ps aux | grep -i sherpa
ps aux | grep python | grep wake

# 检查进程日志
tail -50 ~/xlerobot/logs/wake_word_detector.log
```

**判断标准**：
- ✅ **Pass**: 有唤醒词检测进程在运行，日志显示"Listening..."
- ❌ **Fail**: 无进程 → 启动唤醒词检测程序

### 6.3 唤醒词配置检查

**检查内容**：
```bash
# 查找配置文件
find ~/xlerobot -name "*wake*config*"

# 查看配置
cat ~/xlerobot/config/wake_word_config.yaml

# 确认关键字
grep -r "傻强" ~/xlerobot/
```

**判断标准**：
- ✅ **Pass**: 配置文件存在，包含"傻强"关键词
- ❌ **Fail**: 配置缺失或错误 → 修正配置

### 6.4 手动触发测试

**检查内容**：
```bash
# 如果使用ROS2，手动发送唤醒信号
ros2 topic pub --once /wake_word_detected std_msgs/msg/Bool "{data: true}"

# 或直接调用唤醒后的处理函数
# 查看程序是否有响应
```

**判断标准**：
- ✅ **Pass**: 手动触发后能看到ASR启动
- ❌ **Fail**: 无响应 → 检查程序逻辑

---

## Phase 7: 程序逻辑和流程

**目的**：检查主程序的流程控制

### 7.1 程序启动检查

**检查内容**：
```bash
# 查看主程序入口
ls -la ~/xlerobot/main.py
ls -la ~/xlerobot/start_xlerobot.sh

# 查看启动日志
tail -100 ~/xlerobot/logs/xlerobot.log

# 检查是否有Python错误
tail -100 ~/xlerobot/logs/xlerobot.log | grep -i "error\|exception\|traceback"
```

**判断标准**：
- ✅ **Pass**: 程序正常启动，无Python异常
- ❌ **Fail**: 有异常 → 根据错误信息修复代码

### 7.2 状态机检查

**检查内容**：
```bash
# 检查程序当前状态
# 如果程序提供状态查询接口
curl http://localhost:8080/status

# 或查看日志中的状态信息
tail -50 ~/xlerobot/logs/xlerobot.log | grep -i "state\|status"
```

**判断标准**：
- ✅ **Pass**: 程序处于"等待唤醒"状态
- ❌ **Fail**: 程序卡在某个中间状态 → 重启或调试

### 7.3 错误处理检查

**检查内容**：
```bash
# 查看是否有API调用失败记录
grep -i "api.*fail\|timeout\|error" ~/xlerobot/logs/xlerobot.log | tail -20

# 查看是否有异常捕获
grep -i "exception\|traceback" ~/xlerobot/logs/xlerobot.log | tail -20
```

**判断标准**：
- ✅ **Pass**: 偶尔有错误但有重试或恢复机制
- ❌ **Fail**: 大量错误且程序崩溃 → 修复错误处理逻辑

### 7.4 日志级别和详细度

**检查内容**：
```bash
# 查看日志配置
cat ~/xlerobot/config/logging_config.yaml

# 临时提高日志级别（如果支持）
# 修改配置文件中的level: DEBUG

# 重启程序查看详细日志
```

**判断标准**：
- ✅ **Pass**: 日志包含足够的调试信息
- ❌ **Fail**: 日志太少 → 提高日志级别

---

## Phase 8: 完整链路测试

**目的**：端到端验证整个流程

### 8.1 手动触发完整流程

**检查内容**：
```bash
# 准备：打开多个终端窗口

# 终端1: 监控日志
tail -f ~/xlerobot/logs/xlerobot.log

# 终端2: 执行测试
# 方法1: 真实说唤醒词
echo "请说：傻强"
# 等待几秒
echo "请说一句话：今天天气怎么样？"

# 方法2: 模拟唤醒
ros2 topic pub --once /wake_word_detected std_msgs/msg/Bool "{data: true}"
# 然后说话

# 方法3: 完全模拟（不需要语音）
ros2 topic pub --once /asr_result std_msgs/msg/String "{data: '今天天气怎么样'}"
```

**观察点**：
1. 唤醒词是否被检测到
2. ASR是否开始录音（日志应显示"Recording..."）
3. ASR是否返回识别结果
4. LLM是否收到输入并生成回复
5. TTS是否合成语音
6. 音箱是否播放语音

**判断标准**：
- ✅ **Pass**: 所有步骤都顺利执行，最后听到TTS语音
- ❌ **Fail**: 在某个步骤卡住 → 返回对应Phase排查

### 8.2 连续对话测试

**检查内容**：
```bash
# 进行3轮完整对话

# 轮1
说: "傻强"
等待提示音
说: "你好"
等待回复

# 轮2
说: "傻强"
等待提示音
说: "今天天气怎么样"
等待回复

# 轮3
说: "傻强"
等待提示音
说: "谢谢"
等待回复
```

**判断标准**：
- ✅ **Pass**: 3轮对话都成功
- ❌ **Fail**: 第2轮开始失败 → 检查状态重置逻辑

### 8.3 错误恢复测试

**检查内容**：
```bash
# 测试异常情况

# 情况1: 唤醒后不说话（长时间静音）
说: "傻强"
等待30秒不说话
# 程序应该超时返回等待状态

# 情况2: 网络临时中断
说: "傻强"
sudo iptables -A OUTPUT -d dashscope.aliyuncs.com -j DROP
说: "你好"
# 程序应该报错并恢复
sudo iptables -D OUTPUT -d dashscope.aliyuncs.com -j DROP

# 情况3: 说不清楚的话
说: "傻强"
说: "啊啊啊啊啊"（含糊不清）
# 程序应该处理识别失败情况
```

**判断标准**：
- ✅ **Pass**: 异常情况能优雅处理并恢复
- ❌ **Fail**: 程序崩溃或卡死 → 添加异常处理

### 8.4 性能和延迟测试

**检查内容**：
```bash
# 测量端到端延迟
python3 << 'EOF'
import time

# 记录时间点
t1 = time.time()  # 说唤醒词的时间
# 手动记录或通过日志解析

# t2: 唤醒词被检测到
# t3: ASR开始录音
# t4: ASR返回结果
# t5: LLM返回结果
# t6: TTS开始播放

# 计算各阶段耗时
# 唤醒检测: t2-t1
# ASR识别: t4-t3
# LLM生成: t5-t4
# TTS合成: t6-t5
# 总延迟: t6-t1
EOF
```

**判断标准**：
- ✅ **Pass**: 
  - 唤醒检测<1秒
  - ASR识别<3秒
  - LLM生成<5秒
  - TTS合成<3秒
  - 总延迟<15秒
- ⚠️ **Warning**: 总延迟15-30秒（可用但体验差）
- ❌ **Fail**: 总延迟>30秒 → 优化或排查瓶颈

---

## 常见问题快速定位

| **现象** | **可能原因** | **排查Phase** | **快速验证** |
|---------|------------|--------------|-------------|
| 唤醒词无响应 | 1. 进程未启动<br>2. 麦克风故障<br>3. 配置错误 | Phase 6 | `ps aux \| grep wake` |
| API调用401/403 | 1. Token过期<br>2. API Key错误<br>3. 凭证未配置 | Phase 2 | 重新获取Token |
| ASR返回400 | 1. 音频格式错误<br>2. Token未加X-NLS-Token头<br>3. AppKey错误 | Phase 3 | 检查请求格式 |
| LLM无响应 | 1. API Key无效<br>2. 配额用尽<br>3. 网络超时 | Phase 4 | `curl -I https://dashscope.aliyuncs.com` |
| TTS无声音 | 1. Token失效<br>2. voice参数错误<br>3. 播放设备故障 | Phase 5 | `aplay /tmp/test_tts_output.wav` |
| 连续对话失败 | 1. 状态未重置<br>2. Token过期未刷新<br>3. 资源泄漏 | Phase 7 | 查看状态机日志 |
| 延迟过长 | 1. 网络慢<br>2. API服务繁忙<br>3. 音频文件太大 | Phase 8 | 测量各阶段耗时 |
| 网络错误 | 1. DNS问题<br>2. 防火墙阻断<br>3. 代理配置错误 | Phase 1 | `curl -I https://nls-gateway.cn-shanghai.aliyuncs.com` |

---

## 附录

### A. 关键端点URLs

```
# Token获取
https://nls-meta.cn-shanghai.aliyuncs.com/pop/2018-05-18/tokens

# ASR（实时语音识别）
https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/asr

# TTS（语音合成）
https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts

# 通义千问
https://dashscope.aliyuncs.com/api/v1/services/aigc/text-generation/generation
```

### B. 常用命令速查

```bash
# 网络测试
ping -c 3 nls-gateway.cn-shanghai.aliyuncs.com
curl -I https://dashscope.aliyuncs.com

# 音频测试
arecord -l
arecord -D hw:0,0 -f S16_LE -r 16000 -c 1 -d 3 test.wav
aplay test.wav

# 格式转换
ffmpeg -i input.wav -f s16le -ar 16000 -ac 1 output.pcm

# 进程检查
ps aux | grep python
ps aux | grep wake

# 日志查看
tail -f ~/xlerobot/logs/xlerobot.log
grep -i error ~/xlerobot/logs/xlerobot.log | tail -20
```

### C. 环境变量模板

```bash
# 添加到 ~/.bashrc 或 ~/.zshrc

# 阿里云凭证
export ALIYUN_AK_ID="你的AccessKey_ID"
export ALIYUN_AK_SECRET="你的AccessKey_Secret"
export ALIYUN_ASR_APPKEY="你的ASR_AppKey"
export ALIYUN_TTS_APPKEY="你的TTS_AppKey"

# 通义千问
export DASHSCOPE_API_KEY="你的API_KEY"

# 生效
source ~/.bashrc
```

### D. Python测试脚本模板

```python
#!/usr/bin/env python3
"""
完整链路测试脚本
使用方法: python3 test_pipeline.py
"""

import os
import requests
import time

# 配置
AK_ID = os.environ.get("ALIYUN_AK_ID")
AK_SECRET = os.environ.get("ALIYUN_AK_SECRET")
ASR_APPKEY = os.environ.get("ALIYUN_ASR_APPKEY")
LLM_API_KEY = os.environ.get("DASHSCOPE_API_KEY")

print("=" * 60)
print("完整链路测试")
print("=" * 60)

# 1. 获取Token
print("\n1. 获取Token...")
# [代码见Phase 2.2]

# 2. 录制音频
print("\n2. 录制音频...")
# [代码见Phase 3.1]

# 3. 调用ASR
print("\n3. 调用ASR...")
# [代码见Phase 3.3]

# 4. 调用LLM
print("\n4. 调用LLM...")
# [代码见Phase 4.1]

# 5. 调用TTS
print("\n5. 调用TTS...")
# [代码见Phase 5.1]

# 6. 播放音频
print("\n6. 播放音频...")
# [代码略]

print("\n" + "=" * 60)
print("测试完成！")
print("=" * 60)
```

---

## 总结

这个SOP专门针对**全在线服务架构**设计，重点关注：

1. **网络和凭证**：确保能访问阿里云服务并正确认证
2. **API调用**：单独验证ASR、LLM、TTS每个服务
3. **本地设备**：麦克风、唤醒词检测
4. **程序逻辑**：状态机、错误处理、日志
5. **端到端测试**：完整流程和性能验证

**使用建议**：
- 按Phase顺序执行，不要跳过
- 每个Phase都做记录（Pass/Fail）
- 遇到Fail立即停止，先解决再继续
- Phase 2的Token验证是关键，必须先通过

开始从Phase 1执行，把结果告诉我！🚀
