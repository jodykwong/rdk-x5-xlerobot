# XLeRobot系统数据流可视化图表

**文档版本**: 1.0
**创建日期**: 2025-11-16
**说明**: 本文档包含XLeRobot系统的所有关键流程图和架构图

---

## 📋 目录

1. [系统启动流程图](#1-系统启动流程图)
2. [ROS2节点通信架构图](#2-ros2节点通信架构图)
3. [完整语音交互数据流图](#3-完整语音交互数据流图)
4. [ASR状态机转换图](#4-asr状态机转换图)
5. [错误处理流程图](#5-错误处理流程图)

---

## 1. 系统启动流程图

### 1.1 完整启动时间线

```mermaid
gantt
    title XLeRobot系统启动时间线
    dateFormat ss
    axisFormat %Ss

    section 环境配置
    xlerobot_env.sh执行      :a1, 00, 2s
    清理conda路径            :a2, 00, 1s
    设置Python 3.10         :a3, 01, 1s
    加载ROS2环境            :a4, 01, 1s
    加载.env环境变量        :a5, 02, 1s

    section 环境检查
    显示启动横幅            :b1, 02, 1s
    硬件设备检查            :b2, 03, 3s
    运行环境检查            :b3, 06, 4s
    API服务连接测试         :b4, 10, 3s

    section ROS2启动
    Source ROS2工作空间     :c1, 13, 1s
    启动协调器节点          :c2, 14, 1s
    启动TTS节点(延迟1s)     :c3, 15, 1s
    启动LLM节点(延迟2s)     :c4, 16, 1s
    启动ASR节点(延迟3s)     :c5, 17, 1s

    section 系统就绪
    进入IDLE监听状态        :d1, 18, 1s
```

### 1.2 启动流程详细步骤

```mermaid
flowchart TD
    Start([开始执行启动脚本]) --> LoadEnv[执行source xlerobot_env.sh]

    LoadEnv --> CleanPath{检测conda路径?}
    CleanPath -->|存在| RemovePath[移除conda路径]
    CleanPath -->|不存在| SetPython[设置Python 3.10环境]
    RemovePath --> SetPython

    SetPython --> LoadROS[加载ROS2 Humble环境]
    LoadROS --> SetPaths[设置PYTHONPATH和项目路径]
    SetPaths --> LoadDotEnv[加载.env环境变量]

    LoadDotEnv --> Banner[显示启动横幅]
    Banner --> CheckHardware[硬件设备检查]

    CheckHardware --> CheckMic{麦克风可用?}
    CheckMic -->|否| WarnMic[警告: 麦克风不可用]
    CheckMic -->|是| CheckSpeaker{扬声器可用?}
    WarnMic --> CheckSpeaker

    CheckSpeaker -->|否| WarnSpeaker[警告: 扬声器不可用]
    CheckSpeaker -->|是| CheckPython{Python环境正确?}
    WarnSpeaker --> CheckPython

    CheckPython -->|否| ErrorPython[错误: Python环境问题]
    CheckPython -->|是| CheckROS{ROS2环境正确?}
    ErrorPython --> Exit1([终止启动])

    CheckROS -->|否| ErrorROS[错误: ROS2环境问题]
    CheckROS -->|是| CheckAPI{API连接正常?}
    ErrorROS --> Exit2([终止启动])

    CheckAPI -->|否| ErrorAPI[错误: API连接失败]
    CheckAPI -->|是| CheckPassed[✅ 所有检查通过]
    ErrorAPI --> Exit3([终止启动])

    CheckPassed --> SourceWorkspace[Source ROS2工作空间]
    SourceWorkspace --> LaunchFile[执行ros2 launch]

    LaunchFile --> Node1[启动coordinator节点<br/>延迟0秒]
    Node1 --> Node2[启动tts_service节点<br/>延迟1秒]
    Node2 --> Node3[启动llm_service节点<br/>延迟2秒]
    Node3 --> Node4[启动asr_bridge节点<br/>延迟3秒]

    Node4 --> WaitReady[等待所有节点就绪]
    WaitReady --> SystemReady[✅ 系统就绪]
    SystemReady --> EnterIdle[进入IDLE监听状态]
    EnterIdle --> End([开始监听唤醒词])

    style Start fill:#90EE90
    style End fill:#90EE90
    style CheckPassed fill:#90EE90
    style SystemReady fill:#90EE90
    style Exit1 fill:#FFB6C1
    style Exit2 fill:#FFB6C1
    style Exit3 fill:#FFB6C1
    style ErrorPython fill:#FFB6C1
    style ErrorROS fill:#FFB6C1
    style ErrorAPI fill:#FFB6C1
```

---

## 2. ROS2节点通信架构图

### 2.1 节点拓扑和话题连接

```mermaid
graph TB
    subgraph 硬件层
        Mic[麦克风<br/>USB Audio Device]
        Speaker[扬声器<br/>USB Audio Device]
        Camera[摄像头<br/>可选]
    end

    subgraph ROS2节点网络
        ASR[asr_bridge_node<br/>ASR桥接节点]
        LLM[llm_service_node<br/>LLM服务节点]
        TTS[tts_service_node<br/>TTS服务节点]
        Coord[voice_assistant_coordinator<br/>语音助手协调器]
    end

    subgraph 云端服务
        AliyunASR[阿里云ASR<br/>Paraformer粤语]
        QwenLLM[通义千问<br/>qwen3-vl-plus]
        AliyunTTS[阿里云TTS<br/>jiajia音色]
    end

    Mic -->|音频流| ASR
    ASR -->|HTTP| AliyunASR
    ASR -->|/voice_command<br/>std_msgs/String| LLM
    ASR -->|/asr_status<br/>std_msgs/String| Coord

    Camera -.->|可选<br/>/vision_input<br/>sensor_msgs/Image| LLM
    LLM -->|HTTP| QwenLLM
    LLM -->|/llm_response<br/>std_msgs/String| TTS
    LLM -->|/llm_status<br/>std_msgs/String| Coord

    TTS -->|HTTP| AliyunTTS
    TTS -->|/tts_status<br/>std_msgs/String| Coord
    TTS -->|音频流| Speaker

    Coord -->|/system_status<br/>std_msgs/String| ASR
    Coord -->|/system_status<br/>std_msgs/String| LLM
    Coord -->|/system_status<br/>std_msgs/String| TTS

    style Mic fill:#E6F3FF
    style Speaker fill:#E6F3FF
    style Camera fill:#E6F3FF
    style ASR fill:#FFE6E6
    style LLM fill:#FFE6E6
    style TTS fill:#FFE6E6
    style Coord fill:#FFE6E6
    style AliyunASR fill:#FFF9E6
    style QwenLLM fill:#FFF9E6
    style AliyunTTS fill:#FFF9E6
```

### 2.2 话题详细规范

```mermaid
graph LR
    subgraph 话题列表
        direction TB
        T1[/voice_command<br/>std_msgs/String<br/>ASR识别的文本命令]
        T2[/llm_response<br/>std_msgs/String<br/>LLM生成的回复文本]
        T3[/asr_status<br/>std_msgs/String JSON<br/>ASR节点状态信息]
        T4[/llm_status<br/>std_msgs/String JSON<br/>LLM节点状态信息]
        T5[/tts_status<br/>std_msgs/String JSON<br/>TTS节点状态信息]
        T6[/system_status<br/>std_msgs/String JSON<br/>整体系统状态]
        T7[/wake_word_detected<br/>std_msgs/Bool<br/>唤醒词检测事件]
        T8[/vision_input<br/>sensor_msgs/Image<br/>摄像头图像可选]
    end

    style T1 fill:#E6F7FF
    style T2 fill:#E6F7FF
    style T3 fill:#FFF7E6
    style T4 fill:#FFF7E6
    style T5 fill:#FFF7E6
    style T6 fill:#F0F0F0
    style T7 fill:#E6FFE6
    style T8 fill:#FFE6F0
```

---

## 3. 完整语音交互数据流图

### 3.1 "傻强，今日天气点样？" 完整流程

```mermaid
sequenceDiagram
    autonumber
    actor User as 用户
    participant Mic as 麦克风
    participant ASR as ASR节点
    participant AliyunASR as 阿里云ASR
    participant LLM as LLM节点
    participant QwenLLM as 通义千问
    participant TTS as TTS节点
    participant AliyunTTS as 阿里云TTS
    participant Speaker as 扬声器

    Note over ASR: 状态: IDLE<br/>等待唤醒词

    User->>Mic: 说 "傻强"
    Mic->>ASR: 音频流 (44.1kHz PCM)
    ASR->>ASR: 音频捕获 (3秒)
    ASR->>ASR: WAV格式转换
    ASR->>ASR: 重采样 44.1→16kHz
    ASR->>ASR: Base64编码

    Note over ASR: 状态: IDLE→检测中

    ASR->>AliyunASR: POST /asr<br/>{"audio": "base64...", "language": "cn-cantonese"}
    AliyunASR-->>ASR: {"result": {"text": "傻强", "confidence": 0.98}}

    ASR->>ASR: 唤醒词匹配: True

    Note over ASR: 状态: IDLE→WAKE_DETECTED

    ASR->>TTS: 请求合成 "傻强系度,老细有乜可以帮到你!"
    TTS->>AliyunTTS: POST /tts<br/>{"text": "傻强系度...", "voice": "jiajia"}
    AliyunTTS-->>TTS: WAV音频数据 (16kHz, 45KB)
    TTS->>Speaker: 播放音频
    Speaker->>User: 听到 "傻强系度,老细有乜可以帮到你!"

    Note over ASR: 状态: WAKE_DETECTED→LISTENING_COMMAND

    ASR->>ASR: 重新监听用户指令 (超时5秒)

    User->>Mic: 说 "今日天气点样"
    Mic->>ASR: 音频流 (44.1kHz PCM)
    ASR->>ASR: 音频捕获 (3秒)
    ASR->>ASR: WAV格式转换
    ASR->>ASR: 重采样 44.1→16kHz
    ASR->>ASR: Base64编码

    Note over ASR: 状态: LISTENING_COMMAND→PROCESSING

    ASR->>AliyunASR: POST /asr<br/>{"audio": "base64...", "language": "cn-cantonese"}
    AliyunASR-->>ASR: {"result": {"text": "今日天气点样", "confidence": 0.95}}

    ASR->>LLM: 发布 /voice_command<br/>data: "今日天气点样"

    LLM->>LLM: 添加到对话历史
    LLM->>QwenLLM: POST /generation<br/>{"messages": [{"role": "user", "content": "今日天气点样"}]}
    QwenLLM-->>LLM: {"output": {"text": "今日天气晴朗，气温大约25度..."}}

    LLM->>TTS: 发布 /llm_response<br/>data: "今日天气晴朗..."

    Note over ASR: 状态: PROCESSING→RESPONDING

    TTS->>AliyunTTS: POST /tts<br/>{"text": "今日天气晴朗...", "voice": "jiajia"}
    AliyunTTS-->>TTS: WAV音频数据 (16kHz, 135KB)
    TTS->>Speaker: 播放音频
    Speaker->>User: 听到 "今日天气晴朗，气温大约25度，适合外出活动..."

    Note over ASR: 状态: RESPONDING→IDLE

    ASR->>ASR: 返回空闲监听模式

    Note over ASR: 状态: IDLE<br/>等待下一次唤醒
```

### 3.2 数据格式变换流程

```mermaid
graph LR
    subgraph 阶段1_音频采集
        A1[用户语音] -->|麦克风| A2[模拟音频]
        A2 -->|ALSA驱动| A3[PCM数字音频<br/>44.1kHz, 16-bit]
        A3 -->|speech_recognition| A4[sr.AudioData<br/>numpy array]
    end

    subgraph 阶段2_音频预处理
        A4 --> B1[get_wav_data]
        B1 --> B2[WAV bytes<br/>包含44字节头部<br/>132,344 bytes]
        B2 --> B3[检测采样率<br/>44100Hz]
        B3 --> B4[重采样<br/>44.1→16kHz]
        B4 --> B5[PCM bytes<br/>47,872 bytes<br/>减少64%]
        B5 --> B6[Base64编码<br/>63,829 chars<br/>增加33%]
    end

    subgraph 阶段3_ASR识别
        B6 --> C1[构建API请求<br/>JSON payload]
        C1 --> C2[HTTP POST<br/>阿里云ASR]
        C2 --> C3[等待响应<br/>~2.3秒]
        C3 --> C4[JSON响应<br/>status: 200000]
        C4 --> C5[提取文本<br/>今日天气点样]
    end

    subgraph 阶段4_LLM处理
        C5 --> D1[对话历史<br/>JSON array]
        D1 --> D2[构建API请求<br/>messages]
        D2 --> D3[HTTP POST<br/>通义千问]
        D3 --> D4[等待响应<br/>~2.8秒]
        D4 --> D5[提取回复<br/>今日天气晴朗...]
    end

    subgraph 阶段5_TTS合成
        D5 --> E1[构建TTS请求<br/>text+voice]
        E1 --> E2[HTTP POST<br/>阿里云TTS]
        E2 --> E3[等待响应<br/>~1.2秒]
        E3 --> E4[WAV音频<br/>135,234 bytes]
        E4 --> E5[临时文件<br/>/tmp/xxx.wav]
        E5 --> E6[pygame播放<br/>~7.5秒]
        E6 --> E7[扬声器输出<br/>用户听到回复]
    end

    style A1 fill:#E6F3FF
    style A7 fill:#E6F3FF
    style B5 fill:#FFE6E6
    style C5 fill:#E6FFE6
    style D5 fill:#FFF9E6
    style E7 fill:#F0E6FF
```

---

## 4. ASR状态机转换图

### 4.1 ASRState状态机

```mermaid
stateDiagram-v2
    [*] --> IDLE: 系统启动

    IDLE --> WAKE_DETECTED: 检测到唤醒词<br/>("傻强")
    IDLE --> IDLE: 持续监听音频<br/>(无唤醒词)

    WAKE_DETECTED --> LISTENING_COMMAND: 播放欢迎语完成<br/>(2秒延迟)

    LISTENING_COMMAND --> PROCESSING: 捕获到用户指令<br/>(5秒内)
    LISTENING_COMMAND --> IDLE: 超时未检测到指令<br/>(5秒后)

    PROCESSING --> RESPONDING: ASR识别成功<br/>且LLM生成回复
    PROCESSING --> IDLE: ASR识别失败<br/>或LLM处理失败

    RESPONDING --> IDLE: TTS播放完成

    note right of IDLE
        动作:
        - 持续监听音频
        - 检查唤醒词
        - 每20次打印状态
    end note

    note right of WAKE_DETECTED
        动作:
        - 播放欢迎语
        - 等待2秒
    end note

    note right of LISTENING_COMMAND
        动作:
        - 重新监听
        - 超时5秒
        - 最多10秒指令
    end note

    note right of PROCESSING
        动作:
        - ASR识别
        - LLM处理
        - 更新对话历史
    end note

    note right of RESPONDING
        动作:
        - TTS合成
        - 播放音频
    end note
```

### 4.2 状态转换时间线

```mermaid
gantt
    title ASR状态转换时间线 (单次完整交互)
    dateFormat ss
    axisFormat %Ss

    section 状态转换
    IDLE (监听唤醒词)      :a1, 00, 2s
    WAKE_DETECTED (播放欢迎) :a2, 02, 2s
    LISTENING_COMMAND (等待指令) :a3, 04, 5s
    PROCESSING (ASR+LLM)   :a4, 09, 6s
    RESPONDING (TTS播放)   :a5, 15, 8s
    IDLE (返回监听)        :a6, 23, 1s

    section 关键事件
    检测到"傻强"           :milestone, m1, 02, 0s
    播放欢迎语完成         :milestone, m2, 04, 0s
    捕获用户指令           :milestone, m3, 09, 0s
    LLM响应完成           :milestone, m4, 15, 0s
    TTS播放完成           :milestone, m5, 23, 0s
```

---

## 5. 错误处理流程图

### 5.1 ASR错误处理流程

```mermaid
flowchart TD
    Start([ASR请求开始]) --> Send[发送HTTP请求到阿里云]

    Send --> Wait{等待响应}
    Wait -->|成功| CheckStatus{检查status}
    Wait -->|超时8秒| Timeout[记录超时错误]
    Wait -->|网络错误| NetError[记录网络错误]

    CheckStatus -->|200000| Success[✅ 识别成功]
    CheckStatus -->|401| TokenError[Token错误]
    CheckStatus -->|400| ParamError[参数错误]
    CheckStatus -->|429| RateLimit[限流错误]
    CheckStatus -->|500| ServerError[服务器错误]

    Success --> Return1[返回识别文本]

    TokenError --> RefreshToken{重新获取Token}
    RefreshToken -->|成功| Retry1{重试次数<4?}
    RefreshToken -->|失败| FinalFail1[记录严重错误]

    ParamError --> CheckAudio{检查音频格式}
    CheckAudio -->|格式错误| FixAudio[重新采样音频]
    CheckAudio -->|格式正确| Retry2{重试次数<4?}
    FixAudio --> Retry2

    RateLimit --> ExponentialBackoff[指数退避等待]
    ExponentialBackoff --> Retry3{重试次数<4?}

    ServerError --> Retry4{重试次数<4?}

    Timeout --> Retry5{重试次数<4?}
    NetError --> Retry6{重试次数<4?}

    Retry1 -->|是| Send
    Retry1 -->|否| FinalFail2[所有重试失败]
    Retry2 -->|是| Send
    Retry2 -->|否| FinalFail2
    Retry3 -->|是| Send
    Retry3 -->|否| FinalFail2
    Retry4 -->|是| Send
    Retry4 -->|否| FinalFail2
    Retry5 -->|是| Send
    Retry5 -->|否| FinalFail2
    Retry6 -->|是| Send
    Retry6 -->|否| FinalFail2

    FinalFail1 --> PlayError[播放错误提示音]
    FinalFail2 --> PlayError
    PlayError --> Return2[返回None]

    Return1 --> End([结束])
    Return2 --> End

    style Start fill:#90EE90
    style End fill:#90EE90
    style Success fill:#90EE90
    style Return1 fill:#90EE90
    style FinalFail1 fill:#FFB6C1
    style FinalFail2 fill:#FFB6C1
    style PlayError fill:#FFB6C1
    style Return2 fill:#FFB6C1
```

### 5.2 TTS降级链流程

```mermaid
flowchart TD
    Start([TTS合成请求]) --> TryPrimary[尝试主TTS服务<br/>阿里云TTS + jiajia音色]

    TryPrimary --> CheckPrimary{合成成功?}
    CheckPrimary -->|是| PlayPrimary[使用pygame播放]

    PlayPrimary --> CheckPlay1{播放成功?}
    CheckPlay1 -->|是| Success[✅ 播放成功]
    CheckPlay1 -->|否| TryAplay[尝试aplay播放]

    CheckPrimary -->|否| TryBackupVoice[尝试备用音色<br/>xiaoyun]

    TryBackupVoice --> CheckBackup{合成成功?}
    CheckBackup -->|是| PlayBackup[使用pygame播放]
    CheckBackup -->|否| TryPrerecorded[播放预录提示音]

    PlayBackup --> CheckPlay2{播放成功?}
    CheckPlay2 -->|是| Success
    CheckPlay2 -->|否| TryAplay

    TryAplay --> CheckAplay{播放成功?}
    CheckAplay -->|是| Success
    CheckAplay -->|否| TryPrerecorded

    TryPrerecorded --> CheckPrerecorded{提示音存在?}
    CheckPrerecorded -->|是| PlayBeep[播放beep.wav]
    CheckPrerecorded -->|否| SilentMode[静默模式]

    PlayBeep --> CheckBeep{播放成功?}
    CheckBeep -->|是| PartialSuccess[⚠️ 降级成功<br/>使用提示音]
    CheckBeep -->|否| SilentMode

    SilentMode --> LogOnly[仅记录日志]
    LogOnly --> Fail[❌ 完全失败]

    Success --> End([结束])
    PartialSuccess --> End
    Fail --> End

    style Start fill:#90EE90
    style End fill:#90EE90
    style Success fill:#90EE90
    style PartialSuccess fill:#FFF9E6
    style Fail fill:#FFB6C1
    style SilentMode fill:#FFB6C1
```

### 5.3 系统级错误恢复流程

```mermaid
flowchart TD
    Error([检测到错误]) --> ClassifyError{错误类型分类}

    ClassifyError -->|硬件错误| HardwareError[硬件错误处理]
    ClassifyError -->|网络错误| NetworkError[网络错误处理]
    ClassifyError -->|API错误| APIError[API错误处理]
    ClassifyError -->|业务错误| BusinessError[业务错误处理]

    HardwareError --> CheckMic{麦克风问题?}
    CheckMic -->|是| TryReInitMic[重新初始化麦克风]
    CheckMic -->|否| CheckSpeaker{扬声器问题?}
    TryReInitMic --> RecoverMic{恢复成功?}
    RecoverMic -->|是| Continue1[✅ 继续运行]
    RecoverMic -->|否| WarnUser1[⚠️ 警告用户<br/>进入只读模式]

    CheckSpeaker -->|是| TryReInitSpeaker[重新初始化扬声器]
    CheckSpeaker -->|否| OtherHardware[其他硬件问题]
    TryReInitSpeaker --> RecoverSpeaker{恢复成功?}
    RecoverSpeaker -->|是| Continue1
    RecoverSpeaker -->|否| WarnUser2[⚠️ 警告用户<br/>进入静默模式]

    NetworkError --> Retry{重试策略}
    Retry --> Wait[指数退避等待]
    Wait --> CheckNet{网络恢复?}
    CheckNet -->|是| Continue2[✅ 继续运行]
    CheckNet -->|否| MaxRetry{达到最大重试?}
    MaxRetry -->|否| Retry
    MaxRetry -->|是| OfflineMode[进入离线模式]

    APIError --> CheckToken{Token问题?}
    CheckToken -->|是| RefreshToken[重新获取Token]
    CheckToken -->|否| CheckQuota{配额问题?}
    RefreshToken --> RecoverToken{获取成功?}
    RecoverToken -->|是| Continue3[✅ 继续运行]
    RecoverToken -->|否| CriticalError[❌ 严重错误<br/>需要人工介入]

    CheckQuota -->|是| WaitQuota[等待配额重置]
    CheckQuota -->|否| OtherAPIError[其他API错误]
    WaitQuota --> Continue4[✅ 继续运行]

    BusinessError --> LogError[记录错误日志]
    LogError --> NotifyUser[通知用户]
    NotifyUser --> Continue5[✅ 继续运行]

    Continue1 --> End([结束])
    Continue2 --> End
    Continue3 --> End
    Continue4 --> End
    Continue5 --> End
    WarnUser1 --> End
    WarnUser2 --> End
    OfflineMode --> End
    CriticalError --> End

    style Error fill:#FFB6C1
    style Continue1 fill:#90EE90
    style Continue2 fill:#90EE90
    style Continue3 fill:#90EE90
    style Continue4 fill:#90EE90
    style Continue5 fill:#90EE90
    style WarnUser1 fill:#FFF9E6
    style WarnUser2 fill:#FFF9E6
    style OfflineMode fill:#FFF9E6
    style CriticalError fill:#FFB6C1
    style End fill:#90EE90
```

---

## 6. 性能分析图表

### 6.1 延迟分析饼图（文字版）

```
总延迟: 14.5秒 (从说话到听到回复)

┌─────────────────────────────────────┐
│  音频采集: 3.0秒 (21%)              │ ████
│  ASR处理: 3.1秒 (21%)               │ ████
│  LLM处理: 2.8秒 (19%)               │ ███
│  TTS处理: 1.2秒 (8%)                │ █
│  音频播放: 7.5秒 (52%)              │ ██████████
└─────────────────────────────────────┘

瓶颈分析:
⚠️ ASR API调用: 2.3秒 (主要瓶颈)
⚠️ LLM API调用: 2.8秒 (主要瓶颈)
ℹ️  音频播放: 7.5秒 (受回复长度影响，不可压缩)
```

### 6.2 并发性能对比

```mermaid
gantt
    title 串行 vs 并发处理对比
    dateFormat ss
    axisFormat %Ss

    section 串行处理 (当前)
    唤醒词检测       :a1, 00, 2s
    播放欢迎语       :a2, 02, 2s
    捕获用户指令     :a3, 04, 3s
    ASR识别         :a4, 07, 3s
    LLM处理         :a5, 10, 3s
    TTS合成         :a6, 13, 1s
    播放回复         :a7, 14, 8s
    总延迟22秒       :milestone, m1, 22, 0s

    section 并发处理 (优化后)
    唤醒词检测       :b1, 00, 2s
    播放欢迎语       :b2, 02, 2s
    捕获用户指令     :b3, 04, 3s
    ASR识别         :b4, 07, 2s
    LLM处理 (流式)   :b5, 09, 2s
    TTS合成 (并发)   :b6, 11, 1s
    播放回复         :b7, 12, 8s
    总延迟20秒       :milestone, m2, 20, 0s
```

---

## 附录

### A. Mermaid语法说明

本文档使用Mermaid创建流程图，支持以下渲染工具：
- GitHub (原生支持)
- VS Code (需要Mermaid插件)
- Typora
- Obsidian
- 在线工具: https://mermaid.live/

### B. 图表更新记录

| 版本 | 日期 | 更新内容 |
|------|------|---------|
| 1.0 | 2025-11-16 | 初始版本，包含5个核心流程图 |

### C. 相关文档

- [系统架构与数据流完全指南](xlerobot-system-architecture-dataflow.md)
- [快速参考卡片](xlerobot-quick-reference.md)

---

**文档维护**: 本文档应随系统架构变化而更新
**最后更新**: 2025-11-16
