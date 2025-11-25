# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## ⛔ 全局强制规则 - 严禁Mock数据 ⛔

> **本项目所有开发工作必须严格遵守以下规则，无任何例外**

### 🚫 严禁使用
- ❌ **Mock数据** - 禁止使用任何模拟数据
- ❌ **模拟服务** - 禁止模拟API响应或服务
- ❌ **硬编码数据** - 禁止硬编码测试数据
- ❌ **测试脚本验证** - 禁止编写新的测试脚本来验证功能

### ✅ 必须使用
- ✅ **真实麦克风输入** - 必须使用实际音频设备采集
- ✅ **真实ASR/LLM/TTS算法** - 必须调用阿里云真实API
- ✅ **真实扬声器输出** - 必须通过实际音频设备播放
- ✅ **预编开发环境** - 使用已配置好的开发环境
- ✅ **已有功能代码** - 直接使用已开发好的功能进行验证

### 📋 文件标识要求
所有新建或修改的核心代码文件头部必须包含：
```python
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
```

### 🔧 验证方式
- 直接运行 `./start_voice_assistant.sh` 启动完整系统
- 使用真实唤醒词"傻强"进行语音交互测试
- 观察真实的ASR识别、LLM响应、TTS播放结果

### ⚠️ 违反规则的后果
任何使用Mock、模拟或硬编码数据的代码将被视为无效实现，必须使用真实数据重新实现。

---

## 🔧 系统性排查SOP - 避免重复排查

> **Claude Code在排查问题前必须先查看此SOP，避免重复无用的排查和修改**

### 一、已确认修复的问题（⛔ 禁止再次排查）

| 问题 | 文件位置 | 状态 | 修复说明 |
|------|----------|------|----------|
| 音频分块>64KB错误 | `websocket_asr_service.py:329-339` | ✅ 已修复 | 32KB分块发送 |
| TTS asyncio错误 | `tts_service_node.py` | ✅ 已修复 | 使用ThreadPoolExecutor |
| NumPy布尔值歧义 | `asr_system.py` | ✅ 已修复 | 使用`len()`检查 |
| 循环引用错误 | `simple_alsa_recorder.py:357` | ✅ 已修复 | 移除ThreadSafeAudioRecorder别名 |
| get_completion_event缺失 | `audio_recorder_manager.py:309` | ✅ 已存在 | 方法已实现，无需修改 |
| 摄像头检查卡死 | `start_voice_assistant.sh:199-210,232,714-722,751-784,782` | ✅ 已修复 | 增强超时机制：lsmod(1s)+v4l2-ctl(2s)+权限检查(1s)+命令可用性检查+错误处理优化 |
| 静音产生ASR幻觉 | `asr_system.py:666`, `asr_system.py:635` | ✅ 已修复 | 能量阈值从1000调整到400（正常语音300-800，静音<100） |
| ASR置信度检查错误 | `websocket_asr_service.py:175-184` | ✅ 已修复 | 移除置信度检查(API不返回) |
| 协调器节点离线误报 | `voice_assistant_coordinator.py:108-114,200-213,338-346` | ✅ 已修复 | NodeStatus初始化为当前时间+节点名称匹配+15秒启动宽限期 |
| Asyncio事件循环不存在 | `voice_assistant_coordinator.py:241-253`, `llm_service_node.py:157-161,181-196` | ✅ 已修复 | 使用线程和ThreadPoolExecutor替代asyncio.create_task |
| LLM节点硬要求API密钥 | `llm_service_node.py:385-405` | ✅ 已修复 | 改为警告模式，支持无API密钥启动 |
| Launch文件硬编码路径 | `voice_assistant.launch.py:33-40,95,110,130,172` | ✅ 已修复 | 使用XLEROBOT_ROOT环境变量+路径自动推断 |
| TTS节点asyncio.Queue初始化 | `tts_service_node.py:95-96,257,318-327` | ✅ 已修复 | 使用queue.Queue替代asyncio.Queue |
| TTS引擎API参数错误 | `aliyun_tts_websocket_engine.py:189-197`, `aliyun_tts_websocket_client.py:230-237` | ✅ 已修复 | `on_start`→`on_metainfo`, `on_audio_data`→`on_data`, 添加`url`参数 |
| ASR监听循环NumPy判断错误 | `asr_system.py:429,673` | ✅ 已修复 | `len(audio_data)`→`audio_data.size` 避免布尔值歧义 |
| ASR监听循环异常处理不完善 | `asr_system.py:433-516` | ✅ 已修复 | 为唤醒词检测添加完整异常保护 |
| RDK X5摄像头检查误导性输出 | `start_voice_assistant.sh:191-217,769-787` | ✅ 已修复 | 移除USB驱动检查，添加平台说明，删除重复代码 |
| ASR死循环根本原因 | `asr_system.py:91-92,717-720,1017-1113,698,756` | ✅ 已修复 | TTS播放期间禁用麦克风+回声检测+循环检测+统一能量阈值600 |
| TTS回调函数命名错误 | `aliyun_tts_websocket.py:417,420,456-457,507,510,553-554` | ✅ 已修复 | `on_start`→`on_metainfo`, `on_audio`→`on_data` (阿里云NLS SDK规范) |
| TTS文本长度超限 | `aliyun_tts_websocket.py:65-124,314-361,381-430` | ✅ 已修复 | 添加300字符限制检查和智能分段合成 |
| macOS摄像头检查卡死 | `start_voice_assistant.sh:193-268` | ✅ 已修复 | 平台检测(uname -s)，macOS跳过V4L2检查，避免compgen阻塞 |
| ROS2编译平台不兼容 | `start_voice_assistant.sh:1320-1342` | ✅ 已修复 | macOS跳过xlerobot_camera/vision包，避免setuptools --editable错误 |
| 环境变量传递失败 | `voice_assistant.launch.py:66-94`, `start_voice_assistant.sh:1473-1486` | ✅ 已修复 | launch过滤空值，启动脚本使用env命令显式传递 |

### 二、待修复的核心问题（按优先级排序）

**当前无已知待修复问题** ✅

### 三、标准排查流程（必须按顺序执行）

**第一步**: 检查日志确认错误类型
```bash
./start_voice_assistant.sh logs | grep -E "错误|Error|❌|QWEN_API_KEY"
```

**第二步**: 验证环境变量是否正确设置
```bash
source ./xlerobot_env.sh
echo "QWEN_API_KEY: ${QWEN_API_KEY:0:10}..."
echo "ALIBABA_CLOUD_ACCESS_KEY_ID: ${ALIBABA_CLOUD_ACCESS_KEY_ID:0:10}..."
echo "ALIYUN_NLS_APPKEY: $ALIYUN_NLS_APPKEY"
```

**第三步**: 按错误类型定位问题
| 错误日志关键词 | 问题类型 | 检查文件 |
|---------------|----------|----------|
| `QWEN_API_KEY环境变量未设置` | 环境变量传递 | `start_voice_assistant.sh`, `voice_assistant.launch.py` |
| `no running event loop` | asyncio错误 | 节点`__init__`方法 |
| `录音器等待超时` | 音频设备 | `audio_recorder_manager.py`, ALSA配置 |
| `ASR识别超时` | WebSocket连接 | `websocket_asr_service.py`, 网络配置 |
| `Token获取失败` | API密钥 | `.env`文件, 阿里云配置 |
| `unexpected keyword argument 'on_start'` | TTS引擎API参数 | `aliyun_tts_websocket_engine.py`, `aliyun_tts_websocket_client.py` |
| `The truth value of an array` | NumPy数组判断 | `asr_system.py` 数组长度检查 |
| `唤醒词检测异常` | 监听循环异常 | `asr_system.py` 异常处理机制 |

**第四步**: 定位到具体文件后，只修改必要代码

### 四、修复验证清单

每次修复后必须验证以下项目（直接运行系统，不编写测试脚本）:

- [ ] 1. 环境变量在ROS2子进程中可见: `ros2 run xlerobot llm_service_node` 无QWEN_API_KEY错误
- [ ] 2. 四个ROS2节点全部启动成功: `ros2 node list` 显示所有节点
- [ ] 3. 唤醒词"傻强"能被检测并响应
- [ ] 4. ASR→LLM→TTS链路完整工作

### 五、⛔ 禁止的操作

- ❌ **不要重复修复已确认的问题** - 参见"一、已确认修复的问题"
- ❌ **不要编写新测试脚本验证** - 直接运行`./start_voice_assistant.sh`
- ❌ **不要使用Mock/模拟数据** - 必须使用真实硬件和API
- ❌ **不要猜测性修改** - 必须先通过日志确认问题
- ❌ **不要修改已工作的代码** - 只修改确认有问题的部分

### 六、问题升级机制

如果按照SOP排查后仍无法解决:
1. 记录完整的错误日志
2. 记录已尝试的排查步骤
3. 明确标注"需要用户协助"的具体问题
4. 请用户提供更多上下文信息

---

## 项目概述

**XLeRobot** 是一个基于ROS2的智能粤语语音机器人系统,运行在D-Robotics RDK X5硬件平台上。项目采用Brownfield Level 4企业级标准,实现了完整的语音交互功能,包括语音识别(ASR)、自然语言理解(LLM)和语音合成(TTS)。

### 核心特性
- 粤语语音识别与合成 (阿里云ASR/TTS服务)
- 智能对话理解 (通义千问大语言模型)
- 唤醒词检测 ("傻强")
- 多模态集成 (语音+视觉,Qwen-VL)
- ROS2分布式架构
- 硬件加速 (RDK X5 NPU/BPU)

### 三阶段架构演进
1. **第一阶段 (当前)**: 全在线服务 - 阿里云ASR + 通义千问API + 阿里云TTS
2. **第二阶段 (规划中)**: 离线服务 - TROS本地算法包
3. **第三阶段 (规划中)**: 完整机器人集成 - 深度硬件优化

---

## 关键环境要求 ⚠️

### Python版本 (严格要求)
```bash
# ✅ 必须使用系统Python 3.10.12
python3.10 --version  # Python 3.10.12

# ❌ 禁止使用
python3.13  # Miniconda版本 - 与ROS2不兼容
```

### ROS2环境 (必需)
```bash
# 激活ROS2 Humble环境
source /opt/ros/humble/setup.bash

# 验证ROS2环境
echo $ROS_DISTRO  # 应输出: humble
ros2 --version    # 应显示ROS2版本信息
```

### 项目路径
- **开发环境** (macOS): `/Users/jodykwong/Documents/RDK-X5-Projects/xlerobot`
- **生产环境** (RDK X5): `/home/sunrise/xlerobot`

### 关键环境变量
```bash
export PYTHONPATH="/home/sunrise/xlerobot/src:$PYTHONPATH"
export ROS_DOMAIN_ID=42
export ALIBABA_CLOUD_ACCESS_KEY_ID="your_access_key_id"
export ALIBABA_CLOUD_ACCESS_KEY_SECRET="your_access_key_secret"
export ALIYUN_NLS_APPKEY="your_nls_appkey"
export QWEN_API_KEY="your_qwen_api_key"
```

---

## 🛡️ Miniconda冲突解决指南

### 问题描述
XLeRobot项目严格要求使用系统Python 3.10，但系统安装的Miniconda会：
1. 自动激活conda环境，导致Python版本冲突
2. 修改PATH路径，使conda Python优先
3. 与ROS2 Python包产生兼容性问题

### 解决方案C：XLeRobot专用环境脚本

#### 第一步：使用专用环境脚本
```bash
# ✅ 加载XLeRobot环境 (推荐)
source ./xlerobot_env.sh

# ✅ 验证环境配置
./verify_xlerobot_environment.sh

# ✅ 现在可以安全运行所有脚本
./start_voice_assistant.sh
```

#### 第二步：关键脚本自动环境管理
所有主要启动脚本现在都会自动加载正确环境：
- `start_voice_assistant.sh` - 主启动脚本
- `start_test_mode.sh` - 测试模式
- `run_epic1_tests.sh` - 测试套件
- `start_development.sh` - 开发环境
- 其他所有启动脚本

#### 第三步：环境验证与诊断
```bash
# 运行环境检查
./verify_xlerobot_environment.sh

# 详细的系统环境检查
./start_voice_assistant.sh check

# 快速验证
echo $PYTHON_EXECUTABLE     # 应显示: /usr/bin/python3.10
which python3              # 应指向系统python3
echo $PYTHONPATH           # 应包含项目路径
```

### 紧急修复命令
如果遇到conda环境冲突：

```bash
# 1. 立即加载正确环境
source ./xlerobot_env.sh

# 2. 验证修复效果
python3.10 --version
which python3

# 3. 如果仍有问题，重新启动shell
exec bash
source ./xlerobot_env.sh
```

### 环境状态检查表
- ✅ `python3.10 --version` → `Python 3.10.12`
- ✅ `which python3` → `/usr/bin/python3`
- ✅ `echo $PYTHON_EXECUTABLE` → `/usr/bin/python3.10`
- ✅ `echo $PATH` → 不包含conda/miniconda路径
- ✅ `echo $CONDA_DEFAULT_ENV` → 空或未设置
- ✅ `./verify_xlerobot_environment.sh` → 无错误

### 已废弃的脚本
以下脚本已被`xlerobot_env.sh`替代，不再直接使用：
- `setup_xlerobot_env.sh` → 已废弃
- `fix_python_env.sh` → 已废弃

### 故障排查
| 问题 | 原因 | 解决方案 |
|------|------|----------|
| `python3`指向conda | conda环境活跃 | `conda deactivate` + `source ./xlerobot_env.sh` |
| PATH包含conda | 自动conda初始化 | `source ./xlerobot_env.sh` |
| 导入错误 | Python版本不对 | 使用`$PYTHON_EXECUTABLE`运行脚本 |
| 环境检查失败 | conda干扰 | 重新启动shell并加载环境脚本 |

---

## 常用命令

### 服务启动与管理
```bash
# 启动语音助手服务 (包含完整环境检查)
./start_voice_assistant.sh

# 强制启动 (跳过环境检查)
./start_voice_assistant.sh --force

# 查看服务状态
./start_voice_assistant.sh status

# 停止服务
./start_voice_assistant.sh stop

# 重启服务
./start_voice_assistant.sh restart

# 查看日志
./start_voice_assistant.sh logs

# 环境检查
./start_voice_assistant.sh check
```

### 测试命令
```bash
# Epic 1完整集成测试
$PYTHON_EXECUTABLE tests/test_epic1_complete_integration.py

# Epic 1功能验证
$PYTHON_EXECUTABLE tests/verify_epic1_complete_functionality.py

# 运行特定测试
$PYTHON_EXECUTABLE tests/test_aliyun_api_integration.py
$PYTHON_EXECUTABLE tests/test_audio_components.py
$PYTHON_EXECUTABLE tests/real_epic1_verification.py

# 或使用便捷脚本 (推荐)
./run_epic1_tests.sh

# 端到端测试工具
$PYTHON_EXECUTABLE tools/test_epic1_complete_chain.py
```

### 环境设置
```bash
# 安装依赖
pip3.10 install -r requirements.txt

# 设置开发环境
bash scripts/setup_environment.sh

# 验证环境
bash scripts/validate_environment.sh

# 测试阿里云连接
bash scripts/test_aliyun_connection.sh

# 验证阿里云配置
bash scripts/validate_aliyun_config.sh
```

### 音频设备测试
```bash
# 列出录音设备
arecord -l

# 列出播放设备
aplay -l

# 测试录音
arecord -d 3 -f cd test_recording.wav

# 测试播放
aplay test_recording.wav
```

### ROS2相关命令
```bash
# 列出所有ROS2节点
ros2 node list

# 列出所有话题
ros2 topic list

# 查看话题消息
ros2 topic echo /voice_command

# 查看节点信息
ros2 node info /asr_node
```

---

## 代码架构

### 目录结构
```
xlerobot/
├── src/                              # 源代码目录
│   ├── modules/                      # 核心功能模块
│   │   ├── asr/                      # 语音识别模块
│   │   │   ├── cloud_alibaba/        # 阿里云ASR实现
│   │   │   ├── audio/                # 音频处理
│   │   │   │   ├── audio_player.py   # 音频播放器
│   │   │   │   ├── microphone.py     # 麦克风输入
│   │   │   │   └── preprocessor.py   # 音频预处理
│   │   │   └── streaming/            # 流式处理
│   │   │       └── wake_word_detector.py  # 唤醒词检测
│   │   ├── llm/                      # 大语言模型模块
│   │   │   ├── qwen_client.py        # 通义千问客户端
│   │   │   ├── dialogue_context.py   # 对话上下文管理
│   │   │   └── session_manager.py    # 会话管理
│   │   ├── tts/                      # 语音合成模块
│   │   │   ├── engine/               # TTS引擎实现
│   │   │   │   └── aliyun_tts_engine.py
│   │   │   └── cloud_alibaba/        # 阿里云TTS实现
│   │   └── system_control/           # 系统控制模块
│   │       ├── architecture.py       # 系统架构定义
│   │       └── coordinator/          # 协调器
│   ├── xlerobot_vision/              # 视觉模块 (Qwen-VL)
│   ├── xlerobot_online_dialogue/     # 在线对话服务
│   ├── start_epic1_services.py       # 主服务启动脚本
│   └── aliyun_nls_token_manager.py   # 阿里云Token管理
├── tests/                            # 测试目录
│   ├── README.md                      # 测试文档说明
│   ├── run_all_tests.py               # 测试套件管理器
│   ├── test_aliyun_api_integration.py
│   ├── test_audio_components.py
│   ├── test_e2e_integration.py
│   ├── test_epic1_complete_integration.py
│   ├── verify_epic1_complete_functionality.py
│   └── real_epic1_verification.py
│   └── [其他测试文件...]
├── docs/                             # 文档目录
│   ├── project-overview.md           # 项目概览
│   ├── architecture-analysis.md      # 架构分析
│   ├── tech-stack-documentation.md   # 技术栈文档
│   └── stories/                      # 需求故事文档
├── scripts/                          # 脚本目录
│   ├── setup_environment.sh
│   ├── validate_environment.sh
│   ├── test_aliyun_connection.sh
│   └── verify_project_status.py       # 项目状态验证脚本
├── testing_data/                      # 测试数据目录
│   └── audio_samples/                 # 测试音频样本
│       ├── cantonese/                 # 粤语测试音频
│       ├── jiajia/                    # 家佳测试音频
│       ├── voice_test/                # 语音测试音频
│       └── misc/                      # 其他测试音频
├── config/                           # 配置目录
├── logs/                             # 日志目录
├── requirements.txt                  # Python依赖
├── start_voice_assistant.sh          # 主启动脚本
└── README.md                         # 项目说明
```

### 核心架构模式

#### 1. 模块化设计
每个核心模块 (ASR/LLM/TTS) 都是独立的,通过明确定义的接口进行通信:
- **ASR模块**: 负责音频输入、预处理、语音识别
- **LLM模块**: 负责自然语言理解、对话管理、响应生成
- **TTS模块**: 负责文本预处理、语音合成、音频输出
- **系统控制模块**: 负责协调各模块、资源管理、监控

#### 2. ROS2节点架构
系统基于ROS2分布式架构,设计了15个专业化节点:
- 节点间通过标准ROS2消息通信
- 支持分布式部署和独立扩展
- 实现服务发现和故障恢复

#### 3. 云端服务集成
所有AI能力通过云端API提供:
- **阿里云ASR**: 实时语音识别,支持粤语
- **通义千问API**: 大语言模型推理
- **阿里云TTS**: 高质量语音合成
- **Qwen-VL**: 多模态视觉理解

#### 4. 数据流
```
用户语音输入
  → 音频预处理
  → 唤醒词检测
  → ASR识别
  → 文本理解
  → LLM推理
  → 响应生成
  → TTS合成
  → 音频输出
```

---

## 开发工作流程

### 添加新功能
1. 在 `src/modules/` 下的相应模块中添加代码
2. 确保遵循现有的架构模式 (代理模式、工厂模式等)
3. 在 `tests/` 目录添加对应测试
4. 更新相关文档 (如果需要)

### 修改API集成
1. API客户端代码位于 `src/modules/*/cloud_alibaba/`
2. Token管理在 `src/aliyun_nls_token_manager.py`
3. 确保错误处理和重试机制完善
4. 测试时使用 `tests/test_aliyun_api_integration.py`

### 调试音频问题
1. 使用 `arecord -l` 和 `aplay -l` 检查设备
2. 音频处理逻辑在 `src/modules/asr/audio/`
3. 查看音频参数配置 (采样率16kHz, 单声道, 16-bit)
4. 检查提示音文件 (`src/modules/asr/audio/*.wav`)

### 性能优化
1. 使用异步I/O (`asyncio`) 处理网络请求
2. 考虑批量处理和连接池
3. 监控系统资源使用情况
4. 利用RDK X5 NPU进行硬件加速 (未来)

---

## 重要注意事项

### Python环境
- **绝对不要**使用 `python` 或 `python3`,始终使用 `python3.10`
- **绝对不要**激活Miniconda环境,它会破坏ROS2集成
- 所有脚本和命令都必须显式使用 `python3.10`

### ROS2集成
- 始终在ROS2环境下开发: `source /opt/ros/humble/setup.bash`
- 确保 `PYTHONPATH` 包含项目的 `src` 目录
- 使用ROS2标准消息类型 (std_msgs, sensor_msgs等)

### API密钥管理
- **永远不要**硬编码API密钥到代码中
- 使用环境变量或配置文件
- 生产环境的密钥在启动脚本中设置
- 开发时确保设置了所有必需的环境变量

### 音频设备
- 确保麦克风和扬声器设备可访问
- 检查ALSA设备权限
- 测试录音和播放功能
- 音频格式: 16kHz, 单声道, 16-bit WAV

### 编码规范
- 遵循PEP 8 Python编码规范
- 使用类型提示 (type hints)
- 添加完整的文档注释
- 实现完善的错误处理
- 使用Python logging模块记录日志

### 测试
- 所有新功能必须有对应的测试
- 运行完整测试套件验证改动
- 测试真实的API调用,不使用Mock (项目原则)
- 在真实硬件环境测试音频功能

### 文档维护
- 项目采用严格的文档管理
- 重要文档位于 `docs/` 目录
- 保持架构文档与代码同步
- 记录所有重要决策和变更

---

## 故障排查

### 服务无法启动
1. 运行环境检查: `./start_voice_assistant.sh check`
2. 检查Python版本: `python3.10 --version`
3. 验证ROS2环境: `echo $ROS_DISTRO`
4. 查看日志: `./start_voice_assistant.sh logs`

### API调用失败
1. 验证环境变量是否设置
2. 测试网络连接: `bash scripts/test_aliyun_connection.sh`
3. 检查API密钥有效性
4. 查看详细错误日志

### 音频问题
1. 检查设备: `arecord -l` 和 `aplay -l`
2. 测试录音: `arecord -d 3 -f cd test.wav`
3. 测试播放: `aplay test.wav`
4. 检查ALSA配置和权限

### ROS2节点问题
1. 列出运行中的节点: `ros2 node list`
2. 检查节点状态: `ros2 node info /node_name`
3. 监控话题消息: `ros2 topic echo /topic_name`
4. 检查ROS2日志

---

## 测试文件说明

### 测试组织结构
项目测试采用分层结构，便于维护和扩展：

#### 核心集成测试
- `test_epic1_complete_integration.py` - Epic 1完整集成测试
- `verify_epic1_complete_functionality.py` - Epic 1功能验证
- `real_epic1_verification.py` - 真实环境验证

#### 模块测试
- `test_aliyun_api_integration.py` - 阿里云API集成测试
- `test_audio_components.py` - 音频组件测试
- `test_e2e_integration.py` - 端到端集成测试

#### 工具和验证脚本
- `run_all_tests.py` - 测试套件管理器
- `camera_init.py` - 摄像头初始化测试
- `quick_verification.py` - 快速验证脚本
- `simple_epic1_check.py` - Epic 1简单检查

#### 管道和流程测试
- `test_complete_pipeline.py` - 完整管道测试
- `real_pipeline_test.py` - 真实管道测试
- `fixed_real_pipeline_test.py` - 修复后的管道测试
- `test_audio_pipeline.py` - 音频管道测试

#### 其他专项测试
- `test_dynamic_messages.py` - 动态消息测试
- `test_correct_audio_fix.py` - 音频修复测试
- `test_ros2_nodes.py` - ROS2节点测试
- `story1_2_detailed_analysis.py` - Story 1.2详细分析
- `run_voice_assistant_test.py` - 语音助手运行测试

### 运行测试
```bash
# 运行所有测试
python3.10 tests/run_all_tests.py

# 运行特定测试类别
python3.10 tests/run_all_tests.py --category integration
python3.10 tests/run_all_tests.py --category unit

# 运行单个测试
python3.10 tests/test_epic1_complete_integration.py
```

### 测试数据
测试音频样本按类别组织在 `testing_data/audio_samples/` 目录下：
- `cantonese/` - 粤语语音样本
- `jiajia/` - 家佳语音样本
- `voice_test/` - 语音测试样本
- `misc/` - 其他测试样本

---

## 相关文档

- [README.md](README.md) - Epic 1基本说明
- [docs/project-overview.md](docs/project-overview.md) - 完整项目概览
- [docs/architecture-analysis.md](docs/architecture-analysis.md) - 系统架构分析
- [docs/tech-stack-documentation.md](docs/tech-stack-documentation.md) - 技术栈详细文档
- [docs/prd-epic1-multimodal-services.md](docs/prd-epic1-multimodal-services.md) - Epic 1产品需求

---

## 联系信息

- **项目负责人**: Jody
- **开发语言**: Python 3.10
- **沟通语言**: 中文
- **项目级别**: Brownfield Level 4 Enterprise Scale

---

*最后更新: 2025-11-14*
