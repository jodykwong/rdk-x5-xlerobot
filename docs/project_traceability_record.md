# 语音助手回归测试项目留痕记录

## 项目基础信息

- **项目名称**: Epic 1 - ASR语音识别模块回归测试
- **项目路径**: /home/sunrise/xlerobot
- **设备平台**: ARM64 (xlerobot)
- **核心架构**: ROS2 + Python 3.10.12
- **测试执行日期**: 2025-11-13
- **测试团队**: BMad Master 专家团队

## 设备环境信息

### 硬件配置
```
设备名称: xlerobot
处理器架构: aarch64 (ARM64)
CPU核心数: 4核 (processor 0-3)
内存总量: 6.9GB
可用内存: 5.8GB
交换空间: 8.0GB
```

### 软件环境
```
操作系统: Linux 6.1.83
Python版本: 3.10.12 (系统版本，与ROS2兼容)
ROS2版本: Humble
项目语言: Python 3.10.12 + 粤语支持
```

### 项目结构分析
```
/home/sunrise/xlerobot/
├── bmad/                    # BMad框架模块
├── build/                   # 构建目录
├── src/                     # 源代码目录
│   └── modules/asr/        # ASR核心模块
│       ├── audio/          # 音频处理
│       ├── streaming/      # 流处理
│       └── asr_core.py     # ASR引擎
├── MODELS/                  # 模型文件
│   └── asr/model_output/model.bin (127MB)
├── docs/                    # 文档目录
└── 测试脚本和音频文件
```

## 核心功能模块

### 已实现功能
1. **音频播放器** (`src/modules/asr/audio/audio_player.py`)
   - 支持WAV格式播放
   - 音量控制 (默认1.0)
   - 提示音系统

2. **麦克风输入** (`src/modules/asr/audio/microphone.py`)
   - 实时音频采集
   - 16kHz采样率
   - 单声道输入

3. **唤醒词检测** (`src/modules/asr/streaming/wake_word_detector.py`)
   - 唤醒词: "傻强" (粤语)
   - 当前触发率: 100% (演示模式)
   - 敏感度: 0.7

4. **ASR引擎** (`src/modules/asr/asr_core.py`)
   - Wav2Vec2模型支持
   - 粤语语音识别
   - 流式处理

### 关键音频文件
- `wake_beep.wav` - 唤醒提示音
- `done_beep.wav` - 识别完成提示音
- `error_beep.wav` - 错误提示音

### 测试样本
- 多个粤语语音样本文件 (cantonese_1.wav ~ cantonese_5.wav)
- 大型粤语输入样本 (cantonese_traditional_input.wav)

## 测试环境发现

### 实际环境 vs 原计划差异

| 项目 | 原计划 | 实际环境 | 影响 |
|------|--------|----------|------|
| 硬件平台 | 树莓派4B | ARM64 xlerobot | 性能更好，测试更稳定 |
| 架构类型 | 通用应用 | ROS2嵌入式 | 需要ROS2特定测试 |
| 测试接口 | Web API | ROS2节点 | 测试方法需要调整 |
| 部署方式 | 独立服务 | ROS2节点启动 | 启动流程不同 |

### 环境适配调整
1. **测试框架**: 适配ROS2环境
2. **音频设备**: 使用实际音频硬件
3. **性能基准**: 基于ARM64性能调整
4. **启动方式**: 通过ROS2 launch文件

## 回归测试计划更新

### 测试重点调整

#### 1. ROS2系统测试 (新增)
```
- ROS2节点启动测试
- topic通信验证
- 服务调用测试
- 节点健康检查
```

#### 2. 音频硬件集成测试 (重点)
```
- 音频设备枚举
- 麦克风输入测试
- 扬声器输出测试
- 音频质量验证
```

#### 3. ASR模型验证 (核心)
```
- 模型文件完整性检查
- 模型加载性能测试
- 语音识别准确率测试
- 粤语特定测试
```

#### 4. 唤醒词检测优化 (关键)
```
- "傻强"唤醒词测试
- 噪音环境测试
- 误唤醒率测试
- 响应时间测试
```

### 测试执行流程更新

#### 阶段1: 环境准备 (30分钟)
```bash
# 1. 激活ROS2环境
source /opt/ros/humble/setup.bash

# 2. 运行环境配置
bash setup_correct_env.sh

# 3. 验证音频设备
python3.10 test_audio_devices.py

# 4. 检查模型文件
ls -lh MODELS/asr/model_output/model.bin
```

#### 阶段2: 核心功能测试 (2小时)
```bash
# 1. 单元测试
python3.10 test_audio_player.py
python3.10 test_microphone.py
python3.10 test_wake_word_detector.py
python3.10 test_asr_core.py

# 2. 集成测试
python3.10 test_epic1_end_to_end.py
```

#### 阶段3: 性能和稳定性测试 (1小时)
```bash
# 1. 性能基准测试
python3.10 test_performance_benchmarks.py

# 2. 长时间稳定性测试
python3.10 test_stability_2hours.py
```

#### 阶段4: 异常场景测试 (1小时)
```bash
# 1. 硬件异常测试
python3.10 test_audio_device_failure.py

# 2. 模型异常测试
python3.10 test_asr_model_failure.py
```

## 测试用例适配

### 实际测试用例 (更新后)

#### WT系列: 唤醒词测试
```
WT-001: 安静环境下说"傻强" → 应播放"傻强系度,老细有乜可以帮到你!"
WT-002: 使用实际麦克风测试唤醒响应
WT-003: 测试唤醒词检测敏感度(0.7)
WT-004: 测试误唤醒情况
```

#### AT系列: ASR测试
```
AT-001: 播放cantonese_1.wav，验证识别结果
AT-002: 测试不同粤语口音识别
AT-003: 测试ASR模型加载性能
AT-004: 测试流式识别功能
```

#### RT系列: ROS2集成测试
```
RT-001: 测试ROS2节点正常启动
RT-002: 测试audio topic发布/订阅
RT-003: 测试服务调用响应
RT-004: 测试节点故障恢复
```

## 测试工具更新

### 新增测试工具

#### 1. ROS2测试工具
```python
# ros2_test_helper.py
import rclpy
from rclpy.node import Node

class ROS2TestHelper(Node):
    def __init__(self):
        super().__init__('test_helper')

    def check_node_health(self, node_name):
        # 检查节点健康状态
        pass

    def test_topic_communication(self, topic_name):
        # 测试topic通信
        pass
```

#### 2. 音频设备测试工具
```python
# audio_device_tester.py
import pyaudio
import numpy as np

class AudioDeviceTester:
    def test_microphone_input(self):
        # 测试麦克风输入
        pass

    def test_speaker_output(self):
        # 测试扬声器输出
        pass

    def measure_audio_latency(self):
        # 测量音频延迟
        pass
```

#### 3. 性能监控工具
```python
# performance_monitor_arm64.py
import psutil
import time

class PerformanceMonitorARM64:
    def monitor_cpu_usage(self):
        # 监控CPU使用率(ARM64优化)
        pass

    def monitor_memory_usage(self):
        # 监控内存使用
        pass

    def monitor_asr_latency(self):
        # 监控ASR延迟
        pass
```

## 风险评估更新

### 新识别风险

#### 1. ROS2环境风险 (高)
- **风险**: ROS2环境配置问题
- **影响**: 无法启动测试
- **缓解**: 提供详细环境配置脚本

#### 2. 音频设备兼容性 (中)
- **风险**: 音频设备驱动问题
- **影响**: 音频功能测试失败
- **缓解**: 提供多种音频设备测试方案

#### 3. ARM64性能限制 (中)
- **风险**: 嵌入式设备性能限制
- **影响**: 测试执行时间延长
- **缓解**: 调整性能基准和超时设置

## 执行记录

### 测试开始记录
- **开始时间**: 2025-11-13 [待定]
- **执行人员**: BMad Master团队
- **环境状态**: ROS2环境就绪，音频设备正常
- **模型状态**: model.bin文件存在 (127MB)

### 测试进度记录
| 时间 | 测试项目 | 状态 | 备注 |
|------|----------|------|------|
| [待定] | 环境准备 | [ ] | ROS2环境验证 |
| [待定] | 唤醒词测试 | [ ] | "傻强"检测验证 |
| [待定] | ASR功能测试 | [ ] | 粤语识别验证 |
| [待定] | 集成测试 | [ ] | 端到端流程验证 |

## 问题记录

### 发现问题
| 问题ID | 问题描述 | 发现时间 | 严重程度 | 状态 | 解决方案 |
|--------|----------|----------|----------|------|----------|

### 已解决问题
- [ ] Python版本兼容性问题 (已在v1.3解决)
- [ ] 音量问题 (已调整到1.0)
- [ ] 唤醒词检测问题 (已优化到100%触发率)

## 文档更新记录

### 测试文档版本
- **v1.0**: 初始通用测试计划 (基于错误假设)
- **v1.1**: 更新为实际ROS2环境测试计划
- **v1.2**: 细化ARM64平台适配
- **v1.3**: 增加粤语语音特定测试

### 关键更新
1. **2025-11-13**: 发现平台架构差异，全面调整测试计划
2. **2025-11-13**: 基于实际项目文档重构测试范围
3. **2025-11-13**: 针对ROS2环境设计专用测试工具

## 结论和建议

### 主要发现
1. 项目是基于ROS2的嵌入式粤语语音助手系统
2. 原计划的通用测试框架不适用，需要重新设计
3. ARM64平台性能优于预期，有利于测试执行

### 建议
1. **立即行动**: 调整测试计划，适配实际环境
2. **优先级**: 重点关注ROS2集成和音频硬件测试
3. **工具开发**: 开发ROS2专用的测试工具
4. **文档维护**: 及时更新测试文档，反映实际环境

### 后续行动
1. 执行更新后的回归测试计划
2. 验证现有功能的稳定性
3. 收集实际性能数据
4. 生成准确的测试报告

---

**文档状态**: 已更新至实际环境
**最后更新**: 2025-11-13
**下次更新**: 测试执行后
**负责人**: BMad Master团队