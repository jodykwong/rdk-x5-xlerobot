# 语音助手回归测试环境配置指南

## 1. 硬件环境要求

### 1.1 基础测试设备
- **主测试设备**: 树莓派4B (4GB RAM) 或同等性能设备
- **备用测试设备**: 普通PC/笔记本 (用于对比测试)
- **存储要求**: 至少16GB可用空间
- **网络**: WiFi + 以太网连接 (用于网络异常测试)

### 1.2 音频设备配置
```
主麦克风: USB麦克风，推荐型号：
- Blue Yeti Nano
- Rode NT-USB Mini
- 或同等质量的USB麦克风

备用麦克风: 设备内置麦克风
扬声器: 质量较好的外接扬声器或耳机
音频接口: 确保低延迟 (<50ms)
```

### 1.3 视频设备配置
```
主摄像头: USB摄像头，推荐规格：
- 分辨率: 1080p
- 帧率: 30fps
- 视角: 90度
- 对焦: 自动对焦

备用摄像头: 设备内置摄像头
光照控制: 可调节台灯 (用于不同光线条件测试)
```

### 1.4 网络测试工具
```
网络模拟工具:
- Linux: tc (Traffic Control)
- 跨平台: Clumsy (网络干扰工具)
带宽监控: nload, iftop
延迟测试: ping, mtr
```

### 1.5 噪音控制设备
```
白噪音生成器:
- 软件: online-tone-generator.com
- 硬件: 白噪音机器或手机APP
环境噪音源:
- 电视/收音机 (模拟客厅噪音)
- 多人对话录音
- 交通噪音录音
```

## 2. 软件环境配置

### 2.1 系统要求
```bash
# Linux环境要求
Linux Kernel >= 5.4
Python >= 3.8
Node.js >= 14.0
内存 >= 2GB可用
存储 >= 10GB可用
```

### 2.2 核心软件安装
```bash
# 1. 系统更新
sudo apt update && sudo apt upgrade -y

# 2. 基础依赖
sudo apt install -y \
    python3-pip \
    nodejs \
    npm \
    git \
    curl \
    wget \
    vim \
    htop \
    iotop

# 3. 音频相关
sudo apt install -y \
    alsa-utils \
    pulseaudio \
    pulseaudio-utils \
    sox \
    ffmpeg

# 4. 视频相关
sudo apt install -y \
    v4l-utils \
    cheese \
    opencv-python

# 5. 网络工具
sudo apt install -y \
    tc \
    nload \
    iftop \
    mtr \
    iperf3
```

### 2.3 Python测试环境
```bash
# 创建虚拟环境
python3 -m venv /home/sunrise/xlerobot/test_env
source /home/sunrise/xlerobot/test_env/bin/activate

# 安装测试依赖
pip install -r requirements_test.txt
```

### 2.4 测试工具配置
```bash
# 性能监控
sudo apt install -y \
    sysstat \
    nethogs \
    iotop \
    stress

# 日志收集
sudo apt install -y \
    syslog-ng \
    logrotate

# 自动化测试框架
pip install \
    pytest \
    pytest-html \
    pytest-cov \
    robotframework \
    robotframework-selenium2library
```

## 3. 语音助手系统部署

### 3.1 系统部署检查
```bash
#!/bin/bash
# deploy_check.sh - 部署检查脚本

echo "=== 语音助手系统部署检查 ==="

# 检查系统服务
systemctl status voice-assistant

# 检查进程
ps aux | grep voice-assistant

# 检查端口占用
netstat -tlnp | grep :8080

# 检查硬件设备
echo "=== 音频设备 ==="
arecord -l
aplay -l

echo "=== 视频设备 ==="
v4l2-ctl --list-devices

# 检查权限
echo "=== 权限检查 ==="
groups $USER | grep audio
groups $USER | grep video

# 检查网络连接
ping -c 3 8.8.8.8
```

### 3.2 服务配置文件
```ini
# /etc/systemd/system/voice-assistant.service
[Unit]
Description=Voice Assistant Service
After=network.target sound.target

[Service]
Type=simple
User=sunrise
WorkingDirectory=/home/sunrise/xlerobot
Environment=PYTHONPATH=/home/sunrise/xlerobot/src
ExecStart=/home/sunrise/xlerobot/start_voice_assistant.sh
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

## 4. 测试数据准备

### 4.1 音频测试文件
```bash
# 创建测试音频目录
mkdir -p /home/sunrise/xlerobot/test_data/audio

# 测试音频列表:
# - wake_word_normal.wav (正常"傻强")
# - wake_word_noise.wav (噪音环境"傻强")
# - weather_query.wav ("今日天气点样")
# - time_query.wav ("现在几点")
# - music_command.wav ("放首歌")
# - fast_speech.wav (快速语速)
# - slow_speech.wav (慢速语速)
# - accent_speech.wav (带口音)
```

### 4.2 视觉测试数据
```bash
# 创建测试图像目录
mkdir -p /home/sunrise/xlerobot/test_data/images

# 测试图像列表:
# - phone.jpg (手机)
# - cup.jpg (水杯)
# - remote.jpg (遥控器)
# - book.jpg (书本)
# - bright_light.jpg (强光环境)
# - dark_env.jpg (暗光环境)
# - backlight.jpg (背光环境)
# - complex_bg.jpg (复杂背景)
```

### 4.3 网络测试配置
```bash
#!/bin/bash
# network_test_setup.sh - 网络测试环境配置

# 模拟高延迟
sudo tc qdisc add dev eth0 root netem delay 2000ms

# 模拟丢包
sudo tc qdisc add dev eth0 root netem loss 10%

# 模拟带宽限制
sudo tc qdisc add dev eth0 root netem rate 1mbit

# 清除网络模拟
sudo tc qdisc del dev eth0 root
```

## 5. 监控和日志配置

### 5.1 性能监控配置
```yaml
# monitoring_config.yaml
performance_monitoring:
  metrics:
    - cpu_usage
    - memory_usage
    - disk_io
    - network_io
    - audio_latency
    - video_fps

  sampling_interval: 1s
  log_file: /var/log/voice_assistant/performance.log

  thresholds:
    cpu_warning: 70%
    cpu_critical: 90%
    memory_warning: 80%
    memory_critical: 95%
    disk_warning: 85%
    response_time_warning: 3s
    response_time_critical: 5s
```

### 5.2 日志配置
```python
# logging_config.py
import logging
import logging.config

LOGGING_CONFIG = {
    'version': 1,
    'disable_existing_loggers': False,
    'formatters': {
        'standard': {
            'format': '%(asctime)s [%(levelname)s] %(name)s: %(message)s'
        },
        'detailed': {
            'format': '%(asctime)s [%(levelname)s] %(name)s:%(lineno)d: %(message)s'
        }
    },
    'handlers': {
        'console': {
            'level': 'INFO',
            'class': 'logging.StreamHandler',
            'formatter': 'standard'
        },
        'file': {
            'level': 'DEBUG',
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': '/var/log/voice_assistant/voice_assistant.log',
            'maxBytes': 10485760,  # 10MB
            'backupCount': 5,
            'formatter': 'detailed'
        },
        'error_file': {
            'level': 'ERROR',
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': '/var/log/voice_assistant/error.log',
            'maxBytes': 10485760,
            'backupCount': 5,
            'formatter': 'detailed'
        }
    },
    'loggers': {
        'voice_assistant': {
            'handlers': ['console', 'file', 'error_file'],
            'level': 'DEBUG',
            'propagate': False
        }
    }
}
```

## 6. 环境验证脚本

### 6.1 完整环境检查
```bash
#!/bin/bash
# env_validation.sh - 环境验证脚本

echo "=== 语音助手测试环境验证 ==="
echo "验证时间: $(date)"
echo "验证人员: $USER"
echo ""

# 1. 硬件检查
echo "1. 硬件设备检查"
echo "音频设备:"
arecord -l | head -5
echo "播放设备:"
aplay -l | head -5
echo "视频设备:"
v4l2-ctl --list-devices
echo ""

# 2. 软件环境检查
echo "2. 软件环境检查"
echo "Python版本:"
python3 --version
echo "Node.js版本:"
node --version
echo "系统负载:"
uptime
echo ""

# 3. 网络检查
echo "3. 网络连接检查"
ping -c 2 8.8.8.8 > /dev/null && echo "✓ 外网连接正常" || echo "✗ 外网连接异常"
echo ""

# 4. 服务状态检查
echo "4. 语音助手服务状态"
systemctl is-active voice-assistant && echo "✓ 服务运行中" || echo "✗ 服务未运行"
echo ""

# 5. 权限检查
echo "5. 用户权限检查"
groups $USER | grep -q audio && echo "✓ 音频权限正常" || echo "✗ 缺少音频权限"
groups $USER | grep -q video && echo "✓ 视频权限正常" || echo "✗ 缺少视频权限"
echo ""

# 6. 测试数据检查
echo "6. 测试数据检查"
[ -d "/home/sunrise/xlerobot/test_data" ] && echo "✓ 测试数据目录存在" || echo "✗ 测试数据目录缺失"
echo ""

echo "=== 环境验证完成 ==="
```

## 7. 快速设置指南

### 7.1 一键设置脚本
```bash
#!/bin/bash
# quick_setup.sh - 一键测试环境设置

echo "开始一键设置语音助手测试环境..."

# 执行所有设置步骤
chmod +x deploy_check.sh
chmod +x network_test_setup.sh
chmod +x env_validation.sh

./deploy_check.sh
./env_validation.sh

echo "设置完成！请查看上述输出确认所有组件正常工作。"
```

## 8. 故障排除

### 8.1 常见问题
| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 麦克风无声音 | 权限问题或驱动问题 | 检查用户权限，重新插拔USB设备 |
| 摄像头无法打开 | 被其他应用占用 | 杀死占用进程，检查USB连接 |
| 网络测试失败 | 防火墙或权限问题 | 检查防火墙设置，使用sudo权限 |
| 服务启动失败 | 配置文件错误 | 检查systemd配置文件语法 |
| 音频质量差 | 硬件或驱动问题 | 更换USB接口，更新音频驱动 |

### 8.2 紧急恢复程序
```bash
# 紧急重置测试环境
sudo systemctl stop voice-assistant
sudo tc qdisc del dev eth0 root
sudo pkill -f voice-assistant
source /home/sunrise/xlerobot/test_env/bin/activate
sudo systemctl start voice-assistant
```

---

**配置版本**: v1.0
**最后更新**: 2025-11-13
**维护人员**: BMad Master团队
**文档状态**: 准备就绪