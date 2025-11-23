# XleRobot 部署指南

**文档编号**: XLR-DEPLOY-P0-20251107-001
**项目名称**: XleRobot 家用机器人控制系统
**文档类型**: 部署指南文档
**生成日期**: 2025-11-07
**工作流**: Phase 0 Documentation - document-project

---

## 📋 概述

本部署指南为XleRobot系统提供完整的部署流程、环境配置、监控管理和故障排除指导。严格遵循Brownfield Level 4企业级标准，确保部署过程的可靠性和可维护性。

### 部署目标
- **生产环境**: 稳定可靠的家用机器人控制系统
- **开发环境**: 支持快速迭代和测试的开发平台
- **测试环境**: 完整的功能和性能验证平台
- **演示环境**: 用于展示和演示的环境

### 部署策略
- **渐进式部署**: 分阶段验证，降低风险
- **蓝绿部署**: 确保零停机时间
- **容器化部署**: 简化部署和扩展
- **自动化部署**: 减少人为错误

---

## 🖥️ 硬件环境要求

### 1. 必需硬件
```yaml
核心硬件要求:
  主机平台: D-Robotics RDK X5 V1.0
  处理器: ARM Cortex-A55, 8核
  内存: 8GB RAM (最低7GB可用)
  存储: 128GB SSD (最低50GB可用)
  网络接口: 千兆以太网 + WiFi

音频硬件要求:
  麦克风: USB麦克风 (支持16kHz采样)
  扬声器: 3.5mm音频输出或USB扬声器
  音频接口: 标准音频输入输出接口

可选硬件:
  摄像头: IMX219摄像头 (支持1920x1080@30fps)
  显示屏: HDMI显示器 (可选，用于调试)
  键盘鼠标: USB接口 (开发和配置用)
  网络摄像头: USB摄像头 (替代方案)
```

### 2. 硬件验证清单
```bash
#!/bin/bash
# 硬件验证脚本

echo "🔍 验证硬件环境..."

# 检查CPU
echo "CPU信息:"
lscpu | grep "Model name\|CPU(s):"

# 检查内存
echo -e "\n内存信息:"
free -h

# 检查存储
echo -e "\n存储信息:"
df -h /

# 检查音频设备
echo -e "\n音频设备:"
arecord -l
aplay -l

# 检查摄像头 (如果存在)
echo -e "\n摄像头设备:"
if command -v v4l2-ctl &> /dev/null; then
    v4l2-ctl --list-devices
else
    echo "v4l2-ctl未安装，跳过摄像头检查"
fi

# 检查网络接口
echo -e "\n网络接口:"
ip addr show

# 检查GPU/NPU
echo -e "\nGPU/NPU信息:"
if [ -e "/dev/dri/card0" ]; then
    echo "GPU设备已检测到: /dev/dri/card0"
else
    echo "未检测到GPU设备"
fi

echo -e "\n✅ 硬件验证完成"
```

---

## 💻 软件环境配置

### 1. 操作系统要求
```bash
# 系统要求检查
#!/bin/bash

# 检查Ubuntu版本
lsb_release -a
# 应该显示: Ubuntu 22.04.x LTS

# 检查内核版本
uname -r
# 应该显示: 6.x.x-generic (ARM64)

# 检查系统架构
uname -m
# 应该显示: aarch64 (ARM64)

# 检查用户权限
if [ "$EUID" -ne 0 ]; then
    echo "请使用sudo运行部署脚本"
    exit 1
fi
```

### 2. 系统依赖安装
```bash
#!/bin/bash
# system_deps.sh - 系统依赖安装脚本

set -e

echo "📦 安装系统依赖..."

# 更新包管理器
apt update
apt upgrade -y

# 安装基础开发工具
apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    htop \
    tree \
    unzip \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release

# 安装Python 3.10开发环境
apt install -y \
    python3.10 \
    python3.10-dev \
    python3.10-pip \
    python3.10-venv

# 设置Python 3.10为默认python3
update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1
update-alternatives --install /usr/bin/pip3 pip3 /usr/bin/pip3.10 1

# 安装音频处理库
apt install -y \
    portaudio19-dev \
    python3-pyaudio \
    libasound2-dev \
    libsndfile1-dev \
    ffmpeg \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev

# 安装ROS2 Humble依赖
apt install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common

# 添加ROS2 APT仓库
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 更新包管理器并安装ROS2 Humble
apt update
apt install -y \
    ros-humble-desktop \
    python3-argcomplete \
    python3-colcon-common-extensions

# 安装音频消息包
apt install -y \
    ros-humble-audio-common-msgs \
    ros-humble-sound-play

echo "✅ 系统依赖安装完成"
```

### 3. TROS环境安装
```bash
#!/bin/bash
# tros_install.sh - TROS 2.4.3安装脚本

set -e

echo "🤖 安装TROS 2.4.3..."

# TROS安装包路径 (根据实际情况调整)
TROS_PACKAGE="/path/to/tros-2.4.3-aarch64.tar.gz"

if [ ! -f "$TROS_PACKAGE" ]; then
    echo "❌ TROS安装包不存在: $TROS_PACKAGE"
    echo "请从地平线官方获取TROS 2.4.3安装包"
    exit 1
fi

# 创建安装目录
mkdir -p /opt/tros
cd /opt/tros

# 解压TROS安装包
tar -xzf $TROS_PACKAGE

# 设置权限
chmod -R 755 /opt/tros

# 验证TROS安装
if [ -f "/opt/tros/humble/setup.bash" ]; then
    echo "✅ TROS安装成功"
else
    echo "❌ TROS安装失败"
    exit 1
fi

# 验证TROS算法包数量
ALGORITHM_COUNT=$(ls /opt/tros/humble/lib/ | wc -l)
echo "📊 TROS算法包数量: $ALGORITHM_COUNT"

if [ "$ALGORITHM_COUNT" -lt 50 ]; then
    echo "⚠️ 警告: TROS算法包数量可能不完整"
fi

echo "✅ TROS安装完成"
```

---

## 🚀 应用部署流程

### 1. 项目部署脚本
```bash
#!/bin/bash
# deploy.sh - 主部署脚本

set -e

# 配置变量
PROJECT_ROOT="/home/sunrise/xlerobot"
DEPLOY_USER="sunrise"
BACKUP_DIR="/opt/xlerobot_backup"
LOG_FILE="/var/log/xlerobot_deploy.log"

# 颜色输出函数
print_info() {
    echo -e "\033[34m[INFO]\033[0m $1"
}

print_success() {
    echo -e "\033[32m[SUCCESS]\033[0m $1"
}

print_error() {
    echo -e "\033[31m[ERROR]\033[0m $1"
}

print_warning() {
    echo -e "\033[33m[WARNING]\033[0m $1"
}

# 日志记录函数
log_message() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1" >> $LOG_FILE
}

# 环境检查
check_environment() {
    print_info "检查部署环境..."
    log_message "开始环境检查"

    # 检查用户权限
    if [ "$EUID" -ne 0 ]; then
        print_error "请使用sudo运行部署脚本"
        exit 1
    fi

    # 检查Python版本
    PYTHON_VERSION=$(/usr/bin/python3 --version 2>&1 | grep -o '3\.10')
    if [ -z "$PYTHON_VERSION" ]; then
        print_error "Python 3.10未安装或不是默认版本"
        exit 1
    fi
    print_success "Python 3.10环境正确"

    # 检查ROS2环境
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        print_error "ROS2 Humble未安装"
        exit 1
    fi
    print_success "ROS2 Humble环境正确"

    # 检查TROS环境
    if [ ! -f "/opt/tros/humble/setup.bash" ]; then
        print_error "TROS未安装"
        exit 1
    fi
    print_success "TROS环境正确"

    # 检查项目目录
    if [ ! -d "$PROJECT_ROOT" ]; then
        print_error "项目目录不存在: $PROJECT_ROOT"
        exit 1
    fi
    print_success "项目目录存在"

    log_message "环境检查完成"
}

# 创建备份
create_backup() {
    print_info "创建备份..."
    log_message "开始创建备份"

    BACKUP_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    BACKUP_PATH="$BACKUP_DIR/backup_$BACKUP_TIMESTAMP"

    mkdir -p "$BACKUP_PATH"

    # 备份配置文件
    if [ -d "$PROJECT_ROOT/config" ]; then
        cp -r "$PROJECT_ROOT/config" "$BACKUP_PATH/"
        print_success "配置文件备份完成"
    fi

    # 备份文档
    if [ -d "$PROJECT_ROOT/docs" ]; then
        cp -r "$PROJECT_ROOT/docs" "$BACKUP_PATH/"
        print_success "文档备份完成"
    fi

    # 备份脚本
    if [ -d "$PROJECT_ROOT/scripts" ]; then
        cp -r "$PROJECT_ROOT/scripts" "$BACKUP_PATH/"
        print_success "脚本备份完成"
    fi

    # 保留最近5个备份
    find "$BACKUP_DIR" -type d -name "backup_*" | sort -r | tail -n +6 | xargs rm -rf

    log_message "备份创建完成: $BACKUP_PATH"
}

# 构建项目
build_project() {
    print_info "构建项目..."
    log_message "开始构建项目"

    cd "$PROJECT_ROOT"

    # 激活环境
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash

    # 清理之前的构建
    if [ -d "build" ]; then
        rm -rf build install log
        print_info "清理之前的构建"
    fi

    # 构建ROS2工作空间
    print_info "构建ROS2工作空间..."
    colcon build \
        --symlink-install \
        --parallel-workers $(nproc) \
        --event-handlers console_direct+

    if [ $? -eq 0 ]; then
        print_success "ROS2构建成功"
    else
        print_error "ROS2构建失败"
        exit 1
    fi

    # 构建Python模块
    print_info "构建Python模块..."
    /usr/bin/python3 setup.py build
    /usr/bin/python3 -m pip install -e .

    if [ $? -eq 0 ]; then
        print_success "Python模块构建成功"
    else
        print_error "Python模块构建失败"
        exit 1
    fi

    log_message "项目构建完成"
}

# 运行测试
run_tests() {
    print_info "运行测试..."
    log_message "开始运行测试"

    cd "$PROJECT_ROOT"

    # 激活环境
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    source install/setup.bash

    # 运行单元测试
    print_info "运行单元测试..."
    /usr/bin/python3 -m pytest tests/unit/ -v --tb=short

    if [ $? -eq 0 ]; then
        print_success "单元测试通过"
    else
        print_warning "单元测试存在问题"
    fi

    # 运行集成测试
    print_info "运行集成测试..."
    /usr/bin/python3 -m pytest tests/integration/ -v --tb=short

    if [ $? -eq 0 ]; then
        print_success "集成测试通过"
    else
        print_warning "集成测试存在问题"
    fi

    # 运行硬件测试 (如果硬件可用)
    if [ -e "/dev/dri/card0" ]; then
        print_info "运行硬件测试..."
        ./scripts/test_hardware.sh
    else
        print_warning "跳过硬件测试 (硬件不可用)"
    fi

    log_message "测试运行完成"
}

# 配置系统服务
setup_services() {
    print_info "配置系统服务..."
    log_message "开始配置系统服务"

    # 创建服务用户
    if ! id "$DEPLOY_USER" &>/dev/null; then
        useradd -r -s /bin/false "$DEPLOY_USER"
        print_success "创建服务用户: $DEPLOY_USER"
    fi

    # 创建systemd服务文件
    cat > /etc/systemd/system/xlerobot.service << EOF
[Unit]
Description=XleRobot Voice Assistant Service
After=network.target sound.target
Wants=network.target

[Service]
Type=simple
User=$DEPLOY_USER
Group=$DEPLOY_USER
WorkingDirectory=$PROJECT_ROOT
Environment="PYTHONPATH=$PROJECT_ROOT/src"
Environment="ROS_DOMAIN_ID=42"
ExecStart=/usr/bin/python3 $PROJECT_ROOT/scripts/start_xlerobot.py
ExecStop=/usr/bin/python3 $PROJECT_ROOT/scripts/stop_xlerobot.py
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

    # 重新加载systemd
    systemctl daemon-reload

    # 设置服务自启动
    systemctl enable xlerobot.service

    print_success "系统服务配置完成"
    log_message "系统服务配置完成"
}

# 部署后验证
verify_deployment() {
    print_info "验证部署..."
    log_message "开始部署验证"

    # 检查服务状态
    if systemctl is-active --quiet xlerobot.service; then
        print_success "XleRobot服务运行正常"
    else
        print_error "XleRobot服务未运行"
        systemctl status xlerobot.service
        exit 1
    fi

    # 检查端口监听
    if netstat -ln | grep -q ":8080"; then
        print_success "Web服务端口监听正常"
    else
        print_warning "Web服务端口未监听"
    fi

    # 检查ROS2节点
    source /opt/ros/humble/setup.bash
    source /opt/tros/humble/setup.bash
    NODE_COUNT=$(ros2 node list | wc -l)
    if [ "$NODE_COUNT" -gt 0 ]; then
        print_success "ROS2节点运行正常 ($NODE_COUNT 个节点)"
    else
        print_warning "未检测到ROS2节点"
    fi

    log_message "部署验证完成"
}

# 主函数
main() {
    print_info "开始XleRobot部署..."
    log_message "开始部署流程"

    check_environment
    create_backup
    build_project
    run_tests
    setup_services
    verify_deployment

    print_success "XleRobot部署完成!"
    log_message "部署流程完成"

    # 启动服务
    print_info "启动XleRobot服务..."
    systemctl start xlerobot.service

    print_info "等待服务启动..."
    sleep 10

    if systemctl is-active --quiet xlerobot.service; then
        print_success "XleRobot服务启动成功!"
        print_info "服务状态: $(systemctl is-active xlerobot.service)"
    else
        print_error "XleRobot服务启动失败"
        systemctl status xlerobot.service
        exit 1
    fi

    log_message "部署流程完全结束"
}

# 执行主函数
main "$@"
```

### 2. 启动脚本
```python
#!/usr/bin/env python3
# start_xlerobot.py - XleRobot启动脚本

import os
import sys
import time
import signal
import logging
import subprocess
from pathlib import Path

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/var/log/xlerobot.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class XleRobotStarter:
    def __init__(self):
        self.project_root = Path(__file__).parent.parent
        self.processes = []
        self.running = True

    def setup_environment(self):
        """设置运行环境"""
        logger.info("设置运行环境...")

        # 设置ROS2环境
        os.environ['PYTHONPATH'] = f"{self.project_root}/src:{os.environ.get('PYTHONPATH', '')}"
        os.environ['ROS_DOMAIN_ID'] = '42'
        os.environ['RCUTILS_LOGGING_SEVERITY'] = 'INFO'

        # 激活ROS2和TROS环境
        subprocess.run(['source', '/opt/ros/humble/setup.bash'], shell=True)
        subprocess.run(['source', '/opt/tros/humble/setup.bash'], shell=True)

        logger.info("环境设置完成")

    def start_ros2_nodes(self):
        """启动ROS2节点"""
        logger.info("启动ROS2节点...")

        # 激活ROS2环境
        subprocess.run(['source', '/opt/ros/humble/setup.bash'], shell=True)
        subprocess.run(['source', '/opt/tros/humble/setup.bash'], shell=True)
        subprocess.run(['source', f'{self.project_root}/install/setup.bash'], shell=True)

        # 启动主控制器节点
        cmd = [
            'python3',
            str(self.project_root / 'src/xlerobot_llm/main_controller_node.py')
        ]

        process = subprocess.Popen(
            cmd,
            cwd=self.project_root,
            env=os.environ.copy(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        self.processes.append(process)
        logger.info(f"启动主控制器节点 (PID: {process.pid})")

    def start_web_server(self):
        """启动Web服务器"""
        logger.info("启动Web服务器...")

        cmd = [
            'python3',
            str(self.project_root / 'src/web_server/app.py')
        ]

        process = subprocess.Popen(
            cmd,
            cwd=self.project_root,
            env=os.environ.copy(),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )

        self.processes.append(process)
        logger.info(f"启动Web服务器 (PID: {process.pid})")

    def monitor_processes(self):
        """监控进程状态"""
        while self.running:
            for i, process in enumerate(self.processes):
                if process.poll() is not None:
                    logger.error(f"进程 {i} 异常退出，返回码: {process.returncode}")

                    # 读取错误输出
                    if process.stderr:
                        error_output = process.stderr.read().decode('utf-8')
                        logger.error(f"错误输出: {error_output}")

                    # 重启进程
                    logger.info(f"尝试重启进程 {i}...")
                    self.restart_process(i)

            time.sleep(5)

    def restart_process(self, index):
        """重启指定进程"""
        if index == 0:  # ROS2节点
            self.start_ros2_nodes()
        elif index == 1:  # Web服务器
            self.start_web_server()

    def stop_all(self):
        """停止所有进程"""
        logger.info("停止所有进程...")
        self.running = False

        for process in self.processes:
            try:
                process.terminate()
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()

        self.processes.clear()
        logger.info("所有进程已停止")

    def run(self):
        """运行启动器"""
        try:
            self.setup_environment()
            self.start_ros2_nodes()
            self.start_web_server()

            logger.info("XleRobot启动完成")

            # 设置信号处理
            signal.signal(signal.SIGTERM, self._signal_handler)
            signal.signal(signal.SIGINT, self._signal_handler)

            # 监控进程
            self.monitor_processes()

        except Exception as e:
            logger.error(f"启动过程中发生错误: {e}")
            self.stop_all()
            sys.exit(1)

    def _signal_handler(self, signum, frame):
        """信号处理器"""
        logger.info(f"收到信号 {signum}，正在停止服务...")
        self.stop_all()
        sys.exit(0)

if __name__ == "__main__":
    starter = XleRobotStarter()
    starter.run()
```

---

## 🔧 配置管理

### 1. 环境配置文件
```yaml
# config/environments/production.yaml
production:
  # 基础配置
  debug: false
  log_level: INFO

  # 服务配置
  web_server:
    host: "0.0.0.0"
    port: 8080
    workers: 2

  # ROS2配置
  ros2:
    domain_id: 42
    node_name: "xlerobot_production"

  # 音频配置
  audio:
    sample_rate: 16000
    channels: 1
    format: "wav"
    buffer_size: 1024

  # ASR配置
  asr:
    provider: "alibaba"
    model: "paraformer-realtime-v1"
    language: "cantonese"
    timeout: 5.0

  # TTS配置
  tts:
    provider: "alibaba"
    voice: "cantonese_female"
    speed: 1.0
    pitch: 1.0

  # LLM配置
  llm:
    provider: "qwen"
    model: "qwen-plus"
    max_tokens: 1000
    temperature: 0.7

# config/environments/development.yaml
development:
  debug: true
  log_level: DEBUG

  web_server:
    host: "127.0.0.1"
    port: 8080
    workers: 1

  ros2:
    domain_id: 43
    node_name: "xlerobot_dev"

  # 开发环境特定配置
  hot_reload: true
  auto_restart: true
  mock_services: false
```

### 2. 系统配置脚本
```bash
#!/bin/bash
# configure_system.sh - 系统配置脚本

set -e

echo "⚙️ 配置系统..."

# 创建必要目录
mkdir -p /var/log/xlerobot
mkdir -p /var/lib/xlerobot
mkdir -p /etc/xlerobot
mkdir -p /opt/xlerobot/cache

# 设置权限
chown -R sunrise:sunrise /var/log/xlerobot
chown -R sunrise:sunrise /var/lib/xlerobot
chown -R sunrise:sunrise /etc/xlerobot
chown -R sunrise:sunrise /opt/xlerobot

# 配置日志轮转
cat > /etc/logrotate.d/xlerobot << EOF
/var/log/xlerobot/*.log {
    daily
    missingok
    rotate 30
    compress
    delaycompress
    notifempty
    create 644 sunrise sunrise
    postrotate
        systemctl reload xlerobot.service
    endscript
}
EOF

# 配置防火墙 (如果启用)
if command -v ufw &> /dev/null; then
    ufw allow 8080/tcp
    ufw reload
    echo "防火墙配置完成"
fi

# 配置系统限制
echo "sunrise soft nofile 65536" >> /etc/security/limits.conf
echo "sunrise hard nofile 65536" >> /etc/security/limits.conf

# 优化系统参数
cat > /etc/sysctl.d/99-xlerobot.conf << EOF
# 网络优化
net.core.rmem_max = 16777216
net.core.wmem_max = 16777216
net.ipv4.tcp_rmem = 4096 87380 16777216
net.ipv4.tcp_wmem = 4096 65536 16777216

# 音频优化
fs.inotify.max_user_watches = 524288
vm.swappiness = 10
EOF

sysctl -p /etc/sysctl.d/99-xlerobot.conf

echo "✅ 系统配置完成"
```

---

## 📊 监控和管理

### 1. 健康检查脚本
```python
#!/usr/bin/env python3
# health_check.py - 系统健康检查

import os
import sys
import time
import requests
import subprocess
from pathlib import Path

class HealthChecker:
    def __init__(self):
        self.status = {
            'web_server': False,
            'ros2_nodes': False,
            'audio_devices': False,
            'memory_usage': False,
            'cpu_usage': False,
            'disk_space': False
        }

    def check_web_server(self):
        """检查Web服务器"""
        try:
            response = requests.get('http://localhost:8080/health', timeout=5)
            if response.status_code == 200:
                self.status['web_server'] = True
                print("✅ Web服务器正常")
            else:
                print(f"❌ Web服务器响应异常: {response.status_code}")
        except Exception as e:
            print(f"❌ Web服务器连接失败: {e}")

    def check_ros2_nodes(self):
        """检查ROS2节点"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0 and result.stdout.strip():
                nodes = result.stdout.strip().split('\n')
                if len(nodes) >= 1:
                    self.status['ros2_nodes'] = True
                    print(f"✅ ROS2节点正常 ({len(nodes)} 个节点)")
                else:
                    print("❌ 未发现ROS2节点")
            else:
                print("❌ ROS2节点检查失败")
        except Exception as e:
            print(f"❌ ROS2节点检查异常: {e}")

    def check_audio_devices(self):
        """检查音频设备"""
        try:
            # 检查输入设备
            result = subprocess.run(
                ['arecord', '-l'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0 and 'card' in result.stdout:
                self.status['audio_devices'] = True
                print("✅ 音频设备正常")
            else:
                print("❌ 音频设备异常")
        except Exception as e:
            print(f"❌ 音频设备检查异常: {e}")

    def check_system_resources(self):
        """检查系统资源"""
        try:
            # 检查内存使用率
            result = subprocess.run(
                ['free'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if 'Mem:' in line:
                        parts = line.split()
                        total = int(parts[1])
                        used = int(parts[2])
                        usage_rate = used / total

                        if usage_rate < 0.9:
                            self.status['memory_usage'] = True
                            print(f"✅ 内存使用正常 ({usage_rate:.1%})")
                        else:
                            print(f"⚠️ 内存使用率过高 ({usage_rate:.1%})")
                        break

            # 检查CPU使用率
            result = subprocess.run(
                ['top', '-bn1'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                for line in result.stdout.split('\n'):
                    if '%Cpu(s):' in line:
                        cpu_usage = float(line.split('%')[1].split()[0])
                        if cpu_usage < 80:
                            self.status['cpu_usage'] = True
                            print(f"✅ CPU使用正常 ({cpu_usage:.1f}%)")
                        else:
                            print(f"⚠️ CPU使用率过高 ({cpu_usage:.1f}%)")
                        break

            # 检查磁盘空间
            result = subprocess.run(
                ['df', '-h', '/'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                lines = result.stdout.split('\n')
                for line in lines:
                    if line.startswith('/dev/'):
                        usage = line.split()[4].rstrip('%')
                        usage_rate = int(usage) / 100

                        if usage_rate < 0.9:
                            self.status['disk_space'] = True
                            print(f"✅ 磁盘空间正常 ({usage})")
                        else:
                            print(f"⚠️ 磁盘空间不足 ({usage})")
                        break

        except Exception as e:
            print(f"❌ 系统资源检查异常: {e}")

    def run_health_check(self):
        """运行完整健康检查"""
        print("🏥 开始系统健康检查...")
        print("-" * 50)

        self.check_web_server()
        self.check_ros2_nodes()
        self.check_audio_devices()
        self.check_system_resources()

        print("-" * 50)

        # 计算总体健康状态
        healthy_count = sum(self.status.values())
        total_count = len(self.status)
        health_rate = healthy_count / total_count

        if health_rate >= 0.8:
            print(f"✅ 系统健康状态良好 ({health_rate:.1%})")
            return True
        elif health_rate >= 0.6:
            print(f"⚠️ 系统健康状态一般 ({health_rate:.1%})")
            return True
        else:
            print(f"❌ 系统健康状态较差 ({health_rate:.1%})")
            return False

if __name__ == "__main__":
    checker = HealthChecker()
    is_healthy = checker.run_health_check()
    sys.exit(0 if is_healthy else 1)
```

### 2. 监控仪表板配置
```yaml
# monitoring/dashboard_config.yaml
dashboard:
  title: "XleRobot 系统监控"
  refresh_interval: 30  # 秒

panels:
  - name: "系统状态"
    type: "status"
    metrics:
      - name: "Web服务"
        endpoint: "http://localhost:8080/health"
        expected_status: 200
      - name: "ROS2节点"
        command: "ros2 node list"
        expected_output: "非空"
      - name: "音频设备"
        command: "arecord -l"
        expected_output: "包含设备信息"

  - name: "资源使用"
    type: "resources"
    metrics:
      - name: "CPU使用率"
        command: "top -bn1 | grep '%Cpu'"
        threshold: 80
      - name: "内存使用率"
        command: "free | grep Mem"
        threshold: 90
      - name: "磁盘使用率"
        command: "df -h /"
        threshold: 90

  - name: "服务日志"
    type: "logs"
    sources:
      - file: "/var/log/xlerobot/xlerobot.log"
        level: "ERROR"
        lines: 50
      - file: "/var/log/xlerobot/asr.log"
        level: "WARNING"
        lines: 30
      - file: "/var/log/xlerobot/tts.log"
        level: "INFO"
        lines: 20
```

---

## 🚨 故障排除

### 1. 常见问题诊断
```bash
#!/bin/bash
# troubleshoot.sh - 故障诊断脚本

echo "🔧 XleRobot 故障诊断工具"
echo "=========================="

# 1. 检查服务状态
echo -e "\n1️⃣ 检查服务状态:"
systemctl status xlerobot.service --no-pager

# 2. 检查端口监听
echo -e "\n2️⃣ 检查端口监听:"
netstat -lnp | grep -E ":(8080|11553)"

# 3. 检查ROS2环境
echo -e "\n3️⃣ 检查ROS2环境:"
source /opt/ros/humble/setup.bash 2>/dev/null && echo "✅ ROS2环境正常" || echo "❌ ROS2环境异常"
source /opt/tros/humble/setup.bash 2>/dev/null && echo "✅ TROS环境正常" || echo "❌ TROS环境异常"

# 4. 检查ROS2节点
echo -e "\n4️⃣ 检查ROS2节点:"
ros2 node list 2>/dev/null || echo "❌ 无法获取ROS2节点列表"

# 5. 检查音频设备
echo -e "\n5️⃣ 检查音频设备:"
echo "输入设备:"
arecord -l 2>/dev/null | head -10 || echo "❌ 无法获取音频输入设备"
echo "输出设备:"
aplay -l 2>/dev/null | head -10 || echo "❌ 无法获取音频输出设备"

# 6. 检查Python环境
echo -e "\n6️⃣ 检查Python环境:"
python3 --version
which python3
echo "Python包检查:"
python3 -c "import rclpy; print('✅ rclpy正常')" 2>/dev/null || echo "❌ rclpy异常"
python3 -c "import numpy; print('✅ numpy正常')" 2>/dev/null || echo "❌ numpy异常"

# 7. 检查日志文件
echo -e "\n7️⃣ 检查最近日志:"
if [ -f "/var/log/xlerobot/xlerobot.log" ]; then
    echo "最近10条日志:"
    tail -10 /var/log/xlerobot/xlerobot.log
else
    echo "❌ 日志文件不存在"
fi

# 8. 检查系统资源
echo -e "\n8️⃣ 检查系统资源:"
echo "内存使用:"
free -h
echo "磁盘使用:"
df -h /
echo "CPU负载:"
uptime

# 9. 检查网络连接
echo -e "\n9️⃣ 检查网络连接:"
ping -c 3 8.8.8.8 2>/dev/null && echo "✅ 外网连接正常" || echo "❌ 外网连接异常"

# 10. 生成诊断报告
echo -e "\n📋 生成诊断报告..."
REPORT_FILE="/tmp/xlerobot_diagnostic_$(date +%Y%m%d_%H%M%S).txt"

{
    echo "XleRobot 诊断报告"
    echo "生成时间: $(date)"
    echo "========================"
    echo ""
    echo "系统信息:"
    uname -a
    echo ""
    echo "服务状态:"
    systemctl status xlerobot.service --no-pager
    echo ""
    echo "环境检查:"
    source /opt/ros/humble/setup.bash 2>/dev/null && echo "ROS2: 正常" || echo "ROS2: 异常"
    source /opt/tros/humble/setup.bash 2>/dev/null && echo "TROS: 正常" || echo "TROS: 异常"
    echo ""
    echo "最近错误日志:"
    if [ -f "/var/log/xlerobot/xlerobot.log" ]; then
        grep -i error /var/log/xlerobot/xlerobot.log | tail -10
    fi
} > "$REPORT_FILE"

echo "诊断报告已保存到: $REPORT_FILE"
echo -e "\n🔧 故障诊断完成"
```

### 2. 恢复程序
```bash
#!/bin/bash
# recovery.sh - 系统恢复脚本

set -e

BACKUP_DIR="/opt/xlerobot_backup"
PROJECT_ROOT="/home/sunrise/xlerobot"

echo "🔄 XleRobot 系统恢复"
echo "=================="

# 1. 停止服务
echo "停止现有服务..."
systemctl stop xlerobot.service || true

# 2. 选择备份恢复
echo "可用备份:"
ls -la "$BACKUP_DIR" | grep "^d" | awk '{print $9}' | nl

read -p "请选择要恢复的备份 (输入数字): " backup_choice

if [ -z "$backup_choice" ]; then
    echo "❌ 未选择备份"
    exit 1
fi

BACKUP_NAME=$(ls -la "$BACKUP_DIR" | grep "^d" | awk '{print $9}' | sed -n "${backup_choice}p")

if [ -z "$BACKUP_NAME" ]; then
    echo "❌ 无效的备份选择"
    exit 1
fi

BACKUP_PATH="$BACKUP_DIR/$BACKUP_NAME"

echo "选择备份: $BACKUP_NAME"

# 3. 恢复配置文件
echo "恢复配置文件..."
if [ -d "$BACKUP_PATH/config" ]; then
    cp -r "$BACKUP_PATH/config"/* "$PROJECT_ROOT/config/"
    echo "✅ 配置文件恢复完成"
fi

# 4. 恢复脚本
echo "恢复脚本文件..."
if [ -d "$BACKUP_PATH/scripts" ]; then
    cp -r "$BACKUP_PATH/scripts"/* "$PROJECT_ROOT/scripts/"
    chmod +x "$PROJECT_ROOT/scripts"/*.sh
    echo "✅ 脚本文件恢复完成"
fi

# 5. 重新构建项目
echo "重新构建项目..."
cd "$PROJECT_ROOT"
./scripts/build.sh

# 6. 重新启动服务
echo "重新启动服务..."
systemctl start xlerobot.service

# 7. 验证恢复
echo "验证恢复结果..."
sleep 10

if systemctl is-active --quiet xlerobot.service; then
    echo "✅ 系统恢复成功!"
else
    echo "❌ 系统恢复失败"
    echo "请检查日志: journalctl -u xlerobot.service -f"
    exit 1
fi

echo "🔄 系统恢复完成"
```

---

## 📈 性能优化

### 1. 系统性能调优
```bash
#!/bin/bash
# optimize_performance.sh - 性能优化脚本

echo "⚡ 系统性能优化"
echo "================"

# 1. CPU性能优化
echo "优化CPU性能..."
echo 'performance' | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 2. 内存优化
echo "优化内存设置..."
echo 'vm.swappiness=10' >> /etc/sysctl.conf
echo 'vm.vfs_cache_pressure=50' >> /etc/sysctl.conf

# 3. 网络优化
echo "优化网络设置..."
echo 'net.core.rmem_max = 16777216' >> /etc/sysctl.conf
echo 'net.core.wmem_max = 16777216' >> /etc/sysctl.conf
echo 'net.ipv4.tcp_rmem = 4096 87380 16777216' >> /etc/sysctl.conf
echo 'net.ipv4.tcp_wmem = 4096 65536 16777216' >> /etc/sysctl.conf

# 4. 应用配置优化
echo "优化应用配置..."

# 调整ROS2 QoS设置
cat > "$PROJECT_ROOT/config/ros2_qos.yaml" << EOF
qos_profiles:
  default:
    history: 10
    depth: 10
    reliability: RELIABLE
    durability: VOLATILE

  audio_data:
    history: 5
    depth: 5
    reliability: BEST_EFFORT
    durability: VOLATILE

  system_status:
    history: 1
    depth: 1
    reliability: RELIABLE
    durability: TRANSIENT_LOCAL
EOF

# 5. 应用优化设置
echo "应用优化配置..."

# Python优化
export PYTHONOPTIMIZE=2
export PYTHONDONTWRITEBYTECODE=1

# ROS2优化
export RCUTILS_LOGGING_SEVERITY=ERROR  # 生产环境减少日志
export ROS_DOMAIN_ID=42

echo "✅ 性能优化完成"

# 应用设置
sysctl -p
echo "⚡ 性能优化生效"
```

---

## 📋 部署检查清单

### 部署前检查
- [ ] 硬件环境符合要求 (RDK X5, 音频设备)
- [ ] 操作系统版本正确 (Ubuntu 22.04 LTS)
- [ ] 网络连接正常
- [ ] Python 3.10 环境配置完成
- [ ] ROS2 Humble 安装完成
- [ ] TROS 2.4.3 安装完成
- [ ] 备份现有配置 (如有)
- [ ] 部署脚本权限正确

### 部署过程检查
- [ ] 系统依赖安装成功
- [ ] 项目构建无错误
- [ ] 单元测试通过
- [ ] 集成测试通过
- [ ] 硬件测试通过 (如可用)
- [ ] 系统服务配置完成
- [ ] 服务启动成功
- [ ] 健康检查通过

### 部署后验证
- [ ] Web服务可访问 (http://localhost:8080)
- [ ] ROS2节点运行正常
- [ ] 音频设备工作正常
- [ ] 系统资源使用合理
- [ ] 日志记录正常
- [ ] 监控仪表板工作
- [ ] 性能指标达标
- [ ] 错误处理机制有效

---

*本部署指南遵循Brownfield Level 4企业级标准，为XleRobot系统提供完整的部署指导。文档随部署流程变更持续更新。*