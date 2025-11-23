#!/bin/bash
# -*- coding: utf-8 -*-

"""
XleRobot Story 1.4 - 依赖安装脚本
BMad-Method v6 Brownfield Level 4 企业级实现
Story 1.4: 基础语音合成 (阿里云TTS API集成)

安装TTS服务所需的所有依赖项
"""

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查是否以root权限运行
check_root() {
    if [[ $EUID -eq 0 ]]; then
        log_error "请不要以root用户运行此脚本"
        exit 1
    fi
}

# 检查系统版本
check_system() {
    log_info "检查系统版本..."

    if [[ ! -f /etc/os-release ]]; then
        log_error "无法确定系统版本"
        exit 1
    fi

    source /etc/os-release
    log_info "系统: $PRETTY_NAME"

    if [[ "$ID" != "ubuntu" ]]; then
        log_warning "此脚本主要针对Ubuntu系统，其他系统可能需要手动调整"
    fi

    if [[ "${VERSION_ID%%.*}" -lt 20 ]]; then
        log_error "需要Ubuntu 20.04或更高版本"
        exit 1
    fi

    log_success "系统检查通过"
}

# 检查Python版本
check_python() {
    log_info "检查Python版本..."

    # 检查系统Python3
    if ! command -v python3 &> /dev/null; then
        log_error "Python3未安装"
        exit 1
    fi

    PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
    log_info "Python版本: $PYTHON_VERSION"

    # 检查版本是否符合要求 (>=3.10)
    if python3 -c "import sys; exit(0 if sys.version_info >= (3, 10) else 1)"; then
        log_success "Python版本检查通过"
    else
        log_error "需要Python 3.10或更高版本"
        log_info "请使用系统Python: /usr/bin/python3"
        exit 1
    fi
}

# 检查ROS2环境
check_ros2() {
    log_info "检查ROS2环境..."

    # 检查ROS2安装
    if ! command -v ros2 &> /dev/null; then
        log_error "ROS2未安装或未添加到PATH"
        log_info "请安装ROS2 Humble: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi

    # 尝试source ROS2环境
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        source /opt/ros/humble/setup.bash
        log_success "ROS2 Humble环境已加载"
    elif [[ -f "/opt/ros/galactic/setup.bash" ]]; then
        source /opt/ros/galactic/setup.bash
        log_warning "检测到ROS2 Galactic，建议使用Humble版本"
    else
        log_error "未找到ROS2安装目录"
        exit 1
    fi
}

# 安装系统依赖
install_system_deps() {
    log_info "安装系统依赖..."

    # 更新包列表
    sudo apt-get update

    # 安装基础依赖
    sudo apt-get install -y \
        build-essential \
        cmake \
        git \
        curl \
        wget \
        vim \
        pkg-config \
        python3-dev \
        python3-pip \
        python3-venv

    # 安装音频相关依赖
    sudo apt-get install -y \
        alsa-base \
        alsa-utils \
        libasound2-dev \
        portaudio19-dev \
        libsox-dev \
        sox

    # 安装科学计算依赖
    sudo apt-get install -y \
        libffi-dev \
        libssl-dev \
        libsndfile1 \
        libsndfile1-dev

    log_success "系统依赖安装完成"
}

# 安装Python依赖
install_python_deps() {
    log_info "安装Python依赖..."

    # 确保使用正确的Python版本
    PYTHON_CMD="/usr/bin/python3"

    # 升级pip
    $PYTHON_CMD -m pip install --upgrade pip

    # 安装核心依赖
    $PYTHON_CMD -m pip install \
        numpy==1.24.3 \
        scipy==1.10.1 \
        pyyaml==6.0 \
        requests==2.31.0 \
        rclpy==3.3.11 \
        setuptools==65.5.0

    # 安装音频处理依赖
    $PYTHON_CMD -m pip install \
        soundfile==0.12.1 \
        librosa==0.9.2

    # 安装开发和测试依赖
    $PYTHON_CMD -m pip install \
        pytest==7.2.2 \
        pytest-cov==4.0.0 \
        psutil==5.9.4 \
        mock==5.0.1

    log_success "Python依赖安装完成"
}

# 检查音频设备
check_audio_device() {
    log_info "检查音频设备..."

    # 检查ALSA设备
    if command -v aplay &> /dev/null; then
        if aplay -l &> /dev/null; then
            log_success "ALSA音频设备检测正常"
            aplay -l | head -5
        else
            log_warning "未检测到ALSA音频播放设备"
            log_info "请检查音频硬件或配置虚拟音频设备"
        fi
    else
        log_error "ALSA工具未安装"
        exit 1
    fi

    # 测试音频播放
    log_info "测试音频播放..."
    if speaker-test -t sine -f 440 -l 1 &> /dev/null; then
        log_success "音频播放测试通过"
    else
        log_warning "音频播放测试失败，但不影响TTS服务运行"
    fi
}

# 创建必要目录
create_directories() {
    log_info "创建必要目录..."

    # 创建日志目录
    mkdir -p logs

    # 创建临时文件目录
    mkdir -p temp/audio

    # 创建配置目录
    mkdir -p config

    log_success "目录创建完成"
}

# 验证安装
verify_installation() {
    log_info "验证安装..."

    # 检查Python模块
    log_info "检查Python模块导入..."

    modules_to_check=(
        "numpy"
        "scipy"
        "yaml"
        "requests"
        "rclpy"
        "soundfile"
        "psutil"
    )

    for module in "${modules_to_check[@]}"; do
        if /usr/bin/python3 -c "import $module" 2>/dev/null; then
            log_success "✓ $module"
        else
            log_error "✗ $module 导入失败"
            return 1
        fi
    done

    # 检查TTS客户端
    if /usr/bin/python3 -c "
import sys
sys.path.insert(0, 'src')
from xlerobot.tts.aliyun_tts_client import AliyunTTSClient
print('TTS客户端导入成功')
" 2>/dev/null; then
        log_success "✓ TTS客户端模块"
    else
        log_warning "✗ TTS客户端模块 (可能需要先构建项目)"
    fi

    log_success "安装验证完成"
}

# 配置环境变量
setup_environment() {
    log_info "配置环境变量..."

    # 创建环境配置文件
    cat > ~/.bashrc.d/xlerobot_tts.sh << 'EOF'
# XleRobot TTS环境配置
export TTS_LOG_LEVEL=${TTS_LOG_LEVEL:-INFO}
export PYTHONPATH="${PYTHONPATH}:/home/sunrise/xlerobot/src"

# 如果设置了阿里云Token，则导出
if [[ -n "$ALIBABA_CLOUD_TOKEN" ]]; then
    export ALIBABA_CLOUD_TOKEN
else
    echo "警告: ALIBABA_CLOUD_TOKEN未设置，TTS服务将无法正常工作"
fi
EOF

    # 检查bashrc.d目录是否存在
    if [[ ! -d ~/.bashrc.d ]]; then
        mkdir -p ~/.bashrc.d
        echo '# 加载自定义环境配置' >> ~/.bashrc
        echo 'for file in ~/.bashrc.d/*.sh; do [ -r "$file" ] && [ -f "$file" ] && source "$file"; done' >> ~/.bashrc
    fi

    # 立即加载环境配置
    source ~/.bashrc.d/xlerobot_tts.sh

    log_success "环境变量配置完成"
    log_info "请重新加载shell或运行: source ~/.bashrc"
}

# 显示安装后信息
show_post_install_info() {
    log_success "依赖安装完成！"
    echo
    log_info "下一步操作："
    echo "1. 设置阿里云Token:"
    echo "   export ALIBABA_CLOUD_TOKEN='your_token_here'"
    echo
    echo "2. 构建项目:"
    echo "   cd /home/sunrise/xlerobot"
    echo "   colcon build --packages-select xlerobot"
    echo
    echo "3. 启动TTS服务:"
    echo "   source install/setup.bash"
    echo "   ros2 launch xlerobot tts_service.launch.py"
    echo
    echo "4. 运行测试:"
    echo "   python3 tests/unit/test_tts_service.py"
    echo
    log_info "配置文件位置: config/tts_config.yaml"
    log_info "日志文件位置: logs/tts.log"
    log_info "详细文档: docs/api/tts-service-documentation.md"
}

# 主函数
main() {
    log_info "开始安装XleRobot TTS服务依赖..."
    echo

    # 执行安装步骤
    check_root
    check_system
    check_python
    check_ros2
    install_system_deps
    install_python_deps
    check_audio_device
    create_directories
    verify_installation
    setup_environment
    show_post_install_info

    log_success "所有依赖安装完成！"
}

# 错误处理
trap 'log_error "安装过程中发生错误，请检查上面的错误信息"; exit 1' ERR

# 运行主函数
main "$@"