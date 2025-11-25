#!/bin/bash
# XleRobot Epic 1 快速启动脚本
# =============================
#
# 快速启动纯在线多模态交互服务
# 包含环境检查和错误处理
#
# 使用方法:
#   ./start_voice_assistant.sh    # 启动服务
#   ./start_voice_assistant.sh status  # 检查状态
#   ./start_voice_assistant.sh stop    # 停止服务

set -e

# ============================================
# 🛡️ 加载XLeRobot专用环境配置
# ============================================
# 加载环境脚本，确保使用正确的Python环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$SCRIPT_DIR/xlerobot_env.sh" ]]; then
    source "$SCRIPT_DIR/xlerobot_env.sh"
    log_info "✅ XLeRobot环境已加载"
else
    echo "❌ 错误：找不到xlerobot_env.sh环境脚本"
    echo "请确保在XLeRobot项目根目录中运行此脚本"
    exit 1
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 配置
PROJECT_ROOT="/home/sunrise/xlerobot"
PID_FILE="/tmp/xlerobot_voice_assistant.pid"
LOG_FILE="$PROJECT_ROOT/logs/voice_assistant.log"

# 创建日志目录
mkdir -p "$(dirname "$LOG_FILE")"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" >> "$LOG_FILE" 2>/dev/null || true
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" >> "$LOG_FILE" 2>/dev/null || true
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" >> "$LOG_FILE" 2>/dev/null || true
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" >> "$LOG_FILE" 2>/dev/null || true
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示横幅
show_banner() {
    echo ""
    echo -e "${PURPLE}================================================================${NC}"
    echo -e "${CYAN}🤖 XLeRobot Epic 1 多模态在线智能交互系统${NC}"
    echo -e "${PURPLE}================================================================${NC}"
    echo -e "${CYAN}🌐 架构: 多模态在线服务 (语音+视觉+对话)${NC}"
    echo -e "${CYAN}🎤 功能: ASR → 多模态LLM → TTS + 视觉理解${NC}"
    echo -e "${CYAN}🗣️ 特色: 粤语角色"傻强" + Qwen-VL多模态视觉理解${NC}"
    echo -e "${CYAN}📷 支持: 通义千问VL多模态大模型${NC}"
    echo -e "${CYAN}🛡️ SDK: alibabacloud-nls-python-sdk + 视觉处理${NC}"
    echo -e "${PURPLE}================================================================${NC}"
    echo ""
}

# 显示帮助信息
show_help() {
    cat << EOF
XleRobot Epic 1 语音助手启动脚本

用法: $0 [命令] [选项]

命令:
    (无参数)        启动语音助手服务（包含完整环境检查）
    --force         强制启动（跳过环境检查）
    status          检查服务运行状态
    stop            停止服务
    restart         重启服务
    logs            查看服务日志
    check           仅执行环境检查
    help            显示此帮助信息

环境检查功能:
    ✅ Python环境验证(系统Python 3.10 + Conda冲突检测)
    ✅ 项目结构完整性检查
    ✅ Epic 1多模态功能模块检查:
       🌐 核心模块(ASR/LLM/TTS)
       🎤 阿里云NLS WebSocket SDK检查
       🔊 音频处理依赖(soundfile/numpy/librosa)
       👁️ 视觉理解模块(Qwen-VL多模态)
       💬 在线对话服务模块
       🧠 智能控制模块
       🧪 Epic 1测试套件
    ✅ 核心Python依赖检查:
       💻 系统核心模块(asyncio/threading/requests等)
       🎤 阿里云NLS SDK模块(nls.token/nls.speech_synthesizer等)
       🔊 音频处理模块(soundfile/numpy/librosa)
       👁️ 视觉处理模块(OpenCV/PIL/NumPy/PyTorch)
       🤖 ROS2模块(rclpy/sensor_msgs/std_msgs等)
    ✅ 摄像头设备检查(/dev/video* + 摄像头驱动)
    ✅ 分层API服务连接测试:
       🎤 ASR语音识别服务端点(nls-gateway, nls-meta)
       🔊 TTS语音合成服务端点(阿里云NLS WebSocket)
       🧠 LLM大语言模型服务端点(通义千问API)
       👁️ 视觉理解服务端点(通义千问VL API)
    ✅ 环境变量检查(阿里云API密钥 + QWEN_API_KEY)
    ✅ 音频设备检查(录音+播放设备)
    ✅ 系统资源评估(内存/CPU/磁盘)
    ✅ 权限验证和服务端口检查
    ✅ 多模态服务端口检查

示例:
    $0                  # 启动服务（完整检查）
    $0 --force          # 强制启动（跳过检查）
    $0 status           # 查看状态
    $0 check            # 仅环境检查
    $0 logs             # 查看日志

故障排除:
    如果环境检查失败，请根据提示进行修复：
    1. 安装依赖: pip3.10 install -r requirements.txt
    2. 设置环境变量: export ALIBABA_CLOUD_ACCESS_KEY_ID='your_key'
    3. 检查音频: sudo apt-get install alsa-utils pulseaudio
    4. 强制启动: $0 --force

EOF
}

# 环境检查
check_environment() {
    log_info "🔧 执行全面环境检查..."
    local errors=0
    local warnings=0

    echo -e "${CYAN}📋 环境检查报告${NC}"
    echo "=================================="

    # 第一阶段：硬件设备检查
    log_info "🎯 第一阶段：硬件设备检查"
    echo "----------------------------------"

    # 1. 音频设备检查（优先级最高）
    log_info "🎤 检查音频设备..."
    local audio_devices_ok=true

    # 检查录音设备
    if arecord -l &> /dev/null; then
        local input_devices=$(arecord -l | grep -c "card [0-9]*:")
        log_success "✅ 录音设备: 找到 $input_devices 个设备"
        if [ "$input_devices" -gt 0 ]; then
            echo "   录音设备列表:"
            arecord -l | grep "card [0-9]*:" | sed 's/^/     /'
        fi
    else
        log_error "❌ 无录音设备或arecord不可用"
        audio_devices_ok=false
        ((errors++))
    fi

    # 检查播放设备
    if aplay -l &> /dev/null; then
        local output_devices=$(aplay -l | grep -c "card [0-9]*:")
        log_success "✅ 播放设备: 找到 $output_devices 个设备"
        if [ "$output_devices" -gt 0 ]; then
            echo "   播放设备列表:"
            aplay -l | grep "card [0-9]*:" | sed 's/^/     /'
        fi
    else
        log_error "❌ 无播放设备或aplay不可用"
        audio_devices_ok=false
        ((errors++))
    fi

    # 2. 摄像头设备检查
    log_info "📷 检查摄像头设备和驱动..."

    # 检测操作系统平台
    local platform_os=$(uname -s)
    local skip_camera_check=false

    if [[ "$platform_os" == "Darwin" ]]; then
        # macOS开发环境
        log_info "   📱 检测到macOS开发环境"
        log_warning "   ⚠️  macOS使用AVFoundation框架，不支持Linux V4L2设备"
        log_info "   ℹ️  多模态视觉功能将在RDK X5生产环境部署时启用"
        log_info "   ✅ 跳过V4L2摄像头检查"
        camera_devices=0
        skip_camera_check=true
    elif [ -d "/sys/class/platform" ] && ls /sys/class/platform 2>/dev/null | grep -q "hobot"; then
        # RDK X5平台
        log_info "   📱 检测到RDK X5平台（使用CSI/MIPI摄像头）"
        log_info "   ℹ️  RDK X5使用Hobot驱动，无需USB驱动(uvcvideo)"
        skip_camera_check=false
    else
        # 通用Linux平台
        log_info "   📱 检测到Linux平台"
        skip_camera_check=false
    fi

    # 只在非macOS平台执行V4L2和设备检查
    if [[ "$skip_camera_check" == "false" ]]; then
        # 检查V4L2核心驱动模块加载状态
        log_info "   检查V4L2核心驱动:"
        local driver_modules=("videobuf2_vmalloc" "videobuf2_memops" "videobuf2_v4l2" "videobuf2_common")
        local loaded_drivers=0

        # 先检查lsmod命令是否可用，避免卡死
        if ! command -v lsmod &> /dev/null; then
            log_warning "   ⚠️ lsmod命令不可用，跳过驱动检查"
        else
            for module in "${driver_modules[@]}"; do
                # 使用更安全的超时机制，避免管道阻塞
                if timeout 1 bash -c "lsmod 2>/dev/null | grep -q '^$module '" 2>/dev/null; then
                    log_success "   ✅ V4L2驱动: $module"
                    ((loaded_drivers++))
                else
                    log_warning "   ⚠️ 缺少V4L2驱动: $module（视觉功能可能受限）"
                fi
            done
        fi

        # 检查摄像头设备文件
        log_info "   检查摄像头设备节点:"
        camera_devices=0
        local camera_device_list=""

        for i in {0..9}; do
            local device_path="/dev/video$i"
            if [ -e "$device_path" ]; then
                ((camera_devices++))
                camera_device_list="$camera_device_list $device_path"
                log_success "   ✅ 设备节点: $device_path"

                # 检查设备权限 (添加超时保护防止卡死)
                if timeout 1 bash -c "[ -r '$device_path' ] && [ -w '$device_path' ]" 2>/dev/null; then
                    log_info "      权限: 可读写 ✅"
                else
                    log_warning "      权限: 需要修复或检查超时 ⚠️"
                    log_info "      修复: sudo usermod -a -G video $USER && sudo chmod 666 $device_path"
                    ((warnings++))
                fi
            fi
        done

        if [ $camera_devices -gt 0 ]; then
            log_success "✅ 找到 $camera_devices 个摄像头设备: $camera_device_list"
        else
            log_warning "⚠️ 未找到摄像头设备节点"
            log_info "   多模态服务需要摄像头进行视觉理解"
            ((warnings++))
        fi
    fi

    # 3. 系统资源检查
    log_info "💻 检查系统资源..."

    # 内存检查
    local total_memory=$(free -m | awk 'NR==2{print $2}')
    local available_memory=$(free -m | awk 'NR==2{print $7}')
    local memory_usage=$(( (total_memory - available_memory) * 100 / total_memory ))

    log_info "   内存: ${available_memory}MB可用 / ${total_memory}MB总计 (${memory_usage}%)"
    if [ "$available_memory" -lt 1000 ]; then
        log_warning "⚠️ 可用内存不足1GB，可能影响性能"
        ((warnings++))
    else
        log_success "✅ 内存充足"
    fi

    # CPU检查
    local cpu_cores=$(nproc)
    local cpu_load=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | tr -d ',')
    log_info "   CPU: $cpu_cores 核心，负载: $cpu_load"

    # 磁盘空间检查
    local disk_available=$(df -m "$PROJECT_ROOT" | awk 'NR==2{print $4}')
    if [ "$disk_available" -lt 1000 ]; then
        log_warning "⚠️ 磁盘空间不足: ${disk_available}MB"
        ((warnings++))
    else
        log_success "✅ 磁盘空间充足: ${disk_available}MB"
    fi

    # 第二阶段：运行环境检查
    echo ""
    log_info "🌐 第二阶段：运行环境检查"
    echo "----------------------------------"

    # 4. Python环境检查
    log_info "🐍 检查Python环境..."
    if command -v $PYTHON_EXECUTABLE &> /dev/null; then
        local python_version=$($PYTHON_EXECUTABLE --version 2>&1)
        log_success "✅ Python: $python_version"

        # 检查当前使用的python3是否指向conda
        local current_python3=$(which python3 2>/dev/null || echo "未配置")
        if [[ "$current_python3" == *"conda"* ]] || [[ "$current_python3" == *"miniconda"* ]]; then
            log_error "❌ 检测到conda Python环境冲突！"
            log_error "   当前python3: $current_python3"
            log_error "   XLeRobot必须使用系统Python 3.10，不能使用conda环境"
            log_info "   解决方案:"
            log_info "   1. 运行: source ./xlerobot_env.sh"
            log_info "   2. 或者重新启动shell使配置生效"
            ((errors++))
        else
            log_success "✅ Python环境检查通过，当前python3: $current_python3"
        fi
    else
        log_error "❌ Python3.10 未找到"
        log_info "   请安装Python3.10或设置正确的PATH"
        ((errors++))
    fi

    # 4.1. Miniconda/Conda冲突检测
    log_info "🛡️ 检查conda/miniconda冲突..."

    # 检查PATH中的conda路径
    if echo "$PATH" | grep -q "conda\|miniconda"; then
        log_warning "⚠️  PATH中检测到conda/miniconda路径"
        log_warning "   这可能导致Python环境冲突"
        echo "   以下路径将被清理:"
        echo "$PATH" | tr ':' '\n' | grep -E "conda|miniconda" | sed 's/^/     /'
    else
        log_success "✅ PATH未包含conda/miniconda路径"
    fi

    # 检查活跃的conda环境
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        log_error "❌ 检测到活跃的conda环境: $CONDA_DEFAULT_ENV"
        log_error "   XLeRobot不能在conda环境中运行"
        log_info "   请运行: conda deactivate"
        ((errors++))
    else
        log_success "✅ 未检测到活跃的conda环境"
    fi

    # 5. 项目目录检查
    log_info "📁 检查项目结构..."
    if [ ! -d "$PROJECT_ROOT" ]; then
        log_error "❌ 项目目录不存在: $PROJECT_ROOT"
        ((errors++))
    else
        log_success "✅ 项目目录: $PROJECT_ROOT"

        # 检查关键目录
        local required_dirs=("src" "src/modules" "src/modules/asr" "src/modules/tts" "src/modules/llm")
        for dir in "${required_dirs[@]}"; do
            if [ -d "$PROJECT_ROOT/$dir" ]; then
                log_success "✅ 目录: $dir"
            else
                log_error "❌ 缺少目录: $dir"
                ((errors++))
            fi
        done
    fi

    # 6. ROS2环境检查（多模态系统必需）
    log_info "🤖 检查ROS2环境..."
    local ros2_ok=true

    # 检查ROS2命令
    if command -v ros2 &> /dev/null; then
        local ros2_version=$(ros2 --version 2>&1)
        log_success "✅ ROS2命令: $ros2_version"
    else
        log_error "❌ ROS2命令未找到"
        log_info "   请安装ROS2: sudo apt install ros-humble-desktop"
        ros2_ok=false
        ((errors++))
    fi

    # 检查ROS2环境变量
    if [ -n "$ROS_DISTRO" ]; then
        log_success "✅ ROS发行版: $ROS_DISTRO"
    else
        log_warning "⚠️ ROS_DISTRO环境变量未设置"
        log_info "   请执行: source /opt/ros/humble/setup.bash"
        ((warnings++))
    fi

    # 检查ROS_DOMAIN_ID
    if [ -n "$ROS_DOMAIN_ID" ]; then
        log_success "✅ ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    else
        log_warning "⚠️ ROS_DOMAIN_ID未设置，使用默认值"
        log_info "   建议设置: export ROS_DOMAIN_ID=42"
        ((warnings++))
    fi

    # 检查ROS2节点编译状态
    log_info "   检查ROS2包编译状态..."
    if [ -f "install/setup.bash" ]; then
        log_success "✅ ROS2包已编译"

        # 检查关键包
        local key_packages=(
            "xlerobot:XLeRobot核心节点"
            "audio_msg:音频消息定义"
        )

        for package_info in "${key_packages[@]}"; do
            local package=$(echo "$package_info" | cut -d':' -f1)
            local desc=$(echo "$package_info" | cut -d':' -f2-)

            if [ -d "install/$package" ]; then
                log_success "✅ $desc 已编译"
            else
                log_warning "⚠️ $desc 未编译"
                log_info "   编译: colcon build --packages-select $package"
                ((warnings++))
            fi
        done
    else
        log_error "❌ ROS2包未编译"
        log_info "   编译: colcon build --packages-select xlerobot audio_msg"
        ((errors++))
    fi

    # 检查ROS2 Python模块
    log_info "   检查ROS2 Python模块..."
    local ros_modules=(
        "rclpy:ROS2 Python客户端"
        "sensor_msgs:传感器消息"
        "cv_bridge:CV桥接器"
        "geometry_msgs:几何消息"
        "std_msgs:标准消息"
        "nav_msgs:导航消息"
        "audio_msg:自定义音频消息"
    )

    for module_info in "${ros_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ ROS模块: $desc"
        else
            log_warning "⚠️ 缺少ROS模块: $desc"
            log_info "   安装: sudo apt install ros-humble-$module"
            ((warnings++))
        fi
    done

    # 7. 多模态依赖库检查
    log_info "👁️ 检查多模态Python依赖..."
    local multimodal_modules=(
        "cv2:OpenCV视觉处理"
        "PIL:PIL图像处理"
        "numpy:数值计算库"
        "matplotlib:图像显示"
        "torch:PyTorch深度学习"
        "torchvision:视觉模型库"
        "scipy:科学计算库"
    )

    for module_info in "${multimodal_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ 多模态模块: $desc"
        else
            log_warning "⚠️ 缺少多模态模块: $desc - 视觉功能受限"
            ((warnings++))
        fi
    done

    # 8. 核心Python依赖检查
    log_info "📦 检查核心Python依赖..."
    local critical_modules=(
        "asyncio:异步IO"
        "threading:多线程"
        "queue:队列管理"
        "requests:HTTP客户端"
        "aiohttp:异步HTTP"
        "base64:编码解码"
        "json:JSON处理"
        "logging:日志系统"
        "time:时间模块"
        "os:操作系统接口"
        "sys:系统相关"
        "pathlib:路径处理"
        "signal:信号处理"
        "subprocess:子进程管理"
    )

    for module_info in "${critical_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ 核心模块: $desc"
        else
            log_error "❌ 缺少核心模块: $desc"
            ((errors++))
        fi
    done

    # 9. 阿里云NLS SDK依赖检查
    log_info "🎤 检查阿里云NLS SDK依赖..."
    local nls_modules=(
        "nls:阿里云NLS SDK核心"
        "nls.token:阿里云Token管理"
        "nls.speech_synthesizer:阿里云TTS合成器"
        "nls.speech_recognizer:阿里云ASR识别器"
    )

    for module_info in "${nls_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ NLS模块: $desc"
        else
            log_warning "⚠️ 缺少阿里云NLS模块: $desc"
            log_info "   安装: pip3.10 install alibabacloud-nls-python-sdk"
            ((warnings++))
        fi
    done

    # 10. 音频处理依赖检查
    log_info "🔊 检查音频处理依赖..."
    local audio_modules=(
        "soundfile:音频文件读写"
        "numpy:数值计算"
        "librosa:音频分析"
    )

    for module_info in "${audio_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ 音频模块: $desc"
        else
            log_warning "⚠️ 缺少音频处理模块: $desc"
            log_info "   安装: pip3.10 install soundfile numpy librosa"
            ((warnings++))
        fi
    done

    # 第三阶段：配置和服务检查
    echo ""
    log_info "⚙️ 第三阶段：配置和服务检查"
    echo "----------------------------------"

    # 11. Epic 1多模态功能检查
    log_info "🌐 检查Epic 1多模态功能模块..."

    # 核心ASR/LLM/TTS模块
    local core_files=(
        "start_epic1_services.py:主启动服务"
        "src/modules/asr/simple_aliyun_asr_service.py:ASR服务"
        "src/modules/llm/qwen_client.py:LLM客户端"
        "src/modules/tts/engine/aliyun_tts_client.py:TTS引擎"
    )

    for file_info in "${core_files[@]}"; do
        local file=$(echo "$file_info" | cut -d':' -f1)
        local desc=$(echo "$file_info" | cut -d':' -f2-)
        if [ -f "$PROJECT_ROOT/$file" ]; then
            log_success "✅ $desc: $file"
        else
            log_error "❌ 缺少核心文件: $file"
            ((errors++))
        fi
    done

    # Epic 1多模态视觉模块检查
    log_info "   👁️ 视觉理解模块:"
    local vision_files=(
        "src/xlerobot_vision/xlerobot_vision/qwen_vl_client.py:Qwen-VL视觉客户端"
        "src/xlerobot_vision/vision_llm_node.py:视觉LLM节点"
        "src/xlerobot_vision/multimodal_context.py:多模态上下文管理"
        "src/xlerobot_vision/integration_test.py:多模态集成测试"
    )

    for file_info in "${vision_files[@]}"; do
        local file=$(echo "$file_info" | cut -d':' -f1)
        local desc=$(echo "$file_info" | cut -d':' -f2-)
        if [ -f "$PROJECT_ROOT/$file" ]; then
            log_success "✅ $desc"
        else
            log_warning "⚠️ 缺少视觉模块: $desc - 多模态功能受限"
            ((warnings++))
        fi
    done

    # Epic 1在线对话模块检查
    log_info "   💬 在线对话服务模块:"
    local dialogue_files=(
        "src/xlerobot_online_dialogue/xlerobot_online_dialogue/online_dialogue_api.py:在线对话API"
        "src/xlerobot_online_dialogue/launch/online_dialogue.launch.py:对话服务启动"
        "src/xlerobot_online_dialogue/config/online_dialogue_config.yaml:对话服务配置"
    )

    for file_info in "${dialogue_files[@]}"; do
        local file=$(echo "$file_info" | cut -d':' -f1)
        local desc=$(echo "$file_info" | cut -d':' -f2-)
        if [ -f "$PROJECT_ROOT/$file" ]; then
            log_success "✅ $desc"
        else
            log_warning "⚠️ 缺少对话模块: $desc - 在线对话功能受限"
            ((warnings++))
        fi
    done

    # Epic 1智能模块检查
    log_info "   🧠 智能控制模块:"
    local smart_files=(
        "src/modules/smart_home/iot_service_node.py:智能家居集成"
        "src/modules/system_control/architecture.py:系统架构管理"
        "src/integration/real_voice_assistant.py:真实语音助手"
    )

    for file_info in "${smart_files[@]}"; do
        local file=$(echo "$file_info" | cut -d':' -f1)
        local desc=$(echo "$file_info" | cut -d':' -f2-)
        if [ -f "$PROJECT_ROOT/$file" ]; then
            log_success "✅ $desc"
        else
            log_info "ℹ️ 可选模块: $desc"
        fi
    done

    # Epic 1测试套件检查
    log_info "   🧪 Epic 1测试套件:"
    local test_files=(
        "tests/test_audio_components.py:音频组件测试"
        "tests/test_aliyun_api_integration.py:阿里云API集成测试"
        "tests/test_e2e_integration.py:端到端集成测试"
        "tests/test_runner.py:测试运行器"
    )

    for file_info in "${test_files[@]}"; do
        local file=$(echo "$file_info" | cut -d':' -f1)
        local desc=$(echo "$file_info" | cut -d':' -f2-)
        if [ -f "$PROJECT_ROOT/$file" ]; then
            log_success "✅ $desc"
        else
            log_warning "⚠️ 缺少测试模块: $desc - 质量保证受限"
            ((warnings++))
        fi
    done

    # 4. Python依赖检查
    log_info "📦 检查多模态Python依赖..."
    local critical_modules=(
        "asyncio"
        "threading"
        "queue"
        "requests"
        "aiohttp"
        "base64"
        "json"
        "logging"
        "time"
    )

    for module in "${critical_modules[@]}"; do
        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ 核心模块: $module"
        else
            log_error "❌ 缺少核心模块: $module"
            ((errors++))
        fi
    done

    # 检查视觉处理模块
    log_info "👁️ 检查视觉处理Python模块..."
    local vision_modules=(
        "cv2:OpenCV视觉处理"
        "PIL:PIL图像处理"
        "numpy:数值计算库"
        "matplotlib:图像显示"
        "torch:PyTorch深度学习"
    )

    for module_info in "${vision_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ 视觉模块: $desc"
        else
            log_warning "⚠️ 缺少视觉模块: $desc - 视觉功能受限"
            ((warnings++))
        fi
    done

    # 检查ROS2视觉模块
    log_info "🤖 检查ROS2多模态模块..."
    local ros_modules=(
        "rclpy:ROS2 Python客户端"
        "sensor_msgs:传感器消息"
        "cv_bridge:CV桥接器"
        "geometry_msgs:几何消息"
    )

    for module_info in "${ros_modules[@]}"; do
        local module=$(echo "$module_info" | cut -d':' -f1)
        local desc=$(echo "$module_info" | cut -d':' -f2-)

        if $PYTHON_EXECUTABLE -c "import $module" 2>/dev/null; then
            log_success "✅ ROS模块: $desc"
        else
            log_warning "⚠️ 缺少ROS模块: $desc - 集成功能受限"
            ((warnings++))
        fi
    done

    # 5. 摄像头设备检查（多模态服务必需）
    log_info "📷 检查摄像头设备和驱动..."

    # RDK X5 CSI摄像头初始化（优先处理）
    log_info "   初始化RDK X5 CSI摄像头..."

    # 检查hobot驱动是否加载
    local hobot_drivers=("hobot_mipicsi" "hobot_sensor" "hobot_isi_sensor" "hobot_vin_vnode")
    local hobot_drivers_loaded=0

    if command -v lsmod &> /dev/null; then
        for driver in "${hobot_drivers[@]}"; do
            if timeout 1 bash -c "lsmod 2>/dev/null | grep -q '^$driver '" 2>/dev/null; then
                log_success "   ✅ Hobot驱动: $driver"
                ((hobot_drivers_loaded++))
            fi
        done
    fi

    # 如果hobot驱动已加载，启动cam-service并创建设备映射
    if [ $hobot_drivers_loaded -gt 0 ]; then
        log_info "   启动cam-service..."

        # 检查cam-service是否已运行
        if pgrep -f "cam-service" > /dev/null; then
            log_info "   cam-service已在运行，重启以应用新配置..."
            sudo pkill -f "cam-service" 2>/dev/null || true
            sleep 2
        fi

        # 启动cam-service with优化参数（修正参数格式）
        sudo /usr/hobot/bin/cam-service -C5 3,5,3 -s4,2,4,2 -i6 -V6 &
        sleep 3

        # 检查VIN设备是否创建
        if [ -e "/dev/vin0_cap" ]; then
            log_success "   ✅ VIN设备已创建"

            # 创建V4L2设备映射
            log_info "   创建V4L2设备映射..."
            for i in {0..3}; do
                if [ -e "/dev/vin${i}_cap" ]; then
                    sudo ln -sf "/dev/vin${i}_cap" "/dev/video$i" 2>/dev/null || true
                    sudo chmod 666 "/dev/video$i" 2>/dev/null || true
                    log_success "   ✅ 映射: /dev/vin${i}_cap -> /dev/video$i"
                fi
            done

            # 确保用户权限
            if ! groups $USER | grep -q "vps"; then
                log_info "   添加用户到vps组..."
                sudo usermod -a -G vps $USER 2>/dev/null || true
                log_warning "   ⚠️ 权限修改需要重新登录生效"
            fi
        else
            log_warning "   ⚠️ VIN设备创建失败"
        fi
    else
        log_info "   ℹ️ 未检测到Hobot驱动，跳过CSI摄像头初始化"
    fi

    # 检查摄像头设备文件
    log_info "   检查摄像头设备节点:"
    local camera_devices=0
    local camera_device_list=""

    for i in {0..9}; do
        local device_path="/dev/video$i"
        if [ -e "$device_path" ]; then
            ((camera_devices++))
            camera_device_list="$camera_device_list $device_path"
            log_success "   ✅ 设备节点: $device_path"

            # 检查设备权限 (添加超时保护防止卡死)
            if timeout 1 bash -c "[ -r '$device_path' ] && [ -w '$device_path' ]" 2>/dev/null; then
                log_info "      权限: 可读写 ✅"
            else
                log_warning "      权限: 需要修复或检查超时 ⚠️"
                log_info "      修复: sudo usermod -a -G video $USER && sudo chmod 666 $device_path"
                ((warnings++))
            fi
        fi
    done

    if [ $camera_devices -gt 0 ]; then
        log_success "✅ 找到 $camera_devices 个摄像头设备: $camera_device_list"

        # 尝试获取摄像头详细信息 - 增强超时机制
        if command -v v4l2-ctl &> /dev/null; then
            log_info "   摄像设备详细信息:"
            # 先检查是否有摄像头设备，避免v4l2-ctl卡死
            if compgen -G "/dev/video*" > /dev/null 2>&1; then
                # 使用更严格的超时和错误处理机制
                if timeout 2 bash -c 'v4l2-ctl --list-devices 2>/dev/null' 2>/dev/null >/dev/null; then
                    log_success "   ✅ 摄像头信息获取成功"
                    # 显示设备信息（可选，如果需要）
                    timeout 2 v4l2-ctl --list-devices 2>/dev/null | head -10 | sed 's/^/      /' || true
                else
                    log_warning "   ⚠️ 摄像头信息获取超时或失败"
                fi

                # 检查摄像头支持的格式
                log_info "   检查支持的视频格式:"
                if [ -e "/dev/video0" ]; then
                    # 使用更严格的超时机制
                    if timeout 2 bash -c 'v4l2-ctl -d /dev/video0 --list-formats 2>/dev/null' 2>/dev/null >/dev/null; then
                        log_success "      ✅ 摄像头格式获取成功"
                        # 显示格式信息（可选，如果需要）
                        timeout 2 v4l2-ctl -d /dev/video0 --list-formats 2>/dev/null | head -5 | sed 's/^/      /' || true
                    else
                        log_warning "      ⚠️ 摄像头格式获取超时或失败"
                    fi
                else
                    log_info "      ℹ️ 无主要摄像头设备(/dev/video0)，跳过格式检查"
                fi
            else
                log_info "      ℹ️ 未发现摄像头设备节点，跳过详细信息获取"
            fi
        else
            log_info "      ℹ️ v4l2-ctl工具不可用，跳过详细信息获取"
        fi
    else
        log_warning "⚠️ 未找到摄像头设备节点"
        log_info "   多模态服务需要摄像头进行视觉理解"
        ((warnings++))

        # 诊断建议（针对RDK X5 CSI摄像头）
        log_info "   诊断建议:"
        log_info "   1. 检查Hobot驱动: lsmod | grep hobot"
        log_info "   2. 重启cam-service: sudo /usr/hobot/bin/cam-service -C5 3,5,3 -s4,2,4,2 -i6 -V6"
        log_info "   3. 检查VIN设备: ls -la /dev/vin*_cap"
        log_info "   4. 手动创建映射: sudo ln -sf /dev/vin0_cap /dev/video0"
        log_info "   5. 检查用户权限: groups | grep vps"
    fi

    # 检查摄像头工具
    log_info "   检查摄像头管理工具:"
    local camera_tools=("fswebcam" "ffmpeg" "v4l2-ctl" "cheese" "guvcview")
    local available_tools=0
    local tool_info=""

    for tool in "${camera_tools[@]}"; do
        if command -v "$tool" &> /dev/null; then
            ((available_tools++))
            tool_info="$tool_info $tool"
            log_success "   ✅ 可用工具: $tool"
        fi
    done

    if [ $available_tools -eq 0 ]; then
        log_warning "⚠️ 未找到摄像头管理工具"
        log_info "   安装建议:"
        log_info "   sudo apt-get install v4l-utils fswebcam cheese"
        ((warnings++))
    else
        log_success "✅ 摄像头工具套件: $tool_info"
    fi

    # 检查摄像头权限组
    if ! timeout 2 groups 2>/dev/null | grep -q "video"; then
        log_warning "⚠️ 用户不在video组中"
        log_info "   修复命令: sudo usermod -a -G video $USER"
        log_info "   然后重新登录或使用: newgrp video"
        ((warnings++))
    else
        log_success "✅ 用户已在video组中"
    fi

    # 6. 网络连接检查（区分不同API服务）
    log_info "🌐 检查多模态API服务连接..."

    # ASR服务端点检查
    log_info "   🎤 ASR语音识别服务:"
    local asr_endpoints=(
        "nls-gateway.cn-shanghai.aliyuncs.com:443:阿里云ASR主服务"
        "nls-meta.cn-shanghai.aliyuncs.com:443:阿里云Token服务"
    )

    for endpoint_info in "${asr_endpoints[@]}"; do
        local endpoint=$(echo "$endpoint_info" | cut -d':' -f1-2)
        local desc=$(echo "$endpoint_info" | cut -d':' -f3-)
        local host=$(echo "$endpoint" | cut -d':' -f1)
        local port=$(echo "$endpoint" | cut -d':' -f2)

        if timeout 5 bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null; then
            log_success "✅ $desc ($host:$port)"
        else
            log_error "❌ $desc 不可达 ($host:$port)"
            ((errors++))
        fi
    done

    # TTS服务端点检查
    log_info "   🔊 TTS语音合成服务:"
    local tts_endpoints=(
        "nls-gateway.cn-shanghai.aliyuncs.com:443:阿里云TTS服务"
    )

    for endpoint_info in "${tts_endpoints[@]}"; do
        local endpoint=$(echo "$endpoint_info" | cut -d':' -f1-2)
        local desc=$(echo "$endpoint_info" | cut -d':' -f3-)
        local host=$(echo "$endpoint" | cut -d':' -f1)
        local port=$(echo "$endpoint" | cut -d':' -f2)

        if timeout 5 bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null; then
            log_success "✅ $desc ($host:$port)"
        else
            log_error "❌ $desc 不可达 ($host:$port)"
            ((errors++))
        fi
    done

    # LLM服务端点检查
    log_info "   🧠 LLM大语言模型服务:"
    local llm_endpoints=(
        "dashscope.aliyuncs.com:443:通义千问API"
        "dashscope.aliyuncs.com:443:通义千问推理服务"
    )

    for endpoint_info in "${llm_endpoints[@]}"; do
        local endpoint=$(echo "$endpoint_info" | cut -d':' -f1-2)
        local desc=$(echo "$endpoint_info" | cut -d':' -f3-)
        local host=$(echo "$endpoint" | cut -d':' -f1)
        local port=$(echo "$endpoint" | cut -d':' -f2)

        if timeout 5 bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null; then
            log_success "✅ $desc ($host:$port)"
        else
            log_warning "⚠️ $desc 不可达 ($host:$port) - LLM功能将受限"
            ((warnings++))
        fi
    done

    # 视觉理解服务端点检查
    log_info "   👁️ 视觉理解服务:"
    local vision_endpoints=(
        "dashscope.aliyuncs.com:443:Qwen-VL多模态API"
    )

    for endpoint_info in "${vision_endpoints[@]}"; do
        local endpoint=$(echo "$endpoint_info" | cut -d':' -f1-2)
        local desc=$(echo "$endpoint_info" | cut -d':' -f3-)
        local host=$(echo "$endpoint" | cut -d':' -f1)
        local port=$(echo "$endpoint" | cut -d':' -f2)

        if timeout 5 bash -c "echo >/dev/tcp/$host/$port" 2>/dev/null; then
            log_success "✅ $desc ($host:$port)"
        else
            log_warning "⚠️ $desc 不可达 ($host:$port) - 视觉功能将受限"
            ((warnings++))
        fi
    done

    # 7. 环境变量检查
    log_info "🔑 检查多模态服务环境变量..."
    local env_vars=(
        "ALIBABA_CLOUD_ACCESS_KEY_ID"
        "ALIBABA_CLOUD_ACCESS_KEY_SECRET"
        "ALIYUN_NLS_APPKEY"
        "QWEN_API_KEY"
        "PYTHONPATH"
    )

    for var in "${env_vars[@]}"; do
        if [ -n "${!var}" ]; then
            local value="${!var}"
            if [ ${#value} -gt 20 ]; then
                value="${value:0:20}..."
            fi
            log_success "✅ $var=$value"
        else
            log_warning "⚠️ 未设置: $var"
            ((warnings++))
        fi
    done

    # 8. 阿里云Token管理器功能检查
    log_info "🔑 检查阿里云Token管理器功能..."

    if [ -n "${ALIBABA_CLOUD_ACCESS_KEY_ID:-}" ] && [ -n "${ALIBABA_CLOUD_ACCESS_KEY_SECRET:-}" ]; then
        # 测试Token管理器初始化和Token获取
        local token_test_result
        token_test_result=$($PYTHON_EXECUTABLE -c "
import sys
sys.path.insert(0, '$PROJECT_ROOT/src')
try:
    from aliyun_nls_token_manager import AliyunNLSTokenManager
    import time

    start_time = time.time()
    manager = AliyunNLSTokenManager()
    init_time = time.time() - start_time

    token_start = time.time()
    token = manager.get_token()
    token_time = time.time() - token_start

    if token and len(token) > 0:
        print(f'SUCCESS:{init_time:.3f}:{token_time:.3f}:{len(token)}:{token[:20]}')
    else:
        print(f'FAILED:{init_time:.3f}:{token_time:.3f}:0:None')

except Exception as e:
    print(f'ERROR:0:0:0:{str(e)[:50]}')
" 2>/dev/null || echo "ERROR:0:0:0:ImportError")

        if [[ "$token_test_result" == SUCCESS* ]]; then
            IFS=':' read -r init_time token_time token_length token_preview <<< "$token_test_result"
            log_success "✅ Token管理器功能正常"
            log_info "   - 初始化耗时: ${init_time}s"
            log_info "   - Token获取耗时: ${token_time}s"
            log_info "   - Token长度: ${token_length}字符"
            log_info "   - Token预览: ${token_preview}..."
        elif [[ "$token_test_result" == FAILED* ]]; then
            log_warning "⚠️ Token管理器初始化成功但Token获取失败"
            log_warning "   - 可能原因: 网络问题或API密钥无效"
            ((warnings++))
        else
            log_error "❌ Token管理器功能异常"
            if [[ "$token_test_result" == ImportError* ]]; then
                log_error "   - 导入失败: 请检查aliyun-python-sdk-core依赖"
                log_error "   - 安装命令: pip3.10 install aliyun-python-sdk-core PyYAML"
            else
                log_error "   - 错误信息: ${token_test_result#ERROR:}"
            fi
            ((errors++))
        fi
    else
        log_warning "⚠️ 阿里云API密钥未配置，跳过Token管理器测试"
        log_info "   - 需要设置: ALIBABA_CLOUD_ACCESS_KEY_ID, ALIBABA_CLOUD_ACCESS_KEY_SECRET"
        ((warnings++))
    fi

    # 9. 音频设备检查
    log_info "🎤 检查音频设备..."

    # 检查录音设备
    if arecord -l &> /dev/null; then
        local input_devices=$(arecord -l | grep -c "card [0-9]*:")
        log_success "✅ 录音设备: 找到 $input_devices 个设备"

        # 显示设备详情
        if [ "$input_devices" -gt 0 ]; then
            echo "   录音设备列表:"
            arecord -l | grep "card [0-9]*:" | sed 's/^/     /'
        fi
    else
        log_error "❌ 无录音设备或arecord不可用"
        ((errors++))
    fi

    # 检查播放设备
    if aplay -l &> /dev/null; then
        local output_devices=$(aplay -l | grep -c "card [0-9]*:")
        log_success "✅ 播放设备: 找到 $output_devices 个设备"

        # 显示设备详情
        if [ "$output_devices" -gt 0 ]; then
            echo "   播放设备列表:"
            aplay -l | grep "card [0-9]*:" | sed 's/^/     /'
        fi
    else
        log_error "❌ 无播放设备或aplay不可用"
        ((errors++))
    fi

    # 10. 系统资源检查
    log_info "💻 检查系统资源..."

    # 内存检查
    local total_memory=$(free -m | awk 'NR==2{print $2}')
    local available_memory=$(free -m | awk 'NR==2{print $7}')
    local memory_usage=$(( (total_memory - available_memory) * 100 / total_memory ))

    log_info "   内存: ${available_memory}MB可用 / ${total_memory}MB总计 (${memory_usage}%)"
    if [ "$available_memory" -lt 1000 ]; then
        log_warning "⚠️ 可用内存不足1GB，可能影响性能"
        ((warnings++))
    else
        log_success "✅ 内存充足"
    fi

    # CPU检查
    local cpu_cores=$(nproc)
    local cpu_load=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | tr -d ',')
    log_info "   CPU: $cpu_cores 核心，负载: $cpu_load"

    # 磁盘空间检查
    local disk_available=$(df -m "$PROJECT_ROOT" | awk 'NR==2{print $4}')
    if [ "$disk_available" -lt 1000 ]; then
        log_warning "⚠️ 磁盘空间不足: ${disk_available}MB"
        ((warnings++))
    else
        log_success "✅ 磁盘空间充足: ${disk_available}MB"
    fi

    # 10. 权限检查
    log_info "🔐 检查权限..."

    # 检查项目目录写权限
    if [ -w "$PROJECT_ROOT" ]; then
        log_success "✅ 项目目录写权限正常"
    else
        log_error "❌ 项目目录无写权限"
        ((errors++))
    fi

    # 检查日志目录权限
    local log_dir=$(dirname "$LOG_FILE")
    if [ -w "$log_dir" ]; then
        log_success "✅ 日志目录写权限正常"
    else
        log_error "❌ 日志目录无写权限: $log_dir"
        ((errors++))
    fi

    # 11. 端口检查
    log_info "🔌 检查多模态服务端口占用..."
    local ports=("8000" "8080" "9000" "3000" "5000" "8081")

    for port in "${ports[@]}"; do
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            log_warning "⚠️ 端口 $port 被占用"
            ((warnings++))
        else
            log_success "✅ 端口 $port 可用"
        fi
    done

    # 总结检查结果
    echo "=================================="
    log_info "📊 环境检查总结:"
    log_info "   - 错误: $errors 个"
    log_info "   - 警告: $warnings 个"

    if [ $errors -gt 0 ]; then
        log_error "❌ 环境检查失败，存在 $errors 个错误"
        echo ""
        log_info "🛠️ 建议修复步骤:"
        if [ $errors -gt 0 ]; then
            echo "   1. 安装缺失的Python模块: pip3.10 install -r requirements.txt"
            echo "   2. 检查音频设备: sudo apt-get install alsa-utils pulseaudio"
            echo "   3. 设置环境变量: export ALIBABA_CLOUD_ACCESS_KEY_ID='your_key'"
            echo "   4. 确保项目完整: git pull 或重新克隆"
        fi
        echo ""
        log_error "请修复错误后重试，或使用 --force 参数强制启动"
        exit 1
    elif [ $warnings -gt 0 ]; then
        log_warning "⚠️ 环境检查通过，但存在 $warnings 个警告"
        log_info "服务可能可以运行，但建议解决警告问题"
    else
        log_success "🎉 环境检查完全通过！系统准备就绪"
    fi

    echo ""
}

# 检查服务状态 - 基于ROS2节点列表而非PID文件
check_status() {
    # 检查ROS2环境是否可用
    if ! command -v ros2 &> /dev/null; then
        log_error "❌ ROS2命令不可用，请检查环境配置"
        return 1
    fi

    # 检查XLeRobot相关节点是否运行
    local xlerobot_nodes=$(ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" | wc -l)

    if [ "$xlerobot_nodes" -gt 0 ]; then
        log_success "✅ XLeRobot服务正在运行 ($xlerobot_nodes 个节点)"

        # 显示活跃的ROS2节点
        echo -e "${CYAN}活跃的ROS2节点:${NC}"
        ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" | while read node; do
            echo -e "  ✅ $node"
            # 获取节点信息（如果可用）
            local node_info=$(ros2 node info "$node" 2>/dev/null | head -3 | tr '\n' ' ' | sed 's/[[:space:]]*$//')
            if [ -n "$node_info" ]; then
                echo -e "     $node_info"
            fi
        done

        # 显示相关进程信息
        echo -e "\n${CYAN}相关进程:${NC}"
        ps aux | grep -E "(ros2.*xlerobot|voice_assistant)" | grep -v grep | while read line; do
            local pid=$(echo "$line" | awk '{print $2}')
            local cmd=$(echo "$line" | awk '{print $11,$12,$13}' | sed 's/[[:space:]]*$//')
            local cpu=$(echo "$line" | awk '{print $3}')
            local mem=$(echo "$line" | awk '{print $4}')
            echo -e "  📊 PID $pid: $cmd (CPU: ${cpu}%, MEM: ${mem}%)"
        done

        return 0
    else
        log_warning "⚠️ XLeRobot服务未运行"

        # 检查是否有残留的相关进程
        local orphan_processes=$(ps aux | grep -E "(ros2.*xlerobot|voice_assistant)" | grep -v grep | wc -l)
        if [ "$orphan_processes" -gt 0 ]; then
            log_warning "⚠️ 发现 $orphan_processes 个残留进程"
            ps aux | grep -E "(ros2.*xlerobot|voice_assistant)" | grep -v grep | while read line; do
                local pid=$(echo "$line" | awk '{print $2}')
                local cmd=$(echo "$line" | awk '{print $11,$12,$13}' | sed 's/[[:space:]]*$//')
                echo -e "  ❌ PID $pid: $cmd"
            done
        fi

        return 1
    fi
}

# 清理ROS2临时文件
cleanup_ros2_temp_files() {
    log_info "🧹 清理ROS2临时文件..."

    # 统计需要清理的文件
    local total_files=$(find /tmp -name "launch_params_*" -type f 2>/dev/null | wc -l)

    if [ "$total_files" -gt 0 ]; then
        log_info "📁 发现 $total_files 个ROS2 launch参数文件需要清理"

        # 批量删除文件
        find /tmp -name "launch_params_*" -type f 2>/dev/null -exec rm -f {} \; 2>/dev/null || true

        # 验证清理结果
        local remaining_files=$(find /tmp -name "launch_params_*" -type f 2>/dev/null | wc -l)
        local cleaned_files=$((total_files - remaining_files))

        log_info "📁 成功清理了 $cleaned_files 个ROS2 launch参数文件"

        if [ "$remaining_files" -gt 0 ]; then
            log_warning "⚠️ 仍有 $remaining_files 个文件无法清理（可能正在使用中）"
        fi
    else
        log_info "📁 未发现需要清理的ROS2 launch参数文件"
    fi

    # 清理其他可能的ROS2相关临时文件
    local other_temp_files=$(find /tmp -name "*ros2*" -o -name "*xlerobot*" 2>/dev/null | wc -l)
    if [ "$other_temp_files" -gt 0 ]; then
        log_info "📁 发现 $other_temp_files 个其他相关临时文件"
        find /tmp -name "*ros2*" -o -name "*xlerobot*" 2>/dev/null -exec rm -f {} \; 2>/dev/null || true
        log_info "📁 已清理其他相关临时文件"
    fi
}

# 启动服务
start_service() {
    local force_start=false
    if [ "$1" = "--force" ]; then
        force_start=true
        log_warning "⚠️ 强制启动模式，跳过部分检查"
    fi

    log_info "🚀 启动XleRobot Epic 1语音助手..."

    # 检查是否已经运行
    if check_status > /dev/null 2>&1; then
        log_warning "⚠️ XLeRobot服务已在运行中"
        return 0
    fi

    # 环境检查
    if [ "$force_start" = false ]; then
        check_environment
    else
        log_info "⚠️ 跳过环境检查（强制启动模式）"
    fi

    # 切换到项目目录
    cd "$PROJECT_ROOT"

    # 🔧 新增：ROS2包编译检查和自动编译
    log_info "🔨 检查ROS2包编译状态..."
    if [ ! -d "install/xlerobot" ] || [ ! -f "install/xlerobot/share/xlerobot/launch/voice_assistant.launch.py" ]; then
        log_info "📦 ROS2包未编译或编译不完整，开始自动编译..."

        # 确保rosdep已初始化
        if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
            log_info "🔧 初始化rosdep..."
            sudo apt-get update -qq
            sudo apt-get install -y python3-pip
            sudo pip3 install rosdep
            sudo rosdep init
            rosdep update
        fi

        # 安装依赖
        log_info "📦 安装Python依赖..."
        rosdep install --from-paths src --ignore-src -r -y

        # 编译包 - 根据平台选择要编译的包
        log_info "🔨 编译ROS2包..."

        # 检测操作系统平台
        local platform_os=$(uname -s)
        if [[ "$platform_os" == "Darwin" ]]; then
            # macOS开发环境：跳过camera和vision包（依赖Linux硬件）
            log_info "   📱 macOS环境：编译核心包（xlerobot, audio_msg）"
            log_info "   ⏭️  跳过 xlerobot_camera, xlerobot_vision（需要Linux V4L2支持）"
            colcon build --packages-select xlerobot audio_msg --symlink-install \
                --packages-skip xlerobot_camera xlerobot_vision
        else
            # Linux生产环境：编译所有包
            log_info "   🐧 Linux环境：编译所有包"
            colcon build --packages-select xlerobot xlerobot_camera xlerobot_vision audio_msg --symlink-install
        fi

        if [ $? -eq 0 ]; then
            log_success "✅ ROS2包编译成功"
        else
            log_error "❌ ROS2包编译失败，请检查错误日志"
            exit 1
        fi
    else
        log_success "✅ ROS2包已编译"
    fi

    # 设置环境变量
    export PYTHONPATH="$PROJECT_ROOT/src:$PYTHONPATH"
    export ROS_DOMAIN_ID="42"

    # 从环境文件加载API密钥
    if [[ -f ".env" ]]; then
        source .env
    fi

    # 验证必需的环境变量
    required_vars=("ALIBABA_CLOUD_ACCESS_KEY_ID" "ALIBABA_CLOUD_ACCESS_KEY_SECRET" "ALIYUN_NLS_APPKEY" "QWEN_API_KEY")
    for var in "${required_vars[@]}"; do
        if [[ -z "${!var}" ]]; then
            echo "❌ 错误: 环境变量 $var 未设置"
            echo "请确保在 .env 文件中设置了所有必需的API密钥"
            exit 1
        fi
    done

    log_info "🔧 环境变量已设置"

    # ========== 新增：禁用PulseAudio（避免设备占用）==========
    log_info "🔧 准备音频设备..."

    # 检查PulseAudio是否运行
    if pgrep -x "pulseaudio" > /dev/null; then
        log_warning "⚠️ 检测到PulseAudio进程占用音频设备，正在停止..."

        # 停止PulseAudio
        pkill pulseaudio 2>/dev/null || true

        # 等待进程退出
        sleep 1

        # 验证是否停止
        if pgrep -x "pulseaudio" > /dev/null; then
            log_error "❌ PulseAudio停止失败，强制终止..."
            pkill -9 pulseaudio 2>/dev/null || true
            sleep 1
        fi

        # 最终验证
        if pgrep -x "pulseaudio" > /dev/null; then
            log_error "❌ 无法停止PulseAudio，ASR可能无法工作"
        else
            log_success "✅ PulseAudio已停止，音频设备已释放"
        fi
    else
        log_info "✅ PulseAudio未运行，音频设备可用"
    fi

    # 防止PulseAudio自动重启
    export PULSE_SERVER="none"
    export PULSE_RUNTIME_PATH="/tmp/pulse-disabled"

    log_success "🔧 音频设备准备就绪"
    # =======================================================

    # 启动服务
    log_info "🎤 启动ROS2语音助手服务..."

    # 检查ROS2包是否编译
    if [ ! -f "install/setup.bash" ]; then
        log_error "❌ ROS2包未编译，请运行: colcon build --packages-select xlerobot audio_msg"
        return 1
    fi

    # 启动ROS2 Launch系统
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    # 验证xlerobot包是否可以被发现
    if ! ros2 pkg list | grep -q xlerobot; then
        log_error "❌ xlerobot包未找到，请检查构建状态"
        log_error "请运行: colcon build --packages-select xlerobot audio_msg"
        exit 1
    fi
    log_success "✅ xlerobot包验证通过"

    # 验证launch文件是否存在
    if [ ! -f "install/xlerobot/share/xlerobot/launch/voice_assistant.launch.py" ]; then
        log_error "❌ launch文件不存在: voice_assistant.launch.py"
        log_error "   🔧 修复建议:"
        log_error "   1. 重新编译: colcon build --packages-select xlerobot"
        log_error "   2. 检查项目结构: ls install/"
        exit 1
    fi
    log_success "✅ launch文件验证通过"

    # 🔧 修复：在当前环境中诊断ROS2包发现
    log_info "🔍 诊断ROS2包发现..."

    # 确保ROS2环境生效
    source /opt/ros/humble/setup.bash
    source install/setup.bash 2>/dev/null || true

    # 检查xlerobot包
    local xlerobot_found=false
    if ros2 pkg list | grep -q xlerobot; then
        log_success "✅ xlerobot包已正确注册到ROS2"
        xlerobot_found=true
    else
        log_error "❌ xlerobot包未找到，需要重新编译或刷新环境"
    fi

    # 修复环境配置问题 - 确保ROS2环境正确传递
    # 验证环境变量完整性
    if [[ -z "${QWEN_API_KEY}" ]]; then
        log_error "❌ QWEN_API_KEY 环境变量未设置，服务可能无法正常运行"
        log_warning "⚠️ 请确保设置了 QWEN_API_KEY 环境变量"
    fi

    # 🔧 修复：先确保环境变量在当前shell中完全生效
    export AMENT_PREFIX_PATH="/home/sunrise/xlerobot/install:/opt/ros/humble:$AMENT_PREFIX_PATH"
    export PYTHONPATH="/home/sunrise/xlerobot/install/audio_msg/local/lib/python3.10/dist-packages:/home/sunrise/xlerobot/install/xlerobot/local/lib/python3.10/dist-packages:/home/sunrise/xlerobot/src:$PYTHONPATH"
    export PYTHON_EXECUTABLE="/usr/bin/python3.10"

    # 确保ROS2环境在当前shell中生效
    source /opt/ros/humble/setup.bash
    source install/setup.bash 2>/dev/null || log_warning "⚠️ install/setup.bash未找到，使用环境变量"

    log_info "🚀 启动ROS2服务，环境变量已完全设置..."
    log_info "  PYTHON_EXECUTABLE: $PYTHON_EXECUTABLE"
    log_info "  ALIBABA_CLOUD_ACCESS_KEY_ID: ${ALIBABA_CLOUD_ACCESS_KEY_ID:0:10}..."
    log_info "  QWEN_API_KEY: ${QWEN_API_KEY:0:10}..."

    # 🔧 修复：使用env命令显式传递环境变量到ROS2子进程
    # 避免nohup导致的环境变量丢失
    nohup env \
        QWEN_API_KEY="$QWEN_API_KEY" \
        ALIBABA_CLOUD_ACCESS_KEY_ID="$ALIBABA_CLOUD_ACCESS_KEY_ID" \
        ALIBABA_CLOUD_ACCESS_KEY_SECRET="$ALIBABA_CLOUD_ACCESS_KEY_SECRET" \
        ALIYUN_NLS_APPKEY="$ALIYUN_NLS_APPKEY" \
        PYTHONPATH="$PYTHONPATH" \
        PYTHON_EXECUTABLE="$PYTHON_EXECUTABLE" \
        ros2 launch xlerobot voice_assistant.launch.py \
        tts_voice:=xiaoyun \
        log_level:=info \
        > "$LOG_FILE" 2>&1 &
    local pid=$!

    log_success "✅ ROS2启动进程已创建... (启动PID: $pid)"
    log_info "⏳ 等待XLeRobot服务启动..."

    # 等待服务启动并基于ROS2节点检查状态
    sleep 8  # 增加等待时间，确保ROS2 launch完全启动

    # 检查XLeRobot节点是否成功启动
    local xlerobot_nodes=$(ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" | wc -l)
    if [ "$xlerobot_nodes" -gt 0 ]; then
        log_success "🎉 XLeRobot Epic 1语音助手启动成功！"
        log_info "📊 活跃节点 ($xlerobot_nodes 个):"
        ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" | sed 's/^/  ✅ /'
        log_info "📁 日志文件: $LOG_FILE"
        log_info "🎮 使用 '$0 logs' 查看实时日志"
        log_info "🛑 使用 '$0 stop' 停止服务"
        log_info "🔄 使用 '$0 restart' 重启服务"
        log_info "📊 使用 '$0 status' 查看服务状态"
        log_info "🎤 现在可以尝试唤醒'傻强'了！"
    else
        log_error "❌ XLeRobot ROS2节点未正常启动，请检查日志"
        tail -n 30 "$LOG_FILE"

        # 清理启动进程
        if ps -p "$pid" > /dev/null 2>&1; then
            kill "$pid" 2>/dev/null
            sleep 2
            if ps -p "$pid" > /dev/null 2>&1; then
                kill -KILL "$pid" 2>/dev/null
            fi
        fi
        exit 1
    fi
}

# 停止服务 - 基于ROS2节点和进程名称进行清理
stop_service() {
    log_info "🛑 停止XLeRobot Epic 1语音助手..."

    # 检查并停止XLeRobot相关进程
    local stopped_count=0
    local process_count=0

    # 1. 停止ROS2节点（优雅方式）
    if command -v ros2 &> /dev/null; then
        log_info "🔄 停止ROS2节点..."
        local xlerobot_nodes=$(ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" 2>/dev/null)

        if [ -n "$xlerobot_nodes" ]; then
            echo "$xlerobot_nodes" | while read node; do
                if [ -n "$node" ]; then
                    log_info "  停止节点: $node"
                    ros2 lifecycle set "$node" shutdown 2>/dev/null || true
                    stopped_count=$((stopped_count + 1))
                fi
            done
            sleep 2  # 等待节点优雅关闭
        else
            log_info "  未发现运行中的XLeRobot ROS2节点"
        fi
    else
        log_warning "⚠️ ROS2命令不可用，跳过节点停止"
    fi

    # 2. 停止相关进程（基于进程名称）
    log_info "🔄 停止相关进程..."

    # 定义要停止的进程模式
    local process_patterns=(
        "ros2.*xlerobot"
        "voice_assistant"
        "asr_bridge_node"
        "llm_service_node"
        "tts_service_node"
        "voice_assistant_coordinator"
        "start_epic1_services"
    )

    for pattern in "${process_patterns[@]}"; do
        local pids=$(ps aux | grep -E "$pattern" | grep -v grep | awk '{print $2}')
        if [ -n "$pids" ]; then
            echo "$pids" | while read pid; do
                if [ -n "$pid" ] && ps -p "$pid" > /dev/null 2>&1; then
                    process_count=$((process_count + 1))
                    local cmd=$(ps -p "$pid" -o cmd= --no-headers | head -c 50)
                    log_info "  停止进程: PID $pid ($cmd...)"

                    # 优雅停止
                    kill -TERM "$pid" 2>/dev/null || true

                    # 等待进程停止
                    local count=0
                    while ps -p "$pid" > /dev/null 2>&1 && [ $count -lt 5 ]; do
                        sleep 1
                        count=$((count + 1))
                    done

                    # 如果仍在运行，强制终止
                    if ps -p "$pid" > /dev/null 2>&1; then
                        log_warning "  强制终止进程: PID $pid"
                        kill -KILL "$pid" 2>/dev/null || true
                    fi
                fi
            done
        fi
    done

    # 3. 清理旧的PID文件（如果存在）
    if [ -f "$PID_FILE" ]; then
        local old_pid=$(cat "$PID_FILE")
        rm -f "$PID_FILE"
        log_info "🧹 清理旧PID文件: $old_pid"
    fi

    # 4. 等待所有进程完全停止
    sleep 3

    # 5. 验证停止效果
    log_info "🔍 验证停止效果..."
    local remaining_processes=$(ps aux | grep -E "(ros2.*xlerobot|voice_assistant|asr_bridge_node|llm_service_node|tts_service_node|voice_assistant_coordinator)" | grep -v grep | wc -l)
    local remaining_nodes=0

    if command -v ros2 &> /dev/null; then
        remaining_nodes=$(ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" | wc -l)
    fi

    if [ "$remaining_processes" -eq 0 ] && [ "$remaining_nodes" -eq 0 ]; then
        log_success "✅ XLeRobot服务已完全停止"
        log_info "📊 停止统计: 处理了 $process_count 个进程"
    else
        log_warning "⚠️ 发现 $remaining_processes 个残留进程和 $remaining_nodes 个残留节点"

        # 显示残留进程
        if [ "$remaining_processes" -gt 0 ]; then
            log_warning "残留进程:"
            ps aux | grep -E "(ros2.*xlerobot|voice_assistant|asr_bridge_node|llm_service_node|tts_service_node|voice_assistant_coordinator)" | grep -v grep | while read line; do
                local pid=$(echo "$line" | awk '{print $2}')
                local cmd=$(echo "$line" | awk '{print $11,$12,$13}' | sed 's/[[:space:]]*$//')
                echo -e "  ❌ PID $pid: $cmd"
            done
        fi

        # 显示残留节点
        if [ "$remaining_nodes" -gt 0 ]; then
            log_warning "残留ROS2节点:"
            ros2 node list 2>/dev/null | grep -E "(asr|llm|tts|voice_assistant)" | while read node; do
                echo -e "  ❌ $node"
            done
        fi

        log_warning "⚠️ 建议手动清理或重启系统"
    fi

    # 6. 清理ROS2临时文件
    cleanup_ros2_temp_files

    # ========== 可选：恢复PulseAudio服务 ==========
    if ! pgrep -x "pulseaudio" > /dev/null; then
        log_info "🔄 恢复PulseAudio服务..."
        pulseaudio --start 2>/dev/null &
        sleep 2
        if pgrep -x "pulseaudio" > /dev/null; then
            log_success "✅ PulseAudio已恢复"
        else
            log_warning "⚠️ PulseAudio恢复失败，可能需要手动启动"
        fi
    else
        log_info "✅ PulseAudio已在运行"
    fi
    # ==============================================
}

# 重启服务
restart_service() {
    log_info "🔄 重启XleRobot Epic 1语音助手..."
    stop_service
    sleep 2
    start_service
}

# 显示日志
show_logs() {
    log_info "📋 XleRobot Epic 1语音助手日志:"
    echo "================================================"

    if [ -f "$LOG_FILE" ]; then
        tail -n 50 "$LOG_FILE"
        echo ""
        echo "================================================"
        log_info "📁 完整日志文件: $LOG_FILE"
        log_info "🔄 实时查看: tail -f '$LOG_FILE'"
    else
        log_warning "⚠️ 日志文件不存在"
    fi
}

# 主函数
main() {
    case "${1:-start}" in
        "start")
            show_banner
            start_service "${2:-}"
            ;;
        "--force")
            show_banner
            start_service "--force"
            ;;
        "status")
            show_banner
            check_status
            ;;
        "stop")
            show_banner
            stop_service
            ;;
        "restart")
            show_banner
            restart_service
            ;;
        "logs")
            show_logs
            ;;
        "check")
            show_banner
            check_environment
            ;;
        "help"|"-h"|"--help")
            show_help
            ;;
        *)
            log_error "未知命令: $1"
            show_help
            exit 1
            ;;
    esac
}

# 执行主函数
main "$@"