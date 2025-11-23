#!/bin/bash
# -*- coding: utf-8 -*-

# XleRobot Story 1.4 - 环境验证脚本
# BMad-Method v6 Brownfield Level 4 企业级实现
# Story 1.4: 基础语音合成 (阿里云TTS API集成)
#
# 验证TTS服务运行环境是否配置正确

set -e

# ============================================
# 🛡️ 加载XLeRobot专用环境配置
# ============================================
# 加载环境脚本，确保使用正确的Python环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$SCRIPT_DIR/xlerobot_env.sh" ]]; then
    source "$SCRIPT_DIR/xlerobot_env.sh"
    echo "✅ XLeRobot环境已加载"
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
NC='\033[0m'

# 计数器
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0
WARNING_CHECKS=0

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[PASS]${NC} $1"
    ((PASSED_CHECKS++))
}

log_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
    ((WARNING_CHECKS++))
}

log_error() {
    echo -e "${RED}[FAIL]${NC} $1"
    ((FAILED_CHECKS++))
}

# 检查函数
check_result() {
    ((TOTAL_CHECKS++))
    if [[ $1 -eq 0 ]]; then
        log_success "$2"
    elif [[ $1 -eq 1 ]]; then
        log_warning "$2"
    else
        log_error "$2"
    fi
}

# 检查系统环境
check_system_environment() {
    log_info "检查系统环境..."

    # 检查操作系统
    if [[ -f /etc/os-release ]]; then
        source /etc/os-release
        check_result 0 "操作系统: $PRETTY_NAME"
    else
        check_result 2 "无法确定操作系统版本"
    fi

    # 检查系统架构
    ARCH=$(uname -m)
    if [[ "$ARCH" == "x86_64" ]]; then
        check_result 0 "系统架构: $ARCH (支持)"
    else
        check_result 1 "系统架构: $ARCH (可能存在兼容性问题)"
    fi

    # 检查内存
    TOTAL_MEM=$(free -m | awk 'NR==2{printf "%.0f", $2}')
    if [[ $TOTAL_MEM -ge 4096 ]]; then
        check_result 0 "系统内存: ${TOTAL_MEM}MB (充足)"
    elif [[ $TOTAL_MEM -ge 2048 ]]; then
        check_result 1 "系统内存: ${TOTAL_MEM}MB (建议4GB+)"
    else
        check_result 2 "系统内存: ${TOTAL_MEM}MB (不足，建议4GB+)"
    fi

    # 检查磁盘空间
    AVAILABLE_SPACE=$(df -BG . | awk 'NR==2{print $4}' | sed 's/G//')
    if [[ $AVAILABLE_SPACE -ge 5 ]]; then
        check_result 0 "可用磁盘空间: ${AVAILABLE_SPACE}GB (充足)"
    elif [[ $AVAILABLE_SPACE -ge 2 ]]; then
        check_result 1 "可用磁盘空间: ${AVAILABLE_SPACE}GB (建议5GB+)"
    else
        check_result 2 "可用磁盘空间: ${AVAILABLE_SPACE}GB (不足，需要5GB+)"
    fi
}

# 检查Python环境
check_python_environment() {
    log_info "检查Python环境..."

    # 检查Python3安装
    if command -v $PYTHON_EXECUTABLE &> /dev/null; then
        PYTHON_VERSION=$($PYTHON_EXECUTABLE --version)
        check_result 0 "Python3已安装: $PYTHON_VERSION"

        # 检查Python版本
        if $PYTHON_EXECUTABLE -c "import sys; exit(0 if sys.version_info >= (3, 10) else 1)" 2>/dev/null; then
            check_result 0 "Python版本符合要求 (>=3.10)"
        else
            check_result 2 "Python版本不符合要求 (需要3.10+)"
        fi

        # 检查pip
        if $PYTHON_EXECUTABLE -m pip --version &> /dev/null; then
            PIP_VERSION=$($PYTHON_EXECUTABLE -m pip --version)
            check_result 0 "pip已安装: $PIP_VERSION"
        else
            check_result 2 "pip未安装"
        fi
    else
        check_result 2 "Python3未安装"
    fi
}

# 检查ROS2环境
check_ros2_environment() {
    log_info "检查ROS2环境..."

    # 检查ROS2命令
    if command -v ros2 &> /dev/null; then
        ROS2_VERSION=$(ros2 --version | head -1)
        check_result 0 "ROS2已安装: $ROS2_VERSION"

        # 检查ROS2环境变量
        if [[ -n "$ROS_DISTRO" ]]; then
            check_result 0 "ROS_DISTRO设置正确: $ROS_DISTRO"

            if [[ "$ROS_DISTRO" == "humble" ]]; then
                check_result 0 "ROS2版本为Humble (推荐)"
            else
                check_result 1 "ROS2版本为$ROS_DISTRO (推荐Humble)"
            fi
        else
            check_result 1 "ROS_DISTRO未设置，可能需要source setup.bash"
        fi

        # 检查ROS2包
        if command -v colcon &> /dev/null; then
            check_result 0 "colcon构建工具已安装"
        else
            check_result 2 "colcon构建工具未安装"
        fi
    else
        check_result 2 "ROS2未安装或未添加到PATH"
    fi
}

# 检查Python依赖
check_python_dependencies() {
    log_info "检查Python依赖..."

    PYTHON_CMD="$PYTHON_EXECUTABLE"

    # 核心依赖列表
    declare -A deps=(
        ["numpy"]="数值计算库"
        ["scipy"]="科学计算库"
        ["yaml"]="YAML解析库"
        ["requests"]="HTTP请求库"
        ["rclpy"]="ROS2 Python客户端"
        ["soundfile"]="音频文件处理"
        ["psutil"]="系统监控库"
    )

    for module in "${!deps[@]}"; do
        if $PYTHON_CMD -c "import $module" 2>/dev/null; then
            VERSION=$($PYTHON_CMD -c "import $module; print(getattr($module, '__version__', 'unknown'))" 2>/dev/null || echo "unknown")
            check_result 0 "${deps[$module]} ($module): $VERSION"
        else
            check_result 2 "${deps[$module]} ($module): 未安装"
        fi
    done

    # 检查可选依赖
    log_info "检查可选依赖..."

    if $PYTHON_CMD -c "import librosa" 2>/dev/null; then
        check_result 0 "librosa (音频分析): 已安装"
    else
        check_result 1 "librosa (音频分析): 未安装 (可选)"
    fi
}

# 检查音频系统
check_audio_system() {
    log_info "检查音频系统..."

    # 检查ALSA工具
    if command -v aplay &> /dev/null; then
        check_result 0 "ALSA播放工具 (aplay): 已安装"

        # 检查音频设备
        if aplay -l &> /dev/null; then
            DEVICE_COUNT=$(aplay -l | grep -c "card")
            check_result 0 "音频设备数量: $DEVICE_COUNT"

            # 显示第一个设备
            FIRST_DEVICE=$(aplay -l | head -1)
            check_result 0 "主音频设备: $FIRST_DEVICE"
        else
            check_result 2 "未检测到音频播放设备"
        fi
    else
        check_result 2 "ALSA播放工具 (aplay): 未安装"
    fi

    # 检查音频测试工具
    if command -v speaker-test &> /dev/null; then
        check_result 0 "音频测试工具 (speaker-test): 已安装"
    else
        check_result 1 "音频测试工具 (speaker-test): 未安装"
    fi

    # 检查音频录制工具
    if command -v arecord &> /dev/null; then
        check_result 0 "音频录制工具 (arecord): 已安装"
    else
        check_result 1 "音频录制工具 (arecord): 未安装"
    fi
}

# 检查网络连接
check_network_connectivity() {
    log_info "检查网络连接..."

    # 检查基本网络连接
    if ping -c 1 8.8.8.8 &> /dev/null; then
        check_result 0 "互联网连接: 正常"
    else
        check_result 2 "互联网连接: 异常"
    fi

    # 检查DNS解析
    if nslookup nls-gateway.cn-shanghai.aliyuncs.com &> /dev/null; then
        check_result 0 "DNS解析: 正常"
    else
        check_result 1 "DNS解析: 可能存在问题"
    fi

    # 检查阿里云TTS服务连接
    if curl -s --connect-timeout 5 "https://nls-gateway.cn-shanghai.aliyuncs.com" &> /dev/null; then
        check_result 0 "阿里云TTS服务连接: 可达"
    else
        check_result 2 "阿里云TTS服务连接: 不可达"
    fi
}

# 检查阿里云API和WebSocket连接
check_aliyun_websocket_api() {
    log_info "检查阿里云WebSocket API..."

    # 运行阿里云API验证脚本
    if $PYTHON_EXECUTABLE scripts/check_aliyun_api.py 2>/dev/null; then
        # 解析输出结果
        local output=$($PYTHON_EXECUTABLE scripts/check_aliyun_api.py 2>&1)

        # 检查SDK安装
        if echo "$output" | grep -q "✅ SDK安装: 阿里云NLS SDK: 已安装"; then
            check_result 0 "阿里云NLS SDK: 已安装"
        else
            check_result 2 "阿里云NLS SDK: 未安装或导入失败"
        fi

        # 检查Token生成
        if echo "$output" | grep -q "✅ Token生成: 成功"; then
            check_result 0 "Token生成: 成功"
        else
            check_result 2 "Token生成: 失败"
        fi

        # 检查WebSocket连接
        if echo "$output" | grep -q "✅ WebSocket连接: 可建立"; then
            check_result 0 "WebSocket连接: 可建立"
        else
            check_result 2 "WebSocket连接: 不可建立"
        fi

        # 显示汇总结果
        if echo "$output" | grep -q "🎉 所有阿里云API验证通过"; then
            log_info "阿里云API验证: 全部通过"
        else
            log_warning "阿里云API验证: 部分失败"
        fi
    else
        check_result 2 "阿里云API验证: 脚本执行失败"
    fi
}

# 检查LLM API连接 (新增 - Story 1.6视觉理解集成)
check_llm_api_connection() {
    log_info "检查LLM API连接..."

    # 检查DASHSCOPE_API_KEY是否设置
    if [[ -z "$DASHSCOPE_API_KEY" ]]; then
        check_result 2 "LLM API连接: DASHSCOPE_API_KEY未设置"
        return 1
    fi

    # 创建临时测试脚本
    local test_script="/tmp/llm_api_test_$$.py"
    cat > "$test_script" << 'EOF'
#!/usr/bin/env python3
import os
import sys
import json
import requests
import time

def test_dashscope_api():
    """测试DashScope API连接"""
    api_key = os.getenv('DASHSCOPE_API_KEY')
    if not api_key:
        return 2, "DASHSCOPE_API_KEY未设置"

    try:
        # 测试Qwen-VL-Plus API连接
        headers = {
            'Authorization': f'Bearer {api_key}',
            'Content-Type': 'application/json'
        }

        # 构建测试请求
        test_data = {
            "model": "qwen-vl-plus",
            "input": {
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"text": "测试连接，请简单回复'连接成功'"},
                        ]
                    }
                ]
            },
            "parameters": {
                "max_tokens": 50,
                "temperature": 0.7
            }
        }

        print("🔍 测试DashScope Qwen-VL-Plus API连接...")

        response = requests.post(
            "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation",
            headers=headers,
            json=test_data,
            timeout=10
        )

        if response.status_code == 200:
            result = response.json()
            if 'output' in result and 'choices' in result['output']:
                return 0, f"API连接成功: {result['output']['choices'][0]['message']['content'][:50]}..."
            else:
                return 2, "API响应格式异常"
        else:
            return 2, f"API连接失败: HTTP {response.status_code} - {response.text[:100]}"

    except requests.exceptions.Timeout:
        return 2, "API连接超时"
    except requests.exceptions.RequestException as e:
        return 2, f"API连接异常: {str(e)}"
    except Exception as e:
        return 2, f"测试异常: {str(e)}"

if __name__ == "__main__":
    code, message = test_dashscope_api()
    print(f"代码: {code}, 消息: {message}")
    sys.exit(code)
EOF

    # 执行测试
    source /home/sunrise/xlerobot/config/.env.sprint1 2>/dev/null || true
    if $PYTHON_EXECUTABLE "$test_script"; then
        check_result 0 "LLM API连接: DashScope Qwen-VL-Plus 连接成功"
    else
        check_result 2 "LLM API连接: DashScope Qwen-VL-Plus 连接失败"
    fi

    # 清理临时文件
    rm -f "$test_script"
}

# 检查项目文件
check_project_files() {
    log_info "检查项目文件..."

    # 检查关键目录
    if [[ -d "src" ]]; then
        check_result 0 "源代码目录 (src/): 存在"
    else
        check_result 2 "源代码目录 (src/): 不存在"
    fi

    if [[ -d "config" ]]; then
        check_result 0 "配置目录 (config/): 存在"
    else
        check_result 1 "配置目录 (config/): 不存在"
    fi

    # 检查关键文件
    if [[ -f "src/xlerobot/tts/aliyun_tts_client.py" ]]; then
        check_result 0 "TTS客户端文件: 存在"
    else
        check_result 2 "TTS客户端文件: 不存在"
    fi

    if [[ -f "src/xlerobot/tts/audio_processor.py" ]]; then
        check_result 0 "音频处理器文件: 存在"
    else
        check_result 2 "音频处理器文件: 不存在"
    fi

    if [[ -f "config/tts_config.yaml" ]]; then
        check_result 0 "TTS配置文件: 存在"
    else
        check_result 1 "TTS配置文件: 不存在 (将使用默认配置)"
    fi

    # 检查启动文件
    if [[ -f "launch/tts_service.launch.py" ]]; then
        check_result 0 "TTS启动文件: 存在"
    else
        check_result 2 "TTS启动文件: 不存在"
    fi
}

# 检查环境变量
check_environment_variables() {
    log_info "检查环境变量..."

    # 检查必需的环境变量
    if [[ -n "$ALIBABA_CLOUD_TOKEN" ]]; then
        TOKEN_LENGTH=${#ALIBABA_CLOUD_TOKEN}
        if [[ $TOKEN_LENGTH -gt 10 ]]; then
            check_result 0 "阿里云Token: 已设置 (长度: $TOKEN_LENGTH)"
        else
            check_result 2 "阿里云Token: 设置无效 (长度过短)"
        fi
    else
        check_result 2 "阿里云Token: 未设置 (TTS服务将无法工作)"
    fi

    # 检查LLM API密钥 (新增 - Story 1.6视觉理解集成)
    if [[ -n "$DASHSCOPE_API_KEY" ]]; then
        KEY_LENGTH=${#DASHSCOPE_API_KEY}
        if [[ $KEY_LENGTH -gt 20 ]]; then
            check_result 0 "DashScope API密钥: 已设置 (Qwen-VL-Plus可用)"
        else
            check_result 2 "DashScope API密钥: 设置无效 (长度过短)"
        fi
    else
        check_result 2 "DashScope API密钥: 未设置 (LLM和视觉理解功能将无法工作)"
    fi

    # 检查Qwen API密钥 (兼容性检查)
    if [[ -n "$QWEN_API_KEY" ]]; then
        QWEN_KEY_LENGTH=${#QWEN_API_KEY}
        if [[ $QWEN_KEY_LENGTH -gt 20 ]]; then
            check_result 0 "Qwen API密钥: 已设置 (通用Qwen模型可用)"
        else
            check_result 2 "Qwen API密钥: 设置无效 (长度过短)"
        fi
    else
        check_result 1 "Qwen API密钥: 未设置 (将使用DashScope密钥)"
    fi

    # 检查可选环境变量
    if [[ -n "$TTS_LOG_LEVEL" ]]; then
        check_result 0 "TTS日志级别: $TTS_LOG_LEVEL"
    else
        check_result 1 "TTS日志级别: 未设置 (将使用默认INFO)"
    fi

    if [[ -n "$PYTHONPATH" ]]; then
        if [[ "$PYTHONPATH" == *"src"* ]]; then
            check_result 0 "PYTHONPATH: 包含src目录"
        else
            check_result 1 "PYTHONPATH: 未包含src目录"
        fi
    else
        check_result 1 "PYTHONPATH: 未设置"
    fi
}

# 检查权限
check_permissions() {
    log_info "检查文件权限..."

    # 检查脚本权限
    if [[ -x "scripts/install_dependencies.sh" ]]; then
        check_result 0 "安装脚本权限: 可执行"
    else
        check_result 1 "安装脚本权限: 不可执行"
    fi

    if [[ -x "scripts/validate_environment.sh" ]]; then
        check_result 0 "验证脚本权限: 可执行"
    else
        check_result 1 "验证脚本权限: 不可执行"
    fi

    # 检查目录写入权限
    if [[ -w "." ]]; then
        check_result 0 "当前目录写入权限: 正常"
    else
        check_result 2 "当前目录写入权限: 不足"
    fi

    # 检查音频设备权限
    if [[ -r "/dev/snd" ]] && [[ -w "/dev/snd" ]]; then
        check_result 0 "音频设备权限: 正常"
    else
        check_result 1 "音频设备权限: 可能不足 (用户可能不在audio组)"
    fi
}

# 运行基础功能测试
run_basic_tests() {
    log_info "运行基础功能测试..."

    # 测试TTS模块导入
    if $PYTHON_EXECUTABLE -c "
import sys
sys.path.insert(0, 'src')
try:
    from xlerobot.tts.aliyun_tts_client import AliyunTTSClient
    from xlerobot.tts.audio_processor import AudioProcessor
    print('✓ TTS模块导入成功')
    exit(0)
except ImportError as e:
    print(f'✗ TTS模块导入失败: {e}')
    exit(1)
" 2>/dev/null; then
        check_result 0 "TTS模块导入测试: 通过"
    else
        check_result 2 "TTS模块导入测试: 失败"
    fi

    # 测试音频处理器
    if $PYTHON_EXECUTABLE -c "
import sys
sys.path.insert(0, 'src')
from xlerobot.tts.audio_processor import AudioProcessor
import numpy as np

try:
    processor = AudioProcessor()
    test_audio = np.random.randint(-32768, 32767, 1000, dtype=np.int16)
    wav_data = processor.convert_to_wav(test_audio)
    if wav_data and len(wav_data) > 44:
        print('✓ 音频处理测试通过')
        exit(0)
    else:
        print('✗ 音频处理测试失败: 无效的WAV数据')
        exit(1)
except Exception as e:
    print(f'✗ 音频处理测试失败: {e}')
    exit(1)
" 2>/dev/null; then
        check_result 0 "音频处理测试: 通过"
    else
        check_result 2 "音频处理测试: 失败"
    fi
}

# 生成验证报告
generate_report() {
    echo
    log_info "环境验证报告"
    echo "=================="
    echo "总检查项: $TOTAL_CHECKS"
    echo -e "通过: ${GREEN}$PASSED_CHECKS${NC}"
    echo -e "警告: ${YELLOW}$WARNING_CHECKS${NC}"
    echo -e "失败: ${RED}$FAILED_CHECKS${NC}"
    echo

    # 计算通过率
    if [[ $TOTAL_CHECKS -gt 0 ]]; then
        PASS_RATE=$((PASSED_CHECKS * 100 / TOTAL_CHECKS))
        echo "通过率: $PASS_RATE%"
    fi

    # 生成建议
    echo
    log_info "建议:"
    if [[ $FAILED_CHECKS -gt 0 ]]; then
        echo "- 请解决所有失败项后再运行TTS服务"
        echo "- 运行 ./scripts/install_dependencies.sh 重新安装依赖"
    fi

    if [[ $WARNING_CHECKS -gt 0 ]]; then
        echo "- 建议解决警告项以获得更好的性能"
    fi

    if [[ $FAILED_CHECKS -eq 0 ]]; then
        echo "- 环境验证通过，可以启动TTS服务"
        echo "- 运行: source install/setup.bash && ros2 launch xlerobot tts_service.launch.py"
    fi

    echo
}

# 主函数
main() {
    echo "XleRobot TTS环境验证"
    echo "===================="
    echo

    # 执行所有检查
    check_system_environment
    check_python_environment
    check_ros2_environment
    check_python_dependencies
    check_audio_system
    check_network_connectivity
    check_aliyun_websocket_api
    check_llm_api_connection
    check_project_files
    check_environment_variables
    check_permissions
    run_basic_tests

    # 生成报告
    generate_report

    # 返回适当的退出码
    if [[ $FAILED_CHECKS -gt 0 ]]; then
        exit 2
    elif [[ $WARNING_CHECKS -gt 0 ]]; then
        exit 1
    else
        exit 0
    fi
}

# 运行主函数
main "$@"