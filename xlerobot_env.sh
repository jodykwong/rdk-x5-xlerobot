#!/bin/bash
# XLeRobot 专用环境配置脚本
# 用途：设置正确的Python环境，解决Miniconda PATH冲突问题
# 版本：v1.0
# 创建日期：2025-11-15

set -e  # 遇到错误立即退出

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

# 检查是否在正确的目录
check_project_directory() {
    if [[ ! -f "start_voice_assistant.sh" ]] && [[ ! -d "src" ]]; then
        log_error "请在XLeRobot项目根目录下运行此脚本！"
        return 1
    fi
    return 0
}

# 清理PATH中的conda/miniconda路径
clean_conda_paths() {
    log_info "清理PATH中的conda/miniconda路径..."

    # 移除所有miniconda/conda相关路径
    local new_path=""
    IFS=':' read -ra ADDR <<< "$PATH"
    for i in "${ADDR[@]}"; do
        if [[ "$i" != *"miniconda"* ]] && [[ "$i" != *"conda"* ]]; then
            if [[ -z "$new_path" ]]; then
                new_path="$i"
            else
                new_path="$new_path:$i"
            fi
        else
            log_warning "移除conda路径: $i"
        fi
    done
    export PATH="$new_path"

    # 🔧 增强：确保系统路径在最高优先级，避免后续被conda覆盖
    export PATH="/usr/bin:/usr/local/bin:$PATH"

    # 🔧 增强：取消可能导致conda激活的变量
    unset CONDA_DEFAULT_ENV 2>/dev/null || true
    unset CONDA_PREFIX 2>/dev/null || true
    unset CONDA_EXE 2>/dev/null || true
    unset CONDA_PYTHON_EXE 2>/dev/null || true

    log_success "PATH已清理完成，系统路径优先级已设置"
}

# 设置Python环境
setup_python_environment() {
    log_info "设置Python 3.10环境..."

    # 检查系统Python 3.10是否存在
    if [[ ! -x "/usr/bin/python3.10" ]]; then
        log_error "系统Python 3.10未找到！请确保安装了python3.10"
        return 1
    fi

    # 设置Python可执行文件
    export PYTHON_EXECUTABLE="/usr/bin/python3.10"

    # 验证Python版本
    local python_version=$($PYTHON_EXECUTABLE --version 2>&1)
    log_success "使用Python版本: $python_version"

    # 确保系统Python路径优先
    export PATH="/usr/bin:$PATH"

    # 检查当前使用的python
    local current_python=$(which python3 2>/dev/null || echo "未找到")
    if [[ "$current_python" == *"miniconda"* ]] || [[ "$current_python" == *"conda"* ]]; then
        log_error "检测到conda/miniconda仍在使用中！当前python: $current_python"
        return 1
    fi

    log_success "Python环境设置完成，当前python3: $(which python3 2>/dev/null || echo '未配置别名')"
}

# 设置ROS2环境
setup_ros2_environment() {
    log_info "设置ROS2 Humble环境..."

    # 检查ROS2安装
    if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
        log_error "ROS2 Humble未找到！请确保已正确安装ROS2 Humble"
        return 1
    fi

    # 加载ROS2环境
    source /opt/ros/humble/setup.bash

    # 设置ROS2环境变量
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
    export RCUTILS_COLORIZED_OUTPUT=1

    log_success "ROS2环境设置完成"
}

# 设置项目路径
setup_project_paths() {
    log_info "设置XLeRobot项目路径..."

    # 获取项目根目录
    if [[ -f "start_voice_assistant.sh" ]]; then
        export XLEROBOT_ROOT="$(pwd)"
    elif [[ -f "../start_voice_assistant.sh" ]]; then
        export XLEROBOT_ROOT="$(pwd)/.."
    else
        log_warning "无法自动检测XLeBot根目录，请手动设置XLEROBOT_ROOT"
        export XLEROBOT_ROOT="/home/sunrise/xlerobot"
    fi

    # 初始化PYTHONPATH
    export PYTHONPATH="$XLEROBOT_ROOT/src:$PYTHONPATH"

    # 🔧 关键修复：添加ROS2 install路径到PYTHONPATH
    # 这些路径包含编译后的ROS2 Python消息模块
    if [[ -d "$XLEROBOT_ROOT/install/audio_msg/local/lib/python3.10/dist-packages" ]]; then
        export PYTHONPATH="$XLEROBOT_ROOT/install/audio_msg/local/lib/python3.10/dist-packages:$PYTHONPATH"
        log_info "已添加audio_msg install路径"
    fi
    if [[ -d "$XLEROBOT_ROOT/install/xlerobot/local/lib/python3.10/dist-packages" ]]; then
        export PYTHONPATH="$XLEROBOT_ROOT/install/xlerobot/local/lib/python3.10/dist-packages:$PYTHONPATH"
        log_info "已添加xlerobot install路径"
    fi

    # 添加Hobot库路径（如果存在）
    if [[ -d "/opt/hobot/lib/python3.10/site-packages" ]]; then
        export PYTHONPATH="/opt/hobot/lib/python3.10/site-packages:$PYTHONPATH"
    fi

    log_success "项目路径设置完成"
    log_info "XLEROBOT_ROOT: $XLEROBOT_ROOT"
    log_info "PYTHONPATH: $PYTHONPATH"
}

# 加载.env环境变量文件
load_env_file() {
    log_info "加载环境变量配置文件..."

    # 获取项目根目录
    if [[ -f "start_voice_assistant.sh" ]]; then
        local env_file="$(pwd)/.env"
    elif [[ -f "../start_voice_assistant.sh" ]]; then
        local env_file="$(pwd)/../.env"
    else
        local env_file="/home/sunrise/xlerobot/.env"
    fi

    # 检查.env文件是否存在
    if [[ -f "$env_file" ]]; then
        log_info "发现环境变量文件: $env_file"

        # 加载.env文件（忽略注释和空行）
        while IFS= read -r line; do
            # 跳过注释行和空行
            [[ $line =~ ^[[:space:]]*# ]] && continue
            [[ -z "${line// }" ]] && continue

            # 提取变量名和值
            if [[ $line =~ ^[A-Z_][A-Z0-9_]*= ]]; then
                export "$line"
                log_info "已设置环境变量: ${line%%=*}"
            fi
        done < "$env_file"

        log_success "环境变量文件加载完成"
    else
        log_warning "未找到.env文件: $env_file"
        log_warning "阿里云API功能将不可用"
    fi
}

# 检测conda环境冲突
detect_conda_conflict() {
    log_info "检测conda环境冲突..."

    # 检查PATH中的conda
    if echo "$PATH" | grep -q "conda\|miniconda"; then
        log_error "PATH中仍包含conda/miniconda路径！"
        log_info "当前PATH: $PATH"
        return 1
    fi

    # 🔧 增强：检查可能导致conda激活的环境变量
    local conda_vars=("CONDA_DEFAULT_ENV" "CONDA_PREFIX" "CONDA_EXE" "CONDA_PYTHON_EXE")
    for var in "${conda_vars[@]}"; do
        if [[ -n "${!var}" ]]; then
            log_error "检测到conda环境变量 $var 仍存在: ${!var}"
            return 1
        fi
    done

    # 检查Python可执行文件
    local current_python=$(which python 2>/dev/null || echo "未找到")
    local current_python3=$(which python3 2>/dev/null || echo "未找到")

    if [[ "$current_python" == *"conda"* ]] || [[ "$current_python" == *"miniconda"* ]]; then
        log_error "检测到conda Python仍在使用: $current_python"
        return 1
    fi

    if [[ "$current_python3" == *"conda"* ]] || [[ "$current_python3" == *"miniconda"* ]]; then
        log_error "检测到conda python3仍在使用: $current_python3"
        return 1
    fi

    # 🔧 增强：验证实际使用的Python版本
    if command -v python3.10 &> /dev/null; then
        local actual_version=$(python3.10 --version 2>&1 | grep -o '3\.10\.[0-9]*')
        if [[ "$actual_version" == "3.10"* ]]; then
            log_success "Python 3.10验证通过: $actual_version"
        else
            log_error "Python版本验证失败: 期望3.10.x，实际$actual_version"
            return 1
        fi
    else
        log_error "python3.10不可用"
        return 1
    fi

    log_success "未检测到conda环境冲突"
    return 0
}

# 显示环境摘要
show_environment_summary() {
    log_info "=== XLeRobot环境配置摘要 ==="
    echo ""
    echo "Python环境:"
    echo "  - 可执行文件: $PYTHON_EXECUTABLE"
    echo "  - 版本: $($PYTHON_EXECUTABLE --version 2>&1)"
    echo "  - 当前python3: $(which python3 2>/dev/null || echo '未配置')"
    echo ""
    echo "ROS2环境:"
    echo "  - ROS_DISTRO: ${ROS_DISTRO:-未设置}"
    echo "  - ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    echo "  - RMW_IMPLEMENTATION: $RMW_IMPLEMENTATION"
    echo ""
    echo "项目路径:"
    echo "  - XLEROBOT_ROOT: $XLEROBOT_ROOT"
    echo "  - PYTHONPATH: $PYTHONPATH"
    echo ""
    log_success "环境配置完成！可以安全运行XLeRobot项目。"
}

# 主函数
main() {
    log_info "开始配置XLeRobot专用环境..."

    # 检查项目目录
    check_project_directory || return 1

    # 清理conda路径
    clean_conda_paths

    # 设置Python环境
    setup_python_environment || return 1

    # 设置ROS2环境
    setup_ros2_environment || return 1

    # 设置项目路径
    setup_project_paths

    # 加载环境变量文件
    load_env_file

    # 检测conda冲突
    if detect_conda_conflict; then
        log_success "所有检查通过，环境配置成功！"
    else
        log_error "环境冲突检测失败，请检查配置"
        return 1
    fi

    # 显示摘要
    show_environment_summary

    # 导出关键函数到当前shell（如果需要）
    export PYTHON_EXECUTABLE
    export XLEROBOT_ROOT
}

# 如果直接运行此脚本，执行main函数
# 如果直接运行此脚本，执行main函数
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
else
    # 被source时，直接执行环境设置（不使用函数，确保变量导出到当前shell）
    # 检查项目目录
    if [[ ! -f "start_voice_assistant.sh" ]] && [[ ! -d "src" ]]; then
        echo "❌ 请在XLeRobot项目根目录下运行此脚本！" 1>&2
        return 1
    fi

    # 清理conda路径
    echo "🛡️ 清理conda/miniconda路径..."
    new_path=""
    IFS=':' read -ra ADDR <<< "$PATH"
    for i in "${ADDR[@]}"; do
        if [[ "$i" != *"miniconda"* ]] && [[ "$i" != *"conda"* ]]; then
            if [[ -z "$new_path" ]]; then
                new_path="$i"
            else
                new_path="$new_path:$i"
            fi
        fi
    done
    export PATH="$new_path"

    # 设置Python环境
    if [[ ! -x "/usr/bin/python3.10" ]]; then
        echo "❌ 系统Python 3.10未找到！" 1>&2
        return 1
    fi
    export PYTHON_EXECUTABLE="/usr/bin/python3.10"

    # 🔧 增强：彻底清理conda环境变量
    unset CONDA_DEFAULT_ENV 2>/dev/null || true
    unset CONDA_PREFIX 2>/dev/null || true
    unset CONDA_EXE 2>/dev/null || true
    unset CONDA_PYTHON_EXE 2>/dev/null || true

    # 确保系统路径在最高优先级
    export PATH="/usr/bin:/usr/local/bin:$PATH"

    # 设置ROS2环境
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        source /opt/ros/humble/setup.bash
    fi
    export ROS_DOMAIN_ID=42
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export RCUTILS_LOGGING_BUFFERED_STREAM=1
    export RCUTILS_COLORIZED_OUTPUT=1

    # 设置项目路径
    export XLEROBOT_ROOT="$(pwd)"
    export PYTHONPATH="$XLEROBOT_ROOT/src:$PYTHONPATH"

    # 🔧 关键修复：添加ROS2 install路径到PYTHONPATH
    # 这些路径包含编译后的ROS2 Python消息模块
    if [[ -d "$XLEROBOT_ROOT/install/audio_msg/local/lib/python3.10/dist-packages" ]]; then
        export PYTHONPATH="$XLEROBOT_ROOT/install/audio_msg/local/lib/python3.10/dist-packages:$PYTHONPATH"
        echo "  ✅ 已添加audio_msg install路径"
    fi
    if [[ -d "$XLEROBOT_ROOT/install/xlerobot/local/lib/python3.10/dist-packages" ]]; then
        export PYTHONPATH="$XLEROBOT_ROOT/install/xlerobot/local/lib/python3.10/dist-packages:$PYTHONPATH"
        echo "  ✅ 已添加xlerobot install路径"
    fi

    if [[ -d "/opt/hobot/lib/python3.10/site-packages" ]]; then
        export PYTHONPATH="/opt/hobot/lib/python3.10/site-packages:$PYTHONPATH"
    fi

    # 加载.env环境变量文件
    env_file="$XLEROBOT_ROOT/.env"
    if [[ -f "$env_file" ]]; then
        echo "🔑 加载环境变量配置文件..."
        # 加载.env文件（忽略注释和空行）
        while IFS= read -r line; do
            # 跳过注释行和空行
            [[ $line =~ ^[[:space:]]*# ]] && continue
            [[ -z "${line// }" ]] && continue

            # 提取变量名和值
            if [[ $line =~ ^[A-Z_][A-Z0-9_]*= ]]; then
                export "$line"
                echo "  ✅ 设置: ${line%%=*}"
            fi
        done < "$env_file"
    else
        echo "⚠️ 未找到.env文件: $env_file"
        echo "⚠️ 阿里云API功能将不可用"
    fi

    echo "✅ XLeRobot环境已配置完成"
fi