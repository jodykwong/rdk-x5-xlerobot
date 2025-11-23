#!/bin/bash
# XLeRobot 环境验证脚本
# 用途：快速检查XLeRobot环境配置是否正确
# 创建日期：2025-11-15

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m' # No Color

# 计数器
ERRORS=0
WARNINGS=0
SUCCESS=0

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[✓]${NC} $1"
    ((SUCCESS++))
}

log_warning() {
    echo -e "${YELLOW}[⚠]${NC} $1"
    ((WARNINGS++))
}

log_error() {
    echo -e "${RED}[✗]${NC} $1"
    ((ERRORS++))
}

# 显示横幅
show_banner() {
    echo -e "${PURPLE}"
    echo "🤖 XLeRobot 环境验证工具"
    echo "=========================="
    echo -e "${NC}"
}

# 检查Python环境
check_python_environment() {
    echo -e "${BLUE}🐍 检查Python环境${NC}"
    echo "--------------------"

    # 检查系统Python 3.10
    if [[ -x "/usr/bin/python3.10" ]]; then
        local python_version=$(/usr/bin/python3.10 --version 2>&1)
        log_success "系统Python 3.10: $python_version"
    else
        log_error "系统Python 3.10未找到"
        return 1
    fi

    # 检查PYTHON_EXECUTABLE环境变量
    if [[ -n "$PYTHON_EXECUTABLE" ]]; then
        log_success "PYTHON_EXECUTABLE已设置: $PYTHON_EXECUTABLE"

        # 验证PYTHON_EXECUTABLE是否可执行
        if [[ -x "$PYTHON_EXECUTABLE" ]]; then
            local env_python_version=$($PYTHON_EXECUTABLE --version 2>&1)
            log_success "环境Python版本: $env_python_version"
        else
            log_error "PYTHON_EXECUTABLE不可执行: $PYTHON_EXECUTABLE"
        fi
    else
        log_warning "PYTHON_EXECUTABLE未设置"
    fi

    # 检查当前python3命令
    local current_python3=$(which python3 2>/dev/null || echo "未配置")
    if [[ "$current_python3" == *"conda"* ]] || [[ "$current_python3" == *"miniconda"* ]]; then
        log_error "检测到conda Python冲突: $current_python3"
        log_error "XLeRobot必须使用系统Python 3.10"
    else
        log_success "python3路径: $current_python3"
    fi

    # 测试Python基本功能
    if command -v python3.10 &> /dev/null; then
        if python3.10 -c "import sys; print('Python基础功能正常')" &>/dev/null; then
            log_success "Python基础功能正常"
        else
            log_error "Python基础功能异常"
        fi
    fi

    echo ""
}

# 检查PATH冲突
check_path_conflicts() {
    echo -e "${BLUE}🛡️  检查PATH冲突${NC}"
    echo "--------------------"

    # 检查PATH中的conda/miniconda路径
    if echo "$PATH" | grep -q "conda\|miniconda"; then
        log_warning "PATH中发现conda/miniconda路径"
        echo "发现的问题路径:"
        echo "$PATH" | tr ':' '\n' | grep -E "conda|miniconda" | sed 's/^/  - /'
        echo ""
        echo "建议运行: source ./xlerobot_env.sh"
    else
        log_success "PATH中无conda/miniconda路径"
    fi

    # 检查系统工具路径优先级
    if echo "$PATH" | grep -q "^/usr/bin:"; then
        log_success "/usr/bin路径优先级正确"
    else
        log_warning "/usr/bin路径可能不是最高优先级"
    fi

    echo ""
}

# 检查conda环境
check_conda_environment() {
    echo -e "${BLUE}🔥 检查conda环境${NC}"
    echo "--------------------"

    # 检查CONDA_DEFAULT_ENV
    if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
        log_error "检测到活跃conda环境: $CONDA_DEFAULT_ENV"
        log_error "XLeRobot不能在conda环境中运行"
        echo "解决方案: conda deactivate"
    else
        log_success "未检测到活跃的conda环境"
    fi

    # 检查conda命令是否可用
    if command -v conda &> /dev/null; then
        local conda_path=$(which conda 2>/dev/null)
        log_warning "conda命令可用: $conda_path"
        echo "  警告：确保conda不会干扰XLeRobot环境"
    else
        log_success "conda命令不在PATH中"
    fi

    echo ""
}

# 检查ROS2环境
check_ros2_environment() {
    echo -e "${BLUE}🤖 检查ROS2环境${NC}"
    echo "--------------------"

    # 检查ROS2安装
    if [[ -f "/opt/ros/humble/setup.bash" ]]; then
        log_success "ROS2 Humble已安装"
    else
        log_error "ROS2 Humble未找到"
        return 1
    fi

    # 检查ROS_DISTRO环境变量
    if [[ "$ROS_DISTRO" == "humble" ]]; then
        log_success "ROS_DISTRO: $ROS_DISTRO"
    else
        log_warning "ROS_DISTRO未正确设置: ${ROS_DISTRO:-未设置}"
    fi

    # 检查ROS_DOMAIN_ID
    if [[ "$ROS_DOMAIN_ID" == "42" ]]; then
        log_success "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
    else
        log_warning "ROS_DOMAIN_ID可能未设置: ${ROS_DOMAIN_ID:-未设置}"
    fi

    # 检查ros2命令
    if command -v ros2 &> /dev/null; then
        local ros2_version=$(ros2 --version 2>/dev/null || echo "版本获取失败")
        log_success "ros2命令可用: $ros2_version"
    else
        log_warning "ros2命令不可用"
    fi

    echo ""
}

# 检查项目结构
check_project_structure() {
    echo -e "${BLUE}📁 检查项目结构${NC}"
    echo "--------------------"

    # 检查关键文件和目录
    local critical_files=(
        "xlerobot_env.sh"
        "start_voice_assistant.sh"
        "CLAUDE.md"
        "README.md"
    )

    local critical_dirs=(
        "src"
        "src/modules"
        "src/modules/asr"
        "src/modules/tts"
        "src/modules/llm"
        "tests"
        "config"
    )

    for file in "${critical_files[@]}"; do
        if [[ -f "$file" ]]; then
            log_success "文件: $file"
        else
            log_error "文件缺失: $file"
        fi
    done

    for dir in "${critical_dirs[@]}"; do
        if [[ -d "$dir" ]]; then
            log_success "目录: $dir"
        else
            log_error "目录缺失: $dir"
        fi
    done

    echo ""
}

# 检查环境变量
check_environment_variables() {
    echo -e "${BLUE}🔧 检查环境变量${NC}"
    echo "--------------------"

    local env_vars=(
        "PYTHONPATH"
        "XLEROBOT_ROOT"
        "ROS_DISTRO"
        "ROS_DOMAIN_ID"
        "PYTHON_EXECUTABLE"
    )

    for var in "${env_vars[@]}"; do
        if [[ -n "${!var}" ]]; then
            log_success "$var: ${!var}"
        else
            log_warning "$var: 未设置"
        fi
    done

    echo ""
}

# 运行环境脚本测试
test_environment_script() {
    echo -e "${BLUE}🧪 测试环境脚本${NC}"
    echo "--------------------"

    if [[ -f "xlerobot_env.sh" ]]; then
        log_success "xlerobot_env.sh存在"

        # 测试脚本语法
        if bash -n xlerobot_env.sh 2>/dev/null; then
            log_success "xlerobot_env.sh语法正确"
        else
            log_error "xlerobot_env.sh语法错误"
        fi

        # 测试脚本可执行性
        if [[ -x "xlerobot_env.sh" ]]; then
            log_success "xlerobot_env.sh可执行"
        else
            log_warning "xlerobot_env.sh不可执行，运行: chmod +x xlerobot_env.sh"
        fi
    else
        log_error "xlerobot_env.sh不存在"
    fi

    echo ""
}

# 显示验证结果
show_verification_results() {
    echo -e "${PURPLE}"
    echo "🎯 环境验证结果摘要"
    echo "=================="
    echo -e "${NC}"

    echo -e "${GREEN}✓ 成功: $SUCCESS${NC}"
    echo -e "${YELLOW}⚠ 警告: $WARNINGS${NC}"
    echo -e "${RED}✗ 错误: $ERRORS${NC}"
    echo ""

    if [[ $ERRORS -eq 0 ]]; then
        if [[ $WARNINGS -eq 0 ]]; then
            echo -e "${GREEN}🎉 环境验证完全通过！XLeRobot环境配置正确。${NC}"
            return 0
        else
            echo -e "${YELLOW}⚡ 环境基本正常，但有 $WARNINGS 个警告需要注意。${NC}"
            return 0
        fi
    else
        echo -e "${RED}❌ 发现 $ERRORS 个错误，请修复后再使用XLeRobot。${NC}"
        echo ""
        echo "常见解决方案："
        echo "1. 运行: source ./xlerobot_env.sh"
        echo "2. 重新启动shell: exec bash"
        echo "3. 确保不在conda环境中运行"
        return 1
    fi
}

# 显示帮助信息
show_help() {
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help     显示帮助信息"
    echo "  -q, --quiet    静默模式，只显示结果"
    echo "  -v, --verbose  详细模式，显示更多信息"
    echo ""
    echo "示例:"
    echo "  $0                    # 正常验证"
    echo "  $0 --quiet            # 静默验证"
    echo "  $0 --verbose          # 详细验证"
}

# 主函数
main() {
    local quiet_mode=false
    local verbose_mode=false

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -q|--quiet)
                quiet_mode=true
                shift
                ;;
            -v|--verbose)
                verbose_mode=true
                shift
                ;;
            *)
                echo "未知选项: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # 如果不是静默模式，显示横幅
    if [[ "$quiet_mode" == false ]]; then
        show_banner
    fi

    # 检查是否在正确的目录
    if [[ ! -f "xlerobot_env.sh" ]]; then
        log_error "请在XLeRobot项目根目录中运行此脚本"
        exit 1
    fi

    # 执行所有检查
    check_python_environment
    check_path_conflicts
    check_conda_environment
    check_ros2_environment
    check_project_structure
    check_environment_variables
    test_environment_script

    # 显示结果
    show_verification_results
}

# 如果直接运行此脚本，执行main函数
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi