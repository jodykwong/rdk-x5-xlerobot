#!/bin/bash
# Epic 1 质量保证全面测试 - 安全执行脚本
# ==========================================
#
# 基于根因分析设计的安全测试执行脚本
# 包含超时保护、进程监控和资源管理
#
# 使用方法:
#   ./run_epic1_tests.sh          # 运行所有测试
#   ./run_epic1_tests.sh audio    # 只运行音频测试
#   ./run_epic1_tests.sh api      # 只运行API测试
#   ./run_epic1_tests.sh e2e      # 只运行端到端测试
#
# 作者: Test Safety Agent
# 创建时间: 2025-11-12
# 版本: v1.0 - 安全执行版

set -e  # 遇到错误立即退出

# ============================================
# 🛡️ 加载XLeRobot专用环境配置
# ============================================
# 加载环境脚本，确保使用正确的Python环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
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
NC='\033[0m' # No Color

# 配置
PROJECT_ROOT="/home/sunrise/xlerobot"
TESTS_DIR="$PROJECT_ROOT/tests"
LOG_DIR="$PROJECT_ROOT/logs"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="$LOG_DIR/epic1_test_$TIMESTAMP.log"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a "$LOG_FILE"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a "$LOG_FILE"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a "$LOG_FILE"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a "$LOG_FILE"
}

# 显示帮助信息
show_help() {
    cat << EOF
Epic 1 质量保证全面测试 - 安全执行脚本

用法: $0 [选项] [测试类型]

选项:
    -h, --help      显示此帮助信息
    -v, --verbose   详细输出模式
    -q, --quiet     静默模式
    -f, --force     强制执行（跳过安全检查）

测试类型:
    audio           只运行音频组件测试
    api             只运行阿里云API集成测试
    e2e             只运行端到端集成测试
    (无参数)        运行所有测试

示例:
    $0              # 运行所有测试
    $0 audio        # 只运行音频测试
    $0 -v api       # 详细模式运行API测试

EOF
}

# 安全检查
safety_check() {
    log_info "🛡️ 执行安全检查..."

    # 检查Python环境
    if ! command -v python3 &> /dev/null; then
        log_error "❌ Python3 未安装"
        exit 1
    fi

    # 检查项目目录
    if [ ! -d "$PROJECT_ROOT" ]; then
        log_error "❌ 项目目录不存在: $PROJECT_ROOT"
        exit 1
    fi

    # 检查测试目录
    if [ ! -d "$TESTS_DIR" ]; then
        log_error "❌ 测试目录不存在: $TESTS_DIR"
        exit 1
    fi

    # 检查测试脚本
    local required_scripts=(
        "test_audio_components.py"
        "test_aliyun_api_integration.py"
        "test_e2e_integration.py"
        "test_runner.py"
    )

    for script in "${required_scripts[@]}"; do
        if [ ! -f "$TESTS_DIR/$script" ]; then
            log_error "❌ 测试脚本不存在: $script"
            exit 1
        fi
    done

    # 检查系统资源
    local available_memory=$(free -m | awk 'NR==2{printf "%.0f", $7}')
    if [ "$available_memory" -lt 1000 ]; then
        log_warning "⚠️ 可用内存较少: ${available_memory}MB，建议释放内存"
    fi

    # 检查磁盘空间
    local available_disk=$(df -m "$PROJECT_ROOT" | awk 'NR==2{print $4}')
    if [ "$available_disk" -lt 1000 ]; then
        log_warning "⚠️ 磁盘空间较少: ${available_disk}MB，建议清理空间"
    fi

    log_success "✅ 安全检查通过"
}

# 清理函数
cleanup() {
    log_info "🧹 执行清理操作..."

    # 终止可能的测试进程
    pkill -f "test_.*\.py" 2>/dev/null || true
    pkill -f "test_runner\.py" 2>/dev/null || true

    # 清理临时文件
    find /tmp -name "*test_*.wav" -type f -mmin +60 -delete 2>/dev/null || true
    find /tmp -name "*tmp*.wav" -type f -mmin +60 -delete 2>/dev/null || true
    find /tmp -name "*tts_*.wav" -type f -mmin +60 -delete 2>/dev/null || true

    log_success "✅ 清理完成"
}

# 信号处理
trap cleanup EXIT INT TERM

# 运行特定测试
run_test() {
    local test_type=$1
    local test_script=""

    case $test_type in
        "audio")
            test_script="$TESTS_DIR/test_audio_components.py"
            ;;
        "api")
            test_script="$TESTS_DIR/test_aliyun_api_integration.py"
            ;;
        "e2e")
            test_script="$TESTS_DIR/test_e2e_integration.py"
            ;;
        *)
            log_error "❌ 未知的测试类型: $test_type"
            return 1
            ;;
    esac

    log_info "🚀 运行测试: $test_type"
    log_info "📄 脚本: $test_script"

    # 设置超时保护
    timeout 300 $PYTHON_EXECUTABLE "$test_script" 2>&1 | tee -a "$LOG_FILE"
    local exit_code=${PIPESTATUS[0]}

    if [ $exit_code -eq 0 ]; then
        log_success "✅ 测试通过: $test_type"
        return 0
    elif [ $exit_code -eq 124 ]; then
        log_error "❌ 测试超时: $test_type (5分钟)"
        return 1
    else
        log_error "❌ 测试失败: $test_type (退出码: $exit_code)"
        return 1
    fi
}

# 运行所有测试
run_all_tests() {
    log_info "🚀 运行所有测试 (使用安全执行器)"

    cd "$PROJECT_ROOT"

    # 使用安全测试执行器
    timeout 600 $PYTHON_EXECUTABLE "$TESTS_DIR/test_runner.py" 2>&1 | tee -a "$LOG_FILE"
    local exit_code=${PIPESTATUS[0]}

    if [ $exit_code -eq 0 ]; then
        log_success "✅ 所有测试完成"
        return 0
    elif [ $exit_code -eq 124 ]; then
        log_error "❌ 测试执行超时 (10分钟)"
        return 1
    else
        log_error "❌ 测试执行失败 (退出码: $exit_code)"
        return 1
    fi
}

# 显示测试报告
show_report() {
    if [ -f "$LOG_FILE" ]; then
        log_info "📊 测试报告已保存到: $LOG_FILE"

        # 提取关键信息
        echo ""
        log_info "📋 测试摘要:"
        grep -E "(✅|❌|🎉|💥)" "$LOG_FILE" | tail -10

        # 显示最终结果
        if grep -q "🎉.*测试执行总体成功" "$LOG_FILE"; then
            log_success "🎉 Epic 1 测试总体通过！"
            return 0
        else
            log_error "❌ Epic 1 测试存在问题"
            return 1
        fi
    else
        log_warning "⚠️ 未找到日志文件"
        return 1
    fi
}

# 主函数
main() {
    local test_type=""
    local verbose=false
    local force=false

    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            -q|--quiet)
                set +x
                shift
                ;;
            -f|--force)
                force=true
                shift
                ;;
            audio|api|e2e)
                test_type="$1"
                shift
                ;;
            *)
                log_error "❌ 未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # 显示头部信息
    echo ""
    echo "=============================================="
    echo "🛡️  Epic 1 质量保证全面测试 - 安全执行"
    echo "=============================================="
    echo "📅 执行时间: $(date)"
    echo "📁 项目目录: $PROJECT_ROOT"
    echo "📝 日志文件: $LOG_FILE"
    echo "=============================================="
    echo ""

    # 安全检查（除非强制执行）
    if [ "$force" = false ]; then
        safety_check
    else
        log_warning "⚠️ 跳过安全检查 (强制模式)"
    fi

    local start_time=$(date +%s)
    local exit_code=0

    # 执行测试
    if [ -n "$test_type" ]; then
        # 运行特定测试
        run_test "$test_type"
        exit_code=$?
    else
        # 运行所有测试
        run_all_tests
        exit_code=$?
    fi

    local end_time=$(date +%s)
    local duration=$((end_time - start_time))

    echo ""
    echo "=============================================="
    log_info "⏱️  总执行时间: ${duration} 秒"

    # 显示报告
    if [ $exit_code -eq 0 ]; then
        show_report
    else
        log_error "❌ 测试执行失败"
    fi

    echo "=============================================="

    exit $exit_code
}

# 执行主函数
main "$@"