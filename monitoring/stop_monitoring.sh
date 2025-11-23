#!/bin/bash

# XleRobot Monitoring System Stop Script
# BMad-Method v6 Brownfield Level 4 - Story 1.8 Work Package 2

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

# 停止Python监控组件
stop_monitoring_components() {
    log_info "Stopping Python monitoring components..."

    components=(
        "hardware_monitor"
        "sla_monitor"
        "multimodal_monitor"
        "intelligent_alerting"
        "health_check"
    )

    for component in "${components[@]}"; do
        pid_file="/tmp/xlerobot-monitoring/${component}.pid"
        if [ -f "$pid_file" ]; then
            pid=$(cat "$pid_file")
            if kill -0 "$pid" 2>/dev/null; then
                log_info "Stopping $component (PID: $pid)..."
                kill "$pid"

                # 等待进程优雅停止
                sleep 5
                if kill -0 "$pid" 2>/dev/null; then
                    log_warning "Force killing $component..."
                    kill -9 "$pid"
                fi

                rm -f "$pid_file"
                log_success "$component stopped"
            else
                log_warning "$component is not running"
                rm -f "$pid_file"
            fi
        else
            log_warning "PID file for $component not found"
        fi
    done
}

# 停止Docker Compose服务
stop_docker_services() {
    log_info "Stopping Docker Compose services..."

    cd "$(dirname "$0")"

    if [ -f "docker-compose.monitoring.yml" ]; then
        docker-compose -f docker-compose.monitoring.yml down
        log_success "Docker Compose services stopped"
    else
        log_warning "docker-compose.monitoring.yml not found"
    fi
}

# 清理临时文件
cleanup_temp_files() {
    log_info "Cleaning up temporary files..."

    temp_dirs=(
        "/tmp/xlerobot-monitoring"
    )

    for dir in "${temp_dirs[@]}"; do
        if [ -d "$dir" ]; then
            rm -rf "$dir"
            log_success "Cleaned up $dir"
        fi
    done
}

# 显示停止信息
show_stop_info() {
    log_info "XleRobot Monitoring System has been stopped."
    echo
    echo "All monitoring components and services have been shut down."
    echo "Log files are preserved in /var/log/xlerobot/"
    echo "To restart the monitoring system, run: ./start_monitoring.sh"
}

# 主函数
main() {
    log_info "Stopping XleRobot Monitoring System..."
    echo

    # 执行停止步骤
    stop_monitoring_components
    stop_docker_services
    cleanup_temp_files
    show_stop_info

    log_success "XleRobot Monitoring System stopped!"
}

# 执行主函数
main "$@"