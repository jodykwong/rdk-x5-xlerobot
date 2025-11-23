#!/bin/bash
# XleRobot Epic 1 自动启动服务脚本
# =====================================
#
# 此脚本用于设置XleRobot Epic 1服务开机自启动
# 使用systemd服务管理器
#
# 作者: Dev Agent
# 创建时间: 2025-11-13

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 配置
PROJECT_ROOT="/home/sunrise/xlerobot"
SERVICE_NAME="xlerobot-epic1"
SERVICE_USER="sunrise"
PYTHON_PATH="/usr/bin/python3.10"
START_SCRIPT="$PROJECT_ROOT/start_voice_assistant.sh"

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

# 显示帮助信息
show_help() {
    cat << EOF
XleRobot Epic 1 自动启动服务管理脚本

用法: $0 [选项] [命令]

选项:
    -h, --help      显示此帮助信息

命令:
    install         安装systemd服务并启用自启动
    uninstall       卸载systemd服务
    start           手动启动服务
    stop            手动停止服务
    restart         重启服务
    status          查看服务状态
    logs            查看服务日志
    enable          启用开机自启动
    disable         禁用开机自启动

示例:
    $0 install      # 安装并启用自启动
    $0 start        # 手动启动服务
    $0 status       # 查看服务状态

EOF
}

# 检查权限
check_permissions() {
    if [ "$EUID" -ne 0 ]; then
        log_error "需要root权限来管理系统服务"
        echo "请使用: sudo $0 $1"
        exit 1
    fi
}

# 检查文件
check_files() {
    if [ ! -f "$START_SCRIPT" ]; then
        log_error "启动脚本不存在: $START_SCRIPT"
        exit 1
    fi

    if [ ! -d "$PROJECT_ROOT" ]; then
        log_error "项目目录不存在: $PROJECT_ROOT"
        exit 1
    fi

    if [ ! -f "$PYTHON_PATH" ]; then
        log_error "Python解释器不存在: $PYTHON_PATH"
        exit 1
    fi
}

# 创建systemd服务文件
create_service_file() {
    log_info "创建systemd服务文件..."

    cat > /etc/systemd/system/$SERVICE_NAME.service << EOF
[Unit]
Description=XleRobot Epic 1 纯在线语音交互服务
Documentation=https://docs.xlerobot.com/epic1
After=network.target sound.target
Wants=network.target

[Service]
Type=simple
User=$SERVICE_USER
Group=$SERVICE_USER
WorkingDirectory=$PROJECT_ROOT
Environment=PYTHONPATH=$PROJECT_ROOT/src
Environment=ROS_DOMAIN_ID=42
# ⚠️ 请在下面设置您的阿里云API密钥
Environment=ALIBABA_CLOUD_ACCESS_KEY_ID=YOUR_ACCESS_KEY_ID
Environment=ALIBABA_CLOUD_ACCESS_KEY_SECRET=YOUR_ACCESS_KEY_SECRET
Environment=ALIYUN_NLS_APPKEY=YOUR_NLS_APPKEY
Environment=QWEN_API_KEY=YOUR_QWEN_API_KEY
ExecStart=/bin/bash $START_SCRIPT
ExecReload=/bin/kill -HUP \$MAINPID
KillMode=mixed
TimeoutStopSec=5
Restart=on-failure
RestartSec=10
StartLimitBurst=3
StartLimitInterval=60

# 日志配置
StandardOutput=journal
StandardError=journal
SyslogIdentifier=$SERVICE_NAME

# 音频设备访问配置
SupplementaryGroups=audio video
NoNewPrivileges=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=$PROJECT_ROOT/logs /tmp /dev

[Install]
WantedBy=multi-user.target
EOF

    log_success "systemd服务文件创建完成"
}

# 安装服务
install_service() {
    log_info "安装XleRobot Epic 1服务..."

    check_permissions "install"
    check_files

    # 创建服务文件
    create_service_file

    # 重新加载systemd
    systemctl daemon-reload
    log_info "systemd配置已重新加载"

    # 启用服务
    systemctl enable $SERVICE_NAME
    log_success "服务已设置为开机自启动"

    # 启动服务
    systemctl start $SERVICE_NAME

    # 等待服务启动
    sleep 3

    # 检查服务状态
    if systemctl is-active --quiet $SERVICE_NAME; then
        log_success "XleRobot Epic 1服务启动成功！"
        log_info "服务状态: $(systemctl is-active $SERVICE_NAME)"
    else
        log_error "服务启动失败，请检查日志"
        show_logs
        exit 1
    fi

    log_success "安装完成！服务将在开机时自动启动"
}

# 卸载服务
uninstall_service() {
    log_info "卸载XleRobot Epic 1服务..."

    check_permissions "uninstall"

    # 停止服务
    if systemctl is-active --quiet $SERVICE_NAME; then
        systemctl stop $SERVICE_NAME
        log_info "服务已停止"
    fi

    # 禁用服务
    systemctl disable $SERVICE_NAME 2>/dev/null || true
    log_info "开机自启动已禁用"

    # 删除服务文件
    if [ -f "/etc/systemd/system/$SERVICE_NAME.service" ]; then
        rm -f "/etc/systemd/system/$SERVICE_NAME.service"
        log_info "systemd服务文件已删除"
    fi

    # 重新加载systemd
    systemctl daemon-reload
    log_info "systemd配置已重新加载"

    log_success "XleRobot Epic 1服务卸载完成"
}

# 启动服务
start_service() {
    log_info "启动XleRobot Epic 1服务..."
    systemctl start $SERVICE_NAME

    if systemctl is-active --quiet $SERVICE_NAME; then
        log_success "服务启动成功"
    else
        log_error "服务启动失败"
        exit 1
    fi
}

# 停止服务
stop_service() {
    log_info "停止XleRobot Epic 1服务..."
    systemctl stop $SERVICE_NAME

    if systemctl is-active --quiet $SERVICE_NAME; then
        log_error "服务停止失败"
        exit 1
    else
        log_success "服务已停止"
    fi
}

# 重启服务
restart_service() {
    log_info "重启XleRobot Epic 1服务..."
    systemctl restart $SERVICE_NAME
    log_success "服务重启完成"
}

# 查看服务状态
show_status() {
    log_info "XleRobot Epic 1服务状态:"
    echo "----------------------------------------"
    systemctl status $SERVICE_NAME --no-pager
    echo "----------------------------------------"

    # 显示简要状态
    if systemctl is-active --quiet $SERVICE_NAME; then
        log_success "运行状态: 活跃"
    else
        log_warning "运行状态: 未运行"
    fi

    if systemctl is-enabled --quiet $SERVICE_NAME; then
        log_success "自启动状态: 已启用"
    else
        log_warning "自启动状态: 已禁用"
    fi
}

# 显示日志
show_logs() {
    log_info "XleRobot Epic 1服务日志:"
    echo "----------------------------------------"
    journalctl -u $SERVICE_NAME -n 50 --no-pager
    echo "----------------------------------------"
    echo "提示: 使用 'journalctl -u $SERVICE_NAME -f' 实时查看日志"
}

# 启用自启动
enable_service() {
    log_info "启用开机自启动..."
    systemctl enable $SERVICE_NAME
    log_success "开机自启动已启用"
}

# 禁用自启动
disable_service() {
    log_info "禁用开机自启动..."
    systemctl disable $SERVICE_NAME
    log_success "开机自启动已禁用"
}

# 主函数
main() {
    case "${1:-help}" in
        "install")
            install_service
            ;;
        "uninstall")
            uninstall_service
            ;;
        "start")
            start_service
            ;;
        "stop")
            stop_service
            ;;
        "restart")
            restart_service
            ;;
        "status")
            show_status
            ;;
        "logs")
            show_logs
            ;;
        "enable")
            enable_service
            ;;
        "disable")
            disable_service
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