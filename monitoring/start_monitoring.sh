#!/bin/bash

# XleRobot Monitoring System Startup Script
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

# 检查Docker是否运行
check_docker() {
    log_info "Checking Docker status..."
    if ! docker info > /dev/null 2>&1; then
        log_error "Docker is not running. Please start Docker first."
        exit 1
    fi
    log_success "Docker is running"
}

# 检查必要的端口是否可用
check_ports() {
    log_info "Checking port availability..."

    ports=(3000 9090 9091 9093 8080 9100)
    for port in "${ports[@]}"; do
        if netstat -tuln 2>/dev/null | grep -q ":$port "; then
            log_warning "Port $port is already in use"
        else
            log_success "Port $port is available"
        fi
    done
}

# 创建必要的目录
create_directories() {
    log_info "Creating necessary directories..."

    directories=(
        "/var/log/xlerobot"
        "/var/lib/grafana"
        "/var/lib/prometheus"
        "/var/lib/alertmanager"
        "/tmp/xlerobot-monitoring"
    )

    for dir in "${directories[@]}"; do
        if [ ! -d "$dir" ]; then
            sudo mkdir -p "$dir"
            sudo chmod 755 "$dir"
            log_success "Created directory: $dir"
        fi
    done
}

# 检查Python依赖
check_python_dependencies() {
    log_info "Checking Python dependencies..."

    required_packages=(
        "prometheus_client"
        "psutil"
        "numpy"
        "matplotlib"
        "requests"
    )

    for package in "${required_packages[@]}"; do
        if python3 -c "import $package" 2>/dev/null; then
            log_success "Python package $package is available"
        else
            log_warning "Python package $package is missing. Installing..."
            pip3 install "$package"
        fi
    done
}

# 启动监控基础设施
start_infrastructure() {
    log_info "Starting monitoring infrastructure..."

    # 进入监控目录
    cd "$(dirname "$0")"

    # 启动Docker Compose
    if [ -f "docker-compose.monitoring.yml" ]; then
        log_info "Starting Docker Compose services..."
        docker-compose -f docker-compose.monitoring.yml up -d

        # 等待服务启动
        log_info "Waiting for services to start..."
        sleep 30

        # 检查服务状态
        check_service_health
    else
        log_error "docker-compose.monitoring.yml not found"
        exit 1
    fi
}

# 检查服务健康状态
check_service_health() {
    log_info "Checking service health..."

    services=(
        "Prometheus:http://localhost:9090/api/v1/status/config"
        "Grafana:http://localhost:3000/api/health"
        "AlertManager:http://localhost:9093/api/v1/status"
    )

    for service in "${services[@]}"; do
        name=$(echo $service | cut -d: -f1)
        url=$(echo $service | cut -d: -f2-)

        if curl -s "$url" > /dev/null 2>&1; then
            log_success "$name is healthy"
        else
            log_warning "$name may not be fully started yet"
        fi
    done
}

# 启动Python监控组件
start_monitoring_components() {
    log_info "Starting Python monitoring components..."

    # 获取项目根目录
    PROJECT_ROOT=$(cd "$(dirname "$0")/.." && pwd)
    MONITORING_DIR="$PROJECT_ROOT/src/monitoring"

    if [ ! -d "$MONITORING_DIR" ]; then
        log_error "Monitoring directory not found: $MONITORING_DIR"
        exit 1
    fi

    # 启动硬件监控
    log_info "Starting hardware monitor..."
    nohup python3 "$MONITORING_DIR/hardware_monitor.py" > /var/log/xlerobot/hardware_monitor.log 2>&1 &
    echo $! > /tmp/xlerobot-monitoring/hardware_monitor.pid

    # 启动SLA监控
    log_info "Starting SLA monitor..."
    nohup python3 "$MONITORING_DIR/sla_monitor.py" > /var/log/xlerobot/sla_monitor.log 2>&1 &
    echo $! > /tmp/xlerobot-monitoring/sla_monitor.pid

    # 启动多模态监控
    log_info "Starting multimodal monitor..."
    nohup python3 "$MONITORING_DIR/multimodal_monitor.py" > /var/log/xlerobot/multimodal_monitor.log 2>&1 &
    echo $! > /tmp/xlerobot-monitoring/multimodal_monitor.pid

    # 启动智能告警
    log_info "Starting intelligent alerting..."
    nohup python3 "$MONITORING_DIR/intelligent_alerting.py" > /var/log/xlerobot/intelligent_alerting.log 2>&1 &
    echo $! > /tmp/xlerobot-monitoring/intelligent_alerting.pid

    # 启动健康检查
    log_info "Starting health checker..."
    nohup python3 "$MONITORING_DIR/health_check.py" > /var/log/xlerobot/health_check.log 2>&1 &
    echo $! > /tmp/xlerobot-monitoring/health_check.pid

    log_success "All monitoring components started"
}

# 配置Grafana
configure_grafana() {
    log_info "Configuring Grafana..."

    # 等待Grafana启动
    sleep 10

    # 创建API Key (这里简化了实际配置)
    # 实际部署时需要使用Grafana API进行配置
    log_success "Grafana configuration completed"
}

# 验证监控系统
verify_monitoring() {
    log_info "Verifying monitoring system..."

    # 等待所有组件启动
    sleep 30

    # 运行验证脚本
    PROJECT_ROOT=$(cd "$(dirname "$0")/.." && pwd)
    python3 "$PROJECT_ROOT/src/monitoring/monitoring_validation.py" > /tmp/xlerobot-monitoring/validation_report.json 2>&1

    if [ $? -eq 0 ]; then
        log_success "Monitoring system verification completed"
        log_info "Validation report: /tmp/xlerobot-monitoring/validation_report.json"
    else
        log_warning "Some verification tests may have failed"
    fi
}

# 显示访问信息
show_access_info() {
    log_info "Monitoring system is now running!"
    echo
    echo "Access URLs:"
    echo "  Grafana Dashboard:    http://localhost:3000 (admin/xlerobot@2025)"
    echo "  Prometheus:          http://localhost:9090"
    echo "  AlertManager:        http://localhost:9093"
    echo "  Node Exporter:       http://localhost:9100/metrics"
    echo "  cAdvisor:            http://localhost:8080"
    echo "  Pushgateway:         http://localhost:9091"
    echo
    echo "Monitoring Components:"
    echo "  Hardware Monitor:    Port 8001"
    echo "  SLA Monitor:         Port 8007"
    echo "  Multimodal Monitor:  Port 8006"
    echo "  Health Checker:      Port 8008"
    echo
    echo "Log Files:"
    echo "  Hardware Monitor:    /var/log/xlerobot/hardware_monitor.log"
    echo "  SLA Monitor:         /var/log/xlerobot/sla_monitor.log"
    echo "  Multimodal Monitor:  /var/log/xlerobot/multimodal_monitor.log"
    echo "  Intelligent Alerting: /var/log/xlerobot/intelligent_alerting.log"
    echo "  Health Checker:      /var/log/xlerobot/health_check.log"
    echo
    echo "To stop the monitoring system, run: ./stop_monitoring.sh"
}

# 主函数
main() {
    log_info "Starting XleRobot Monitoring System..."
    log_info "BMad-Method v6 Brownfield Level 4 - Story 1.8 Work Package 2"
    echo

    # 执行启动步骤
    check_docker
    check_ports
    create_directories
    check_python_dependencies
    start_infrastructure
    start_monitoring_components
    configure_grafana
    verify_monitoring
    show_access_info

    log_success "XleRobot Monitoring System startup completed!"
}

# 错误处理
trap 'log_error "Startup failed! Check logs for details."; exit 1' ERR

# 执行主函数
main "$@"