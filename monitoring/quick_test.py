#!/usr/bin/env python3
"""
XleRobot Monitoring System Quick Test
监控系统快速测试脚本
"""

import time
import json
import requests
import logging
from datetime import datetime

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('QuickTest')

def test_service_endpoints():
    """测试各个服务端点"""
    services = {
        "Grafana": "http://localhost:3000/api/health",
        "Prometheus": "http://localhost:9090/api/v1/status/config",
        "AlertManager": "http://localhost:9093/api/v1/status",
        "Node Exporter": "http://localhost:9100/metrics",
        "Hardware Monitor": "http://localhost:8001/metrics",
        "SLA Monitor": "http://localhost:8007/metrics",
        "Multimodal Monitor": "http://localhost:8006/metrics",
        "Health Checker": "http://localhost:8008/metrics"
    }

    results = {}

    for service, url in services.items():
        try:
            response = requests.get(url, timeout=5)
            if response.status_code == 200:
                results[service] = "✅ OK"
                logger.info(f"{service}: OK")
            else:
                results[service] = f"❌ HTTP {response.status_code}"
                logger.warning(f"{service}: HTTP {response.status_code}")
        except Exception as e:
            results[service] = f"❌ Error: {str(e)}"
            logger.error(f"{service}: {str(e)}")

    return results

def test_prometheus_metrics():
    """测试Prometheus指标采集"""
    try:
        # 查询系统健康分数
        response = requests.get(
            "http://localhost:9090/api/v1/query?query=system_health_score",
            timeout=10
        )

        if response.status_code == 200:
            data = response.json()
            if data['status'] == 'success' and data['data']['result']:
                value = float(data['data']['result'][0]['value'][1])
                return f"✅ 系统健康分数: {value:.2f}"
            else:
                return "❌ 无系统健康分数数据"
        else:
            return f"❌ HTTP {response.status_code}"
    except Exception as e:
        return f"❌ Error: {str(e)}"

def test_grafana_dashboard():
    """测试Grafana仪表板"""
    try:
        # 检查Grafana API是否可用
        response = requests.get(
            "http://localhost:3000/api/dashboards/home",
            headers={'Authorization': 'Basic YWRtaW46eGxlcm9ib3RAMjAyNQ=='},  # admin:xlerobot@2025
            timeout=5
        )

        if response.status_code == 200:
            return "✅ Grafana仪表板可访问"
        else:
            return f"❌ HTTP {response.status_code}"
    except Exception as e:
        return f"❌ Error: {str(e)}"

def generate_test_report():
    """生成测试报告"""
    print("\n" + "="*60)
    print("XleRobot 监控系统快速测试")
    print("="*60)
    print(f"测试时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("-"*60)

    # 测试服务端点
    print("1. 服务端点测试:")
    service_results = test_service_endpoints()
    for service, status in service_results.items():
        print(f"   {service}: {status}")

    print("\n2. Prometheus指标测试:")
    metrics_result = test_prometheus_metrics()
    print(f"   {metrics_result}")

    print("\n3. Grafana仪表板测试:")
    dashboard_result = test_grafana_dashboard()
    print(f"   {dashboard_result}")

    # 统计结果
    ok_count = sum(1 for result in [metrics_result, dashboard_result] + list(service_results.values()) if "✅" in result)
    total_count = len([metrics_result, dashboard_result] + list(service_results.values()))

    print("-"*60)
    print(f"测试结果: {ok_count}/{total_count} 服务正常")
    print(f"成功率: {ok_count/total_count*100:.1f}%")

    if ok_count == total_count:
        print("🎉 所有监控服务运行正常!")
    else:
        print("⚠️  部分服务存在问题，请检查日志")

    print("="*60)

    # 输出访问信息
    print("\n📊 访问地址:")
    print("   Grafana仪表板: http://localhost:3000 (admin/xlerobot@2025)")
    print("   Prometheus: http://localhost:9090")
    print("   AlertManager: http://localhost:9093")
    print("   系统健康分数: http://localhost:9090/api/v1/query?query=system_health_score")

if __name__ == '__main__':
    generate_test_report()