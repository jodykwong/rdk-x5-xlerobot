#!/usr/bin/env python3.10
"""
XLeRobot 简化兼容性检查器
========================

快速检查系统兼容性，确保核心组件正常工作。

作者: Claude Code Agent
版本: 1.0
日期: 2025-11-19
"""

import os
import sys
import subprocess
import platform
import time
from typing import Dict, List, Any, Tuple

def check_python_version() -> Tuple[bool, str, Dict[str, Any]]:
    """检查Python版本"""
    current_version = sys.version_info[:3]
    required_version = (3, 10, 0)  # 最低要求3.10

    if current_version >= required_version:
        return True, f"Python版本符合要求: {'.'.join(map(str, current_version))}", {
            "current_version": '.'.join(map(str, current_version)),
            "python_executable": sys.executable
        }
    else:
        return False, f"Python版本过低: {'.'.join(map(str, current_version))}, 要求 3.10+", {
            "current_version": '.'.join(map(str, current_version))
        }

def check_environment_variables() -> Tuple[bool, str, Dict[str, Any]]:
    """检查关键环境变量"""
    required_vars = [
        'ALIBABA_CLOUD_ACCESS_KEY_ID',
        'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
        'ALIYUN_NLS_APPKEY',
        'QWEN_API_KEY'
    ]

    missing_vars = []
    set_vars = []

    for var in required_vars:
        if os.environ.get(var):
            set_vars.append(var)
        else:
            missing_vars.append(var)

    if not missing_vars:
        return True, f"所有环境变量已设置 ({len(set_vars)}/{len(required_vars)})", {
            "set_count": len(set_vars),
            "missing_vars": missing_vars
        }
    else:
        return False, f"缺少环境变量: {', '.join(missing_vars)}", {
            "set_count": len(set_vars),
            "missing_vars": missing_vars
        }

def check_audio_devices() -> Tuple[bool, str, Dict[str, Any]]:
    """检查音频设备"""
    try:
        # 检查录音设备
        result_record = subprocess.run(
            ['arecord', '-l'],
            capture_output=True,
            text=True,
            timeout=5
        )

        # 检查播放设备
        result_play = subprocess.run(
            ['aplay', '-l'],
            capture_output=True,
            text=True,
            timeout=5
        )

        record_devices = len(result_record.stdout.strip().split('\n')) if result_record.returncode == 0 else 0
        play_devices = len(result_play.stdout.strip().split('\n')) if result_play.returncode == 0 else 0

        if record_devices > 0 and play_devices > 0:
            return True, f"音频设备检测成功 (录音:{record_devices}, 播放:{play_devices})", {
                "record_devices": record_devices,
                "play_devices": play_devices
            }
        else:
            return False, f"音频设备不完整 (录音:{record_devices}, 播放:{play_devices})", {
                "record_devices": record_devices,
                "play_devices": play_devices
            }

    except Exception as e:
        return False, f"音频设备检查失败: {e}", {"error": str(e)}

def check_ros2_environment() -> Tuple[bool, str, Dict[str, Any]]:
    """检查ROS2环境"""
    ros_distro = os.environ.get('ROS_DISTRO')

    if ros_distro and 'humble' in ros_distro.lower():
        return True, f"ROS2环境正确: {ros_distro}", {
            "ros_distro": ros_distro,
            "ros_domain_id": os.environ.get('ROS_DOMAIN_ID', '未设置')
        }
    elif ros_distro:
        return False, f"ROS2版本不支持: {ros_distro}", {"ros_distro": ros_distro}
    else:
        return False, "ROS2环境未设置", {"ros_distro": None}

def check_network_connectivity() -> Tuple[bool, str, Dict[str, Any]]:
    """检查网络连接"""
    import socket

    test_hosts = [
        ('8.8.8.8', 53),      # Google DNS
        ('aliyun.com', 443),   # 阿里云
    ]

    connected = 0
    failed = []

    for host, port in test_hosts:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3)
            result = sock.connect_ex((host, port))
            sock.close()

            if result == 0:
                connected += 1
            else:
                failed.append(f"{host}:{port}")
        except Exception:
            failed.append(f"{host}:{port}")

    if connected >= len(test_hosts) // 2:
        return True, f"网络连接正常 ({connected}/{len(test_hosts)})", {
            "connected": connected,
            "total_tests": len(test_hosts),
            "failed_hosts": failed
        }
    else:
        return False, f"网络连接异常 ({connected}/{len(test_hosts)})", {
            "connected": connected,
            "total_tests": len(test_hosts),
            "failed_hosts": failed
        }

def run_compatibility_check() -> Dict[str, Any]:
    """运行完整兼容性检查"""
    print("🔍 开始系统兼容性检查...")

    # 定义检查项目
    checks = [
        ("Python版本", check_python_version),
        ("环境变量", check_environment_variables),
        ("音频设备", check_audio_devices),
        ("ROS2环境", check_ros2_environment),
        ("网络连接", check_network_connectivity),
    ]

    start_time = time.time()
    results = []

    for name, check_func in checks:
        try:
            print(f"  检查 {name}...")
            success, message, details = check_func()

            results.append({
                "name": name,
                "success": success,
                "message": message,
                "details": details
            })

            status = "✅" if success else "❌"
            print(f"    {status} {message}")

        except Exception as e:
            print(f"    ❌ 检查失败: {e}")
            results.append({
                "name": name,
                "success": False,
                "message": f"检查异常: {e}",
                "details": {"error": str(e)}
            })

    total_time = time.time() - start_time

    # 统计结果
    total_checks = len(results)
    passed_checks = sum(1 for r in results if r["success"])
    pass_rate = passed_checks / total_checks if total_checks > 0 else 0

    # 判断总体状态
    if pass_rate >= 0.8:
        overall_status = "PASS"
        overall_message = "系统兼容性检查通过"
    elif pass_rate >= 0.6:
        overall_status = "WARNING"
        overall_message = "系统兼容性基本满足，有部分问题"
    else:
        overall_status = "FAIL"
        overall_message = "系统兼容性检查失败，存在关键问题"

    # 显示失败的项目
    failed_checks = [r for r in results if not r["success"]]
    if failed_checks:
        print(f"\n❌ 失败检查 ({len(failed_checks)}):")
        for check in failed_checks:
            print(f"  - {check['name']}: {check['message']}")

    print(f"\n📊 检查完成:")
    print(f"  - 总体状态: {overall_status}")
    print(f"  - 通过率: {pass_rate:.1%} ({passed_checks}/{total_checks})")
    print(f"  - 检查时间: {total_time:.2f}秒")

    return {
        "overall_status": overall_status,
        "overall_message": overall_message,
        "summary": {
            "total_checks": total_checks,
            "passed_checks": passed_checks,
            "pass_rate": pass_rate,
            "duration": total_time
        },
        "results": results
    }

# 测试函数
def test_compatibility_checker():
    """测试兼容性检查器"""
    result = run_compatibility_check()
    return result

if __name__ == "__main__":
    test_compatibility_checker()