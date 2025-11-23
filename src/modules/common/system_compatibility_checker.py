#!/usr/bin/env python3.10
"""
XLeRobot 系统兼容性检查器
========================

提供全面的系统兼容性检查和监控功能。
确保所有组件在目标环境下正常工作。

主要功能：
- Python版本和依赖检查
- ROS2环境验证
- 音频设备检测
- 网络连接测试
- API密钥验证
- 性能基准测试

作者: Claude Code Agent
版本: 1.0
日期: 2025-11-19
"""

import os
import sys
import subprocess
import platform
import logging
import time
import socket
import threading
import importlib.util
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import json

logger = logging.getLogger(__name__)

class CheckStatus(Enum):
    """检查状态枚举"""
    PASS = "pass"
    FAIL = "fail"
    WARNING = "warning"
    SKIP = "skip"

class CheckLevel(Enum):
    """检查级别枚举"""
    CRITICAL = "critical"  # 关键检查，必须通过
    IMPORTANT = "important"  # 重要检查，建议通过
    OPTIONAL = "optional"   # 可选检查

@dataclass
class CheckResult:
    """检查结果"""
    name: str
    level: CheckLevel
    status: CheckStatus
    message: str
    details: Optional[Dict[str, Any]] = None
    duration: float = 0.0
    timestamp: float = 0.0

class SystemCompatibilityChecker:
    """
    系统兼容性检查器

    全面检查系统兼容性，确保XLeRobot能够正常运行。
    """

    def __init__(self):
        """初始化系统兼容性检查器"""
        self.results: List[CheckResult] = []
        self.start_time = time.time()

        logger.info("🔍 系统兼容性检查器初始化")

    def run_all_checks(self) -> Dict[str, Any]:
        """
        运行所有兼容性检查

        Returns:
            检查结果汇总
        """
        logger.info("🔍 开始运行所有兼容性检查...")

        # 清空之前的结果
        self.results = []
        self.start_time = time.time()

        # 直接运行简化的检查
        checks_to_run = [
            ("Python版本检查", self._check_python_version, CheckLevel.CRITICAL),
            ("环境变量检查", self._check_environment_variables, CheckLevel.CRITICAL),
            ("音频设备检查", self._check_audio_devices, CheckLevel.IMPORTANT),
            ("网络连接检查", self._check_internet_connection, CheckLevel.IMPORTANT),
        ]

        # 执行检查
        for name, check_func, level in checks_to_run:
            try:
                check_start = time.time()
                status, message, details = check_func()
                duration = time.time() - check_start

                result = CheckResult(
                    name=name,
                    level=level,
                    status=status,
                    message=message,
                    details=details,
                    duration=duration,
                    timestamp=time.time()
                )
                self.results.append(result)

            except Exception as e:
                logger.error(f"检查执行失败: {name}, 错误: {e}")
                # 创建失败结果
                error_result = CheckResult(
                    name=name,
                    level=level,
                    status=CheckStatus.FAIL,
                    message=f"检查执行异常: {e}",
                    duration=0,
                    timestamp=time.time()
                )
                self.results.append(error_result)

        # 生成汇总报告
        return self._generate_summary()

    def _check_python_version(self) -> Tuple[CheckStatus, str, Dict[str, Any]]:
        """检查Python版本"""
        required_version = (3, 10, 12)
        current_version = sys.version_info[:3]

        if current_version >= required_version:
            status = CheckStatus.PASS
            message = f"Python版本符合要求: {'.'.join(map(str, current_version))}"
        else:
            status = CheckStatus.FAIL
            message = f"Python版本不符合要求: 当前{'.'.join(map(str, current_version))}, 要求 {'.'.join(map(str, required_version))}"

        details = {
            "current_version": '.'.join(map(str, current_version)),
            "required_version": '.'.join(map(str, required_version)),
            "python_executable": sys.executable,
            "is_venv": hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)
        }

        return status, message, details

    def _check_system_platform(self) -> CheckResult:
        """检查系统平台"""
        current_platform = platform.system().lower()
        current_arch = platform.machine().lower()

        # 支持的平台
        supported_platforms = ['linux']
        supported_archs = ['x86_64', 'aarch64', 'arm64']

        if current_platform in supported_platforms and current_arch in supported_archs:
            status = CheckStatus.PASS
            message = f"支持的平台: {current_platform}-{current_arch}"
        else:
            status = CheckStatus.WARNING
            message = f"未验证的平台: {current_platform}-{current_arch}"

        return CheckResult(
            name="系统平台检查",
            status=status,
            message=message,
            details={
                "platform": current_platform,
                "architecture": current_arch,
                "platform_release": platform.release(),
                "platform_version": platform.version()
            }
        )

    def _check_environment_variables(self) -> CheckResult:
        """检查环境变量"""
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
            status = CheckStatus.PASS
            message = f"所有必需的环境变量已设置 ({len(set_vars)}/{len(required_vars)})"
        else:
            status = CheckStatus.FAIL
            message = f"缺少环境变量: {', '.join(missing_vars)}"

        return CheckResult(
            name="环境变量检查",
            status=status,
            message=message,
            details={
                "required_count": len(required_vars),
                "set_count": len(set_vars),
                "missing_vars": missing_vars,
                "set_vars": set_vars
            }
        )

    def _check_required_packages(self) -> CheckResult:
        """检查必需的Python包"""
        required_packages = {
            'yaml': 'PyYAML',
            'numpy': 'numpy',
            'scipy': 'scipy',
            'rclpy': 'ROS2 Python客户端',
            'std_msgs': 'ROS2标准消息'
        }

        installed = {}
        missing = []

        for module, package in required_packages.items():
            try:
                spec = importlib.util.find_spec(module)
                if spec:
                    installed[module] = package
                else:
                    missing.append(package)
            except ImportError:
                missing.append(package)

        if not missing:
            status = CheckStatus.PASS
            message = f"所有必需包已安装 ({len(installed)}/{len(required_packages)})"
        else:
            status = CheckStatus.FAIL
            message = f"缺少必需包: {', '.join(missing)}"

        return CheckResult(
            name="必需Python包检查",
            status=status,
            message=message,
            details={
                "installed": installed,
                "missing": missing,
                "total_required": len(required_packages)
            }
        )

    def _check_optional_packages(self) -> CheckResult:
        """检查可选的Python包"""
        optional_packages = {
            'pyaudio': 'PyAudio',
            'soundfile': 'SoundFile',
            'librosa': 'librosa',
            'matplotlib': 'matplotlib'
        }

        installed = {}
        missing = []

        for module, package in optional_packages.items():
            try:
                spec = importlib.util.find_spec(module)
                if spec:
                    installed[module] = package
                else:
                    missing.append(package)
            except ImportError:
                missing.append(package)

        if len(installed) >= len(optional_packages) // 2:
            status = CheckStatus.PASS
            message = f"大部分可选包已安装 ({len(installed)}/{len(optional_packages)})"
        else:
            status = CheckStatus.WARNING
            message = f"部分可选包缺失: {', '.join(missing)}"

        return CheckResult(
            name="可选Python包检查",
            status=status,
            message=message,
            details={
                "installed": installed,
                "missing": missing,
                "total_optional": len(optional_packages)
            }
        )

    def _check_ros2_environment(self) -> CheckResult:
        """检查ROS2环境"""
        ros_distro = os.environ.get('ROS_DISTRO')
        ament_prefix_path = os.environ.get('AMENT_PREFIX_PATH', '')

        if ros_distro and 'humble' in ros_distro.lower():
            status = CheckStatus.PASS
            message = f"ROS2环境正确: {ros_distro}"
        elif ros_distro:
            status = CheckStatus.WARNING
            message = f"ROS2版本可能不支持: {ros_distro}"
        else:
            status = CheckStatus.FAIL
            message = "ROS2环境未设置"

        return CheckResult(
            name="ROS2环境检查",
            status=status,
            message=message,
            details={
                "ros_distro": ros_distro,
                "ament_prefix_path": ament_prefix_path.split(':') if ament_prefix_path else [],
                "ros_domain_id": os.environ.get('ROS_DOMAIN_ID')
            }
        )

    def _check_ros2_packages(self) -> CheckResult:
        """检查ROS2包"""
        required_packages = [
            'rclpy',
            'std_msgs',
            'audio_msg',
            'xlerobot'
        ]

        try:
            # 使用ros2 pkg list命令
            result = subprocess.run(
                ['ros2', 'pkg', 'list'],
                capture_output=True,
                text=True,
                timeout=10
            )

            if result.returncode == 0:
                installed_packages = result.stdout.strip().split('\n')
                found_packages = [pkg for pkg in required_packages if any(pkg in line for line in installed_packages)]

                if len(found_packages) >= len(required_packages) - 1:  # audio_msg可选
                    status = CheckStatus.PASS
                    message = f"ROS2包检查通过 ({len(found_packages)}/{len(required_packages)})"
                else:
                    status = CheckStatus.WARNING
                    missing = set(required_packages) - set(found_packages)
                    message = f"部分ROS2包缺失: {', '.join(missing)}"

                return CheckResult(
                    name="ROS2包检查",
                    status=status,
                    message=message,
                    details={
                        "required": required_packages,
                        "found": found_packages,
                        "missing": list(set(required_packages) - set(found_packages))
                    }
                )
            else:
                status = CheckStatus.WARNING
                message = "无法获取ROS2包列表"

        except Exception as e:
            status = CheckStatus.WARNING
            message = f"ROS2包检查失败: {e}"

        return CheckResult(
            name="ROS2包检查",
            status=status,
            message=message
        )

    def _check_audio_devices(self) -> CheckResult:
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
                status = CheckStatus.PASS
                message = f"音频设备检测成功 (录音:{record_devices}, 播放:{play_devices})"
            elif record_devices > 0 or play_devices > 0:
                status = CheckStatus.WARNING
                message = f"部分音频设备可用 (录音:{record_devices}, 播放:{play_devices})"
            else:
                status = CheckStatus.FAIL
                message = "未检测到音频设备"

            return CheckResult(
                name="音频设备检查",
                status=status,
                message=message,
                details={
                    "record_devices": record_devices,
                    "play_devices": play_devices,
                    "arecord_output": result_record.stdout if result_record.returncode == 0 else result_record.stderr,
                    "aplay_output": result_play.stdout if result_play.returncode == 0 else result_play.stderr
                }
            )

        except Exception as e:
            return CheckResult(
                name="音频设备检查",
                status=CheckStatus.FAIL,
                message=f"音频设备检查失败: {e}"
            )

    def _check_alsa_tools(self) -> CheckResult:
        """检查ALSA工具"""
        tools = ['arecord', 'aplay', 'amixer']
        installed = []

        for tool in tools:
            try:
                result = subprocess.run(
                    ['which', tool],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode == 0:
                    installed.append(tool)
            except Exception:
                pass

        if len(installed) == len(tools):
            status = CheckStatus.PASS
            message = f"所有ALSA工具已安装: {', '.join(installed)}"
        elif len(installed) >= len(tools) - 1:
            status = CheckStatus.WARNING
            message = f"部分ALSA工具缺失: {', '.join(installed)}/{len(tools)}"
        else:
            status = CheckStatus.FAIL
            message = f"关键ALSA工具缺失: {set(tools) - set(installed)}"

        return CheckResult(
            name="ALSA工具检查",
            status=status,
            message=message,
            details={
                "required_tools": tools,
                "installed_tools": installed,
                "missing_tools": list(set(tools) - set(installed))
            }
        )

    def _check_internet_connection(self) -> CheckResult:
        """检查网络连接"""
        test_hosts = [
            ('8.8.8.8', 53),      # Google DNS
            ('1.1.1.1', 53),      # Cloudflare DNS
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
            status = CheckStatus.PASS
            message = f"网络连接正常 ({connected}/{len(test_hosts)})"
        elif connected > 0:
            status = CheckStatus.WARNING
            message = f"网络连接部分异常 ({connected}/{len(test_hosts)})"
        else:
            status = CheckStatus.FAIL
            message = "网络连接失败"

        return CheckResult(
            name="网络连接检查",
            status=status,
            message=message,
            details={
                "total_tests": len(test_hosts),
                "successful": connected,
                "failed_hosts": failed
            }
        )

    def _check_aliyun_connectivity(self) -> CheckResult:
        """检查阿里云服务连接"""
        test_endpoints = [
            'nls-gateway.aliyuncs.com',
            'nls-meta.cn-shanghai.aliyuncs.com'
        ]

        connected = 0
        failed = []

        for endpoint in test_endpoints:
            try:
                # 使用HTTP连接测试
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5)
                result = sock.connect_ex((endpoint, 443))
                sock.close()

                if result == 0:
                    connected += 1
                else:
                    failed.append(endpoint)
            except Exception:
                failed.append(endpoint)

        if connected >= len(test_endpoints):
            status = CheckStatus.PASS
            message = f"阿里云服务连接正常 ({connected}/{len(test_endpoints)})"
        elif connected > 0:
            status = CheckStatus.WARNING
            message = f"阿里云服务部分异常 ({connected}/{len(test_endpoints)})"
        else:
            status = CheckStatus.FAIL
            message = "阿里云服务连接失败"

        return CheckResult(
            name="阿里云连接检查",
            status=status,
            message=message,
            details={
                "total_endpoints": len(test_endpoints),
                "successful": connected,
                "failed_endpoints": failed
            }
        )

    def _check_api_credentials(self) -> CheckResult:
        """检查API凭证"""
        credentials = {
            'ALIBABA_CLOUD_ACCESS_KEY_ID': os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_ID'),
            'ALIBABA_CLOUD_ACCESS_KEY_SECRET': os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_SECRET'),
            'ALIYUN_NLS_APPKEY': os.environ.get('ALIYUN_NLS_APPKEY'),
            'QWEN_API_KEY': os.environ.get('QWEN_API_KEY')
        }

        valid_count = 0
        invalid = []

        for key, value in credentials.items():
            if value and len(value.strip()) > 0:
                valid_count += 1
            else:
                invalid.append(key)

        if valid_count == len(credentials):
            status = CheckStatus.PASS
            message = f"所有API凭证已配置 ({valid_count}/{len(credentials)})"
        elif valid_count >= len(credentials) - 1:
            status = CheckStatus.WARNING
            message = f"部分API凭证缺失: {', '.join(invalid)}"
        else:
            status = CheckStatus.FAIL
            message = f"关键API凭证缺失: {', '.join(invalid)}"

        return CheckResult(
            name="API凭证检查",
            status=status,
            message=message,
            details={
                "total_credentials": len(credentials),
                "valid_count": valid_count,
                "invalid_credentials": invalid
            }
        )

    def _check_api_permissions(self) -> CheckResult:
        """检查API权限（简单检查）"""
        # 这里只做基本的凭证格式检查，实际权限测试需要调用API
        access_key_id = os.environ.get('ALIBABA_CLOUD_ACCESS_KEY_ID', '')
        appkey = os.environ.get('ALIYUN_NLS_APPKEY', '')

        if access_key_id.startswith('LTAI') and len(appkey) >= 8:
            status = CheckStatus.PASS
            message = "API凭证格式正确"
        elif access_key_id or appkey:
            status = CheckStatus.WARNING
            message = "API凭证可能无效"
        else:
            status = CheckStatus.SKIP
            message = "API凭证未设置，跳过检查"

        return CheckResult(
            name="API权限检查",
            status=status,
            message=message,
            details={
                "access_key_format": access_key_id.startswith('LTAI') if access_key_id else False,
                "appkey_length": len(appkey) if appkey else 0
            }
        )

    def _check_system_resources(self) -> CheckResult:
        """检查系统资源"""
        try:
            import psutil

            # CPU使用率
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')

            details = {
                "cpu_percent": cpu_percent,
                "memory_percent": memory.percent,
                "memory_available_gb": memory.available / (1024**3),
                "disk_percent": disk.percent,
                "disk_free_gb": disk.free / (1024**3)
            }

            if cpu_percent < 80 and memory.percent < 80 and disk.percent < 90:
                status = CheckStatus.PASS
                message = "系统资源充足"
            elif cpu_percent < 95 and memory.percent < 95:
                status = CheckStatus.WARNING
                message = "系统资源紧张"
            else:
                status = CheckStatus.FAIL
                message = "系统资源不足"

            return CheckResult(
                name="系统资源检查",
                status=status,
                message=message,
                details=details
            )

        except ImportError:
            return CheckResult(
                name="系统资源检查",
                status=CheckStatus.SKIP,
                message="psutil未安装，跳过系统资源检查"
            )
        except Exception as e:
            return CheckResult(
                name="系统资源检查",
                status=CheckStatus.WARNING,
                message=f"系统资源检查失败: {e}"
            )

    def _check_audio_latency(self) -> CheckResult:
        """检查音频延迟"""
        try:
            # 简单的音频延迟测试
            start_time = time.time()

            # 尝试创建一个短的录音测试
            result = subprocess.run(
                ['arecord', '-d', '0.1', '-f', 'cd', '/tmp/test_latency.wav'],
                capture_output=True,
                text=True,
                timeout=2
            )

            end_time = time.time()
            latency = end_time - start_time

            # 清理测试文件
            try:
                os.unlink('/tmp/test_latency.wav')
            except:
                pass

            if result.returncode == 0 and latency < 1.0:
                status = CheckStatus.PASS
                message = f"音频延迟正常 ({latency:.3f}秒)"
            elif latency < 2.0:
                status = CheckStatus.WARNING
                message = f"音频延迟较高 ({latency:.3f}秒)"
            else:
                status = CheckStatus.FAIL
                message = f"音频延迟过高 ({latency:.3f}秒)"

            return CheckResult(
                name="音频延迟检查",
                status=status,
                message=message,
                details={
                    "latency_seconds": latency,
                    "arecord_returncode": result.returncode,
                    "arecord_stderr": result.stderr
                }
            )

        except Exception as e:
            return CheckResult(
                name="音频延迟检查",
                status=CheckStatus.WARNING,
                message=f"音频延迟检查失败: {e}"
            )

    def _generate_summary(self) -> Dict[str, Any]:
        """生成检查结果汇总"""
        total_time = time.time() - self.start_time

        # 统计各级别和状态
        level_counts = {}
        status_counts = {}

        for result in self.results:
            # 级别统计
            level = result.level.value
            level_counts[level] = level_counts.get(level, 0) + 1

            # 状态统计
            status = result.status.value
            status_counts[status] = status_counts.get(status, 0) + 1

        # 计算通过率
        critical_checks = [r for r in self.results if r.level == CheckLevel.CRITICAL]
        critical_passed = sum(1 for r in critical_checks if r.status == CheckStatus.PASS)

        critical_pass_rate = (critical_passed / len(critical_checks)) if critical_checks else 0

        # 判断总体状态
        if critical_pass_rate == 1.0:
            overall_status = "PASS"
            overall_message = "系统兼容性检查通过"
        elif critical_pass_rate >= 0.8:
            overall_status = "WARNING"
            overall_message = "系统兼容性基本满足，有部分问题"
        else:
            overall_status = "FAIL"
            overall_message = "系统兼容性检查失败，存在关键问题"

        return {
            "overall_status": overall_status,
            "overall_message": overall_message,
            "summary": {
                "total_checks": len(self.results),
                "critical_checks": len(critical_checks),
                "critical_passed": critical_passed,
                "critical_pass_rate": critical_pass_rate,
                "total_duration": total_time
            },
            "level_distribution": level_counts,
            "status_distribution": status_counts,
            "results": [
                {
                    "name": result.name,
                    "level": result.level.value,
                    "status": result.status.value,
                    "message": result.message,
                    "duration": result.duration,
                    "details": result.details
                }
                for result in self.results
            ]
        }

def run_compatibility_check() -> Dict[str, Any]:
    """运行兼容性检查的便捷函数"""
    checker = SystemCompatibilityChecker()
    return checker.run_all_checks()

# 测试函数
def test_compatibility_checker():
    """测试兼容性检查器"""
    logging.basicConfig(level=logging.INFO)

    print("🧪 测试系统兼容性检查器...")

    result = run_compatibility_check()

    print(f"📊 检查结果: {result['overall_status']}")
    print(f"💬 检查消息: {result['overall_message']}")
    print(f"📈 关键检查通过率: {result['summary']['critical_pass_rate']:.1%}")

    # 显示失败的项目
    failed_checks = [r for r in result['results'] if r['status'] == 'FAIL']
    if failed_checks:
        print(f"❌ 失败检查 ({len(failed_checks)}):")
        for check in failed_checks:
            print(f"  - {check['name']}: {check['message']}")

    print("🎉 兼容性检查完成")

if __name__ == "__main__":
    test_compatibility_checker()