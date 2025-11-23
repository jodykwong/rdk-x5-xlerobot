#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
XLeRobot 项目状态快速验证脚本
检查项目环境、依赖、配置和核心组件状态
"""

import os
import sys
import subprocess
import json
from pathlib import Path
from typing import Dict, List, Any

class ProjectStatusVerifier:
    """项目状态验证器"""

    def __init__(self):
        self.project_root = Path(__file__).parent
        self.issues = []
        self.warnings = []
        self.success_count = 0
        self.total_checks = 0

    def log(self, message: str, status: str = "INFO"):
        """日志输出"""
        icons = {
            "SUCCESS": "✅",
            "ERROR": "❌",
            "WARNING": "⚠️",
            "INFO": "ℹ️"
        }
        icon = icons.get(status, "ℹ️")
        print(f"{icon} {message}")

    def check_python_version(self) -> bool:
        """检查Python版本"""
        self.total_checks += 1

        version = sys.version_info
        if version.major == 3 and version.minor == 10:
            self.log(f"Python版本正确: {version.major}.{version.minor}.{version.micro}", "SUCCESS")
            self.success_count += 1
            return True
        else:
            self.log(f"Python版本错误: {version.major}.{version.minor}.{version.micro}, 需要Python 3.10", "ERROR")
            self.issues.append(f"Python版本错误: 需要Python 3.10, 当前为{version.major}.{version.minor}")
            return False

    def check_ros2_environment(self) -> bool:
        """检查ROS2环境"""
        self.total_checks += 1

        ros_distro = os.environ.get('ROS_DISTRO')
        if ros_distro == 'humble':
            self.log(f"ROS2版本正确: {ros_distro}", "SUCCESS")
            self.success_count += 1
            return True
        else:
            self.log(f"ROS2版本错误: {ros_distro or '未设置'}, 需要ROS2 Humble", "ERROR")
            self.issues.append(f"ROS2版本错误: 需要ROS2 Humble, 当前为{ros_distro}")
            return False

    def check_project_structure(self) -> bool:
        """检查项目结构"""
        self.total_checks += 1

        required_dirs = [
            "src",
            "src/modules",
            "src/modules/asr",
            "src/modules/llm",
            "src/modules/tts",
            "tests",
            "docs",
            "config"
        ]

        missing_dirs = []
        for dir_path in required_dirs:
            full_path = self.project_root / dir_path
            if not full_path.exists():
                missing_dirs.append(dir_path)

        if not missing_dirs:
            self.log("项目结构完整", "SUCCESS")
            self.success_count += 1
            return True
        else:
            self.log(f"缺少目录: {', '.join(missing_dirs)}", "ERROR")
            self.issues.append(f"缺少必需目录: {', '.join(missing_dirs)}")
            return False

    def check_environment_variables(self) -> bool:
        """检查环境变量"""
        self.total_checks += 1

        required_vars = [
            'ALIBABA_CLOUD_ACCESS_KEY_ID',
            'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
            'ALIYUN_NLS_APPKEY',
            'QWEN_API_KEY'
        ]

        # 检查.env文件
        env_file = self.project_root / ".env"
        if env_file.exists():
            self.log("找到.env配置文件", "SUCCESS")
            # 加载.env文件
            with open(env_file, 'r') as f:
                for line in f:
                    if '=' in line and not line.strip().startswith('#'):
                        key, value = line.split('=', 1)
                        os.environ[key.strip()] = value.strip()

        missing_vars = []
        for var in required_vars:
            if not os.environ.get(var):
                missing_vars.append(var)

        if not missing_vars:
            self.log("所有必需的环境变量已设置", "SUCCESS")
            self.success_count += 1
            return True
        else:
            self.log(f"缺少环境变量: {', '.join(missing_vars)}", "ERROR")
            self.issues.append(f"缺少环境变量: {', '.join(missing_vars)}")
            return False

    def check_dependencies(self) -> bool:
        """检查Python依赖"""
        self.total_checks += 1

        requirements_file = self.project_root / "requirements.txt"
        if not requirements_file.exists():
            self.log("requirements.txt文件不存在", "ERROR")
            self.issues.append("缺少requirements.txt文件")
            return False

        try:
            with open(requirements_file, 'r') as f:
                requirements = [line.strip() for line in f if line.strip() and not line.startswith('#')]

            # 检查关键依赖包
            key_packages = ['websockets', 'asyncio', 'requests', 'numpy']
            missing_packages = []

            for package in key_packages:
                try:
                    __import__(package)
                except ImportError:
                    missing_packages.append(package)

            if not missing_packages:
                self.log("关键依赖包已安装", "SUCCESS")
                self.success_count += 1
                return True
            else:
                self.log(f"缺少依赖包: {', '.join(missing_packages)}", "WARNING")
                self.warnings.append(f"建议安装依赖包: {', '.join(missing_packages)}")
                return False

        except Exception as e:
            self.log(f"检查依赖时出错: {str(e)}", "ERROR")
            self.issues.append(f"依赖检查失败: {str(e)}")
            return False

    def check_core_modules(self) -> bool:
        """检查核心模块文件"""
        self.total_checks += 1

        core_files = [
            "src/modules/asr/cloud_alibaba/alibaba_asr.py",
            "src/modules/llm/qwen_client.py",
            "src/modules/tts/cloud_alibaba/alibaba_tts.py",
            "src/start_epic1_services.py",
            "start_voice_assistant.sh"
        ]

        missing_files = []
        for file_path in core_files:
            full_path = self.project_root / file_path
            if not full_path.exists():
                missing_files.append(file_path)

        if not missing_files:
            self.log("所有核心模块文件存在", "SUCCESS")
            self.success_count += 1
            return True
        else:
            self.log(f"缺少核心文件: {', '.join(missing_files)}", "ERROR")
            self.issues.append(f"缺少核心文件: {', '.join(missing_files)}")
            return False

    def check_audio_devices(self) -> bool:
        """检查音频设备"""
        self.total_checks += 1

        try:
            # 检查录音设备
            record_result = subprocess.run(
                ['arecord', '-l'],
                capture_output=True,
                text=True,
                timeout=5
            )

            # 检查播放设备
            play_result = subprocess.run(
                ['aplay', '-l'],
                capture_output=True,
                text=True,
                timeout=5
            )

            if record_result.returncode == 0 and play_result.returncode == 0:
                self.log("音频设备正常", "SUCCESS")
                self.success_count += 1
                return True
            else:
                self.log("音频设备检测失败", "ERROR")
                self.issues.append("音频设备不可用")
                return False

        except subprocess.TimeoutExpired:
            self.log("音频设备检测超时", "WARNING")
            self.warnings.append("音频设备检测超时")
            return False
        except FileNotFoundError:
            self.log("ALSA工具未安装", "WARNING")
            self.warnings.append("建议安装alsa-utils")
            return False
        except Exception as e:
            self.log(f"音频设备检查出错: {str(e)}", "ERROR")
            self.issues.append(f"音频设备检查失败: {str(e)}")
            return False

    def check_ros2_connectivity(self) -> bool:
        """检查ROS2连接性"""
        self.total_checks += 1

        try:
            # 使用topic list检查ROS2功能
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode == 0:
                topics = result.stdout.strip().split('\n') if result.stdout.strip() else []
                self.log(f"ROS2功能正常: 发现 {len(topics)} 个话题", "SUCCESS")
                self.success_count += 1
                return True
            else:
                self.log("ROS2功能不可用", "ERROR")
                self.issues.append("ROS2未正确安装或未激活")
                return False

        except subprocess.TimeoutExpired:
            self.log("ROS2命令超时", "ERROR")
            self.issues.append("ROS2响应超时")
            return False
        except FileNotFoundError:
            self.log("ROS2命令未找到", "ERROR")
            self.issues.append("ROS2未安装或未在PATH中")
            return False
        except Exception as e:
            self.log(f"ROS2检查出错: {str(e)}", "ERROR")
            self.issues.append(f"ROS2检查失败: {str(e)}")
            return False

    def check_config_files(self) -> bool:
        """检查配置文件"""
        self.total_checks += 1

        config_files = [
            ".env.example",
            "CLAUDE.md",
            "README.md"
        ]

        missing_files = []
        for file_name in config_files:
            file_path = self.project_root / file_name
            if not file_path.exists():
                missing_files.append(file_name)

        if not missing_files:
            self.log("配置文件完整", "SUCCESS")
            self.success_count += 1
            return True
        else:
            self.log(f"缺少配置文件: {', '.join(missing_files)}", "WARNING")
            self.warnings.append(f"建议添加配置文件: {', '.join(missing_files)}")
            return False

    def run_verification(self) -> Dict[str, Any]:
        """运行完整验证"""
        print("=" * 60)
        print("🔍 XLeRobot 项目状态验证")
        print("=" * 60)

        checks = [
            ("Python版本", self.check_python_version),
            ("ROS2环境", self.check_ros2_environment),
            ("项目结构", self.check_project_structure),
            ("环境变量", self.check_environment_variables),
            ("依赖包", self.check_dependencies),
            ("核心模块", self.check_core_modules),
            ("音频设备", self.check_audio_devices),
            ("ROS2连接", self.check_ros2_connectivity),
            ("配置文件", self.check_config_files),
        ]

        for check_name, check_func in checks:
            print(f"\n📋 检查: {check_name}")
            check_func()

        # 计算结果
        success_rate = (self.success_count / self.total_checks) * 100 if self.total_checks > 0 else 0

        print("\n" + "=" * 60)
        print("📊 验证结果总结")
        print("=" * 60)
        print(f"✅ 通过检查: {self.success_count}/{self.total_checks}")
        print(f"📈 成功率: {success_rate:.1f}%")

        if self.issues:
            print(f"\n❌ 发现问题 ({len(self.issues)}):")
            for issue in self.issues:
                print(f"   • {issue}")

        if self.warnings:
            print(f"\n⚠️ 警告 ({len(self.warnings)}):")
            for warning in self.warnings:
                print(f"   • {warning}")

        # 总体状态
        if success_rate >= 90:
            overall_status = "优秀"
            status_icon = "🏆"
        elif success_rate >= 80:
            overall_status = "良好"
            status_icon = "✅"
        elif success_rate >= 60:
            overall_status = "一般"
            status_icon = "⚠️"
        else:
            overall_status = "需要修复"
            status_icon = "❌"

        print(f"\n{status_icon} 项目状态: {overall_status}")
        print("=" * 60)

        # 生成验证报告
        report = {
            'timestamp': str(os.times()),
            'total_checks': self.total_checks,
            'success_count': self.success_count,
            'success_rate': success_rate,
            'overall_status': overall_status,
            'issues': self.issues,
            'warnings': self.warnings
        }

        return report

    def save_report(self, report: Dict[str, Any]):
        """保存验证报告"""
        import time
        report_path = self.project_root / "docs" / "reports" / f"project_status_verify_{int(time.time())}.json"

        try:
            with open(report_path, 'w', encoding='utf-8') as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            print(f"\n📄 验证报告已保存: {report_path}")
        except Exception as e:
            print(f"❌ 保存报告失败: {str(e)}")

def main():
    """主函数"""
    verifier = ProjectStatusVerifier()
    report = verifier.run_verification()
    verifier.save_report(report)

    # 返回状态码
    return 0 if report['success_rate'] >= 80 else 1

if __name__ == "__main__":
    sys.exit(main())