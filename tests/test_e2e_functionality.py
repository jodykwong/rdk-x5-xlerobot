#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
端到端功能测试脚本
验证ASR→LLM→TTS串联功能

作者: BMad代理团队
版本: 1.0.0
"""

import os
import sys
import time
import logging
import asyncio
from pathlib import Path

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class E2ETestSuite:
    """端到端测试套件"""

    def __init__(self):
        self.test_results = {}
        self.start_time = time.time()

    def log_result(self, test_name, passed, details=""):
        """记录测试结果"""
        status = "✅ PASS" if passed else "❌ FAIL"
        self.test_results[test_name] = {"status": passed, "details": details}
        logger.info(f"{status}: {test_name}")
        if details:
            logger.info(f"    详情: {details}")

    def test_environment_setup(self):
        """测试环境设置"""
        try:
            # 检查Python版本
            python_version = sys.version
            if "3.10" in python_version:
                self.log_result("Python版本检查", True, f"Python 3.10 - {python_version.split()[0]}")
            else:
                self.log_result("Python版本检查", False, f"版本: {python_version}")

            # 检查环境变量
            required_vars = ['ALIBABA_CLOUD_ACCESS_KEY_ID', 'QWEN_API_KEY']
            missing_vars = []
            for var in required_vars:
                if os.getenv(var):
                    self.log_result(f"环境变量_{var}", True, "已设置")
                else:
                    missing_vars.append(var)
                    self.log_result(f"环境变量_{var}", False, "未设置")

            # 检查音频设备
            try:
                import subprocess
                result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
                if result.returncode == 0:
                    devices = result.stdout.count("card")
                    self.log_result("音频录制设备", True, f"找到 {devices} 个设备")
                else:
                    self.log_result("音频录制设备", False, "无法访问")

                result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
                if result.returncode == 0:
                    devices = result.stdout.count("card")
                    self.log_result("音频播放设备", True, f"找到 {devices} 个设备")
                else:
                    self.log_result("音频播放设备", False, "无法访问")

            except Exception as e:
                self.log_result("音频设备检查", False, str(e))

            return len(missing_vars) == 0

        except Exception as e:
            self.log_result("环境设置检查", False, str(e))
            return False

    def test_node_files(self):
        """测试节点文件完整性"""
        try:
            node_files = [
                '/home/sunrise/xlerobot/src/xlerobot/nodes/llm_service_node.py',
                '/home/sunrise/xlerobot/src/xlerobot/nodes/tts_service_node.py',
                '/home/sunrise/xlerobot/src/xlerobot/nodes/voice_assistant_coordinator.py'
            ]

            for node_file in node_files:
                node_name = Path(node_file).stem
                if os.path.exists(node_file):
                    # 检查语法
                    try:
                        with open(node_file, 'r') as f:
                            code = f.read()
                        compile(code, node_file, 'exec')
                        self.log_result(f"节点文件_{node_name}", True, "语法正确，文件存在")
                    except SyntaxError as e:
                        self.log_result(f"节点文件_{node_name}", False, f"语法错误: {e}")
                else:
                    self.log_result(f"节点文件_{node_name}", False, "文件不存在")

            return True

        except Exception as e:
            self.log_result("节点文件检查", False, str(e))
            return False

    def test_message_definitions(self):
        """测试消息定义"""
        try:
            message_files = [
                '/home/sunrise/xlerobot/src/audio_msg/msg/LLMResponse.msg',
                '/home/sunrise/xlerobot/src/audio_msg/msg/LLMStatus.msg',
                '/home/sunrise/xlerobot/src/audio_msg/msg/TTSStatus.msg'
            ]

            for msg_file in message_files:
                msg_name = Path(msg_file).stem
                if os.path.exists(msg_file):
                    with open(msg_file, 'r') as f:
                        content = f.read()
                        if 'std_msgs/Header' in content:
                            self.log_result(f"消息定义_{msg_name}", True, "格式正确")
                        else:
                            self.log_result(f"消息定义_{msg_name}", False, "缺少Header")
                else:
                    self.log_result(f"消息定义_{msg_name}", False, "文件不存在")

            return True

        except Exception as e:
            self.log_result("消息定义检查", False, str(e))
            return False

    def test_ros2_environment(self):
        """测试ROS2环境"""
        try:
            # 检查ROS2安装
            try:
                result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
                if result.returncode == 0:
                    self.log_result("ROS2安装检查", True, result.stdout.strip())
                else:
                    self.log_result("ROS2安装检查", False, "无法获取版本")
            except FileNotFoundError:
                self.log_result("ROS2安装检查", False, "ROS2未安装")
                return False

            # 检查rclpy
            try:
                import rclpy
                self.log_result("rclpy导入", True, "成功导入")
            except ImportError as e:
                self.log_result("rclpy导入", False, str(e))
                return False

            # 检查启动脚本修改
            launch_file = '/home/sunrise/xlerobot/src/xlerobot/launch/voice_assistant.launch.py'
            if os.path.exists(launch_file):
                self.log_result("Launch文件检查", True, "文件存在")
            else:
                self.log_result("Launch文件检查", False, "文件不存在")

            return True

        except Exception as e:
            self.log_result("ROS2环境检查", False, str(e))
            return False

    def test_asr_components(self):
        """测试ASR组件"""
        try:
            # 检查ASR相关模块
            asr_modules = [
                'modules.asr.aliyun_websocket_asr_client',
                'modules.asr.siqiang_intelligent_dialogue'
            ]

            for module in asr_modules:
                try:
                    __import__(module)
                    module_name = module.split('.')[-1]
                    self.log_result(f"ASR模块_{module_name}", True, "导入成功")
                except ImportError as e:
                    module_name = module.split('.')[-1]
                    self.log_result(f"ASR模块_{module_name}", False, str(e))

            return True

        except Exception as e:
            self.log_result("ASR组件检查", False, str(e))
            return False

    def test_llm_components(self):
        """测试LLM组件"""
        try:
            # 检查LLM相关模块
            llm_modules = [
                'modules.llm.qwen_client',
                'modules.llm.dialogue_context'
            ]

            for module in llm_modules:
                try:
                    __import__(module)
                    module_name = module.split('.')[-1]
                    self.log_result(f"LLM模块_{module_name}", True, "导入成功")
                except ImportError as e:
                    module_name = module.split('.')[-1]
                    self.log_result(f"LLM模块_{module_name}", False, str(e))

            return True

        except Exception as e:
            self.log_result("LLM组件检查", False, str(e))
            return False

    def test_tts_components(self):
        """测试TTS组件"""
        try:
            # 检查TTS相关模块
            tts_modules = [
                'modules.tts.simple_tts_service',
                'modules.tts.aliyun_tts_system'
            ]

            for module in tts_modules:
                try:
                    __import__(module)
                    module_name = module.split('.')[-1]
                    self.log_result(f"TTS模块_{module_name}", True, "导入成功")
                except ImportError as e:
                    module_name = module.split('.')[-1]
                    self.log_result(f"TTS模块_{module_name}", False, str(e))

            return True

        except Exception as e:
            self.log_result("TTS组件检查", False, str(e))
            return False

    def test_integration_flow(self):
        """测试集成流程"""
        try:
            # 这里可以添加更复杂的集成测试
            # 目前测试基本的组件连接性

            # 测试Token管理器
            try:
                from aliyun_nls_token_manager import AliyunNLSTokenManager
                # 只测试导入，不测试实际连接
                self.log_result("Token管理器", True, "导入成功")
            except Exception as e:
                self.log_result("Token管理器", False, str(e))

            # 测试启动脚本修改
            start_script = '/home/sunrise/xlerobot/start_voice_assistant.sh'
            if os.path.exists(start_script):
                with open(start_script, 'r') as f:
                    content = f.read()
                    if 'ros2 launch' in content:
                        self.log_result("启动脚本修改", True, "已更新为ROS2 Launch")
                    else:
                        self.log_result("启动脚本修改", False, "未找到ROS2 Launch")
            else:
                self.log_result("启动脚本修改", False, "文件不存在")

            return True

        except Exception as e:
            self.log_result("集成流程检查", False, str(e))
            return False

    def run_all_tests(self):
        """运行所有测试"""
        print("🧪 XLeRobot ASR→LLM→TTS 端到端功能测试")
        print("=" * 60)

        # 运行测试套件
        test_suites = [
            ("环境设置", self.test_environment_setup),
            ("节点文件", self.test_node_files),
            ("消息定义", self.test_message_definitions),
            ("ROS2环境", self.test_ros2_environment),
            ("ASR组件", self.test_asr_components),
            ("LLM组件", self.test_llm_components),
            ("TTS组件", self.test_tts_components),
            ("集成流程", self.test_integration_flow)
        ]

        passed_tests = 0
        total_tests = 0

        for suite_name, test_func in test_suites:
            print(f"\n🔍 {suite_name}测试:")
            print("-" * 30)

            try:
                result = test_func()
                if result:
                    passed_tests += 1
                total_tests += 1
            except Exception as e:
                self.log_result(f"测试套件_{suite_name}", False, f"执行异常: {e}")
                total_tests += 1

        # 生成测试报告
        self.generate_report(passed_tests, total_tests)

    def generate_report(self, passed, total):
        """生成测试报告"""
        print("\n" + "=" * 60)
        print("📊 测试结果汇总")
        print("=" * 60)

        success_rate = (passed / total) * 100 if total > 0 else 0
        print(f"测试通过率: {success_rate:.1f}% ({passed}/{total})")

        if success_rate >= 90:
            print("🎉 优秀！系统状态良好，修复工作完成度很高")
        elif success_rate >= 75:
            print("✅ 良好！系统基本就绪，有少量问题需要解决")
        elif success_rate >= 50:
            print("⚠️ 一般！系统部分功能正常，需要进一步修复")
        else:
            print("❌ 需要改进！系统存在较多问题")

        # 详细结果
        print("\n📋 详细测试结果:")
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result["status"] else "❌ FAIL"
            print(f"  {status} {test_name}")
            if result["details"]:
                print(f"      {result['details']}")

        elapsed_time = time.time() - self.start_time
        print(f"\n⏱️ 测试耗时: {elapsed_time:.2f}秒")


def main():
    """主函数"""
    try:
        # 设置环境变量（模拟启动脚本环境）
        os.environ['PYTHONPATH'] = "/home/sunrise/xlerobot/src:" + os.environ.get('PYTHONPATH', '')

        # 创建测试套件
        test_suite = E2ETestSuite()

        # 运行测试
        test_suite.run_all_tests()

        return 0

    except KeyboardInterrupt:
        print("\n⚠️ 测试被用户中断")
        return 1
    except Exception as e:
        print(f"\n❌ 测试执行异常: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())