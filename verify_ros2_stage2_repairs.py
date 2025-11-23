#!/usr/bin/env python3.10
"""
verify_ros2_stage2_repairs.py
Comprehensive verification of Stage 2 ROS2 communication repairs
"""

import os
import sys
import time
import subprocess
import asyncio
import logging
import re
from pathlib import Path

class ROS2Stage2Verification:
    def __init__(self):
        self.project_root = Path(__file__).parent
        self.verification_results = {}

    def log(self, message, level="INFO"):
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] [{level}] {message}")

    async def verify_launch_file_syntax(self):
        """Verify launch file syntax and structure"""
        self.log("🔍 验证Launch文件语法和结构...")

        try:
            # 检查Launch文件存在
            launch_file = self.project_root / "src" / "xlerobot" / "launch" / "voice_assistant.launch.py"
            if not launch_file.exists():
                return False, "Launch文件不存在"

            # 检查Python语法
            with open(launch_file, 'r', encoding='utf-8') as f:
                code = f.read()
            compile(code, str(launch_file), 'exec')
            self.log("✅ Launch文件Python语法: PASS")

            # 统计节点定义
            node_count = len(re.findall(r'\bNode\(', code))
            timer_count = len(re.findall(r'\bTimerAction\(', code))
            env_count = len(re.findall(r'\bSetEnvironmentVariable\(', code))

            self.log(f"✅ 找到 {node_count} 个节点定义")
            self.log(f"✅ 找到 {timer_count} 个定时器动作")
            self.log(f"✅ 找到 {env_count} 个环境变量设置")

            # 验证启动延迟配置
            delays = re.findall(r'period=([0-9.]+)', code)
            self.log(f"✅ 延迟配置: {delays}")

            if node_count == 4 and timer_count == 3:
                return True, f"Launch文件结构正确，{node_count}个节点，{timer_count}个定时器"
            else:
                return False, f"Launch文件结构异常，预期4个节点3个定时器，实际{node_count}个节点{timer_count}个定时器"

        except SyntaxError as e:
            return False, f"Launch文件语法错误: {e}"
        except Exception as e:
            return False, f"Launch文件验证异常: {e}"

    async def verify_node_topic_mapping(self):
        """Verify node topic mapping"""
        self.log("🔍 验证节点主题映射...")

        try:
            # 检查ASR Bridge Node
            asr_file = self.project_root / "src" / "xlerobot" / "nodes" / "asr_bridge_node.py"
            if asr_file.exists():
                with open(asr_file, 'r', encoding='utf-8') as f:
                    asr_content = f.read()

                asr_publishers = re.findall(r'self\.create_publisher\([^,]+,\s*[\'"]([^\'"]+)[\'"]', asr_content)
                self.log(f"   ASR发布者: {asr_publishers}")

                required_topics = ['/voice_command', '/voice_command_string', '/asr/status']
                missing_asr = [t for t in required_topics if t not in asr_publishers]
                if missing_asr:
                    self.log(f"   ⚠️ ASR缺失主题: {missing_asr}")
                else:
                    self.log("   ✅ ASR主题映射正确")

            # 检查LLM Service Node
            llm_file = self.project_root / "src" / "xlerobot" / "nodes" / "llm_service_node.py"
            if llm_file.exists():
                with open(llm_file, 'r', encoding='utf-8') as f:
                    llm_content = f.read()

                llm_publishers = re.findall(r'self\.create_publisher\([^,]+,\s*[\'"]([^\'"]+)[\'"]', llm_content)
                llm_subscribers = re.findall(r'self\.create_subscription\([^,]+,\s*[\'"]([^\'"]+)[\'"]', llm_content)
                self.log(f"   LLM发布者: {llm_publishers}")
                self.log(f"   LLM订阅者: {llm_subscribers}")

                if '/voice_command' in llm_subscribers and '/llm_response' in llm_publishers:
                    self.log("   ✅ LLM主题映射正确")
                else:
                    self.log("   ⚠️ LLM主题映射不完整")

            # 检查TTS Service Node
            tts_file = self.project_root / "src" / "xlerobot" / "nodes" / "tts_service_node.py"
            if tts_file.exists():
                with open(tts_file, 'r', encoding='utf-8') as f:
                    tts_content = f.read()

                tts_subscribers = re.findall(r'self\.create_subscription\([^,]+,\s*[\'"]([^\'"]+)[\'"]', tts_content)
                self.log(f"   TTS订阅者: {tts_subscribers}")

                if '/tts_request' in tts_subscribers:
                    self.log("   ✅ TTS主题映射正确")
                else:
                    self.log("   ⚠️ TTS主题映射不完整")

            # 检查Coordinator Node
            coord_file = self.project_root / "src" / "xlerobot" / "nodes" / "voice_assistant_coordinator.py"
            if coord_file.exists():
                with open(coord_file, 'r', encoding='utf-8') as f:
                    coord_content = f.read()

                coord_publishers = re.findall(r'self\.create_publisher\([^,]+,\s*[\'"]([^\'"]+)[\'"]', coord_content)
                coord_subscribers = re.findall(r'self\.create_subscription\([^,]+,\s*[\'"]([^\'"]+)[\'"]', coord_content)
                self.log(f"   Coordinator发布者: {coord_publishers}")
                self.log(f"   Coordinator订阅者: {coord_subscribers}")

                if '/llm_request' in coord_publishers and '/tts_request' in coord_publishers:
                    self.log("   ✅ Coordinator主题映射正确")
                else:
                    self.log("   ⚠️ Coordinator主题映射不完整")

            return True, "节点主题映射验证完成"

        except Exception as e:
            return False, f"主题映射验证异常: {e}"

    async def verify_message_types(self):
        """Verify message type consistency"""
        self.log("🔍 验证消息类型一致性...")

        try:
            nodes = [
                'asr_bridge_node.py',
                'llm_service_node.py',
                'tts_service_node.py',
                'voice_assistant_coordinator.py'
            ]

            results = {}
            for node in nodes:
                filepath = self.project_root / "src" / "xlerobot" / "nodes" / node
                if filepath.exists():
                    with open(filepath, 'r', encoding='utf-8') as f:
                        content = f.read()

                    results[node] = {
                        'has_string': 'from std_msgs.msg import String' in content,
                        'has_header': 'from std_msgs.msg import Header' in content,
                        'has_audio_msg': 'from audio_msg.msg import' in content
                    }

            # 检查Vision节点
            vision_file = self.project_root / "src" / "xlerobot_vision" / "vision_llm_node.py"
            if vision_file.exists():
                with open(vision_file, 'r', encoding='utf-8') as f:
                    vision_content = f.read()

                results['vision_llm_node.py'] = {
                    'has_string': 'from std_msgs.msg import String' in vision_content,
                    'has_header': 'from std_msgs.msg import Header' in vision_content,
                }

            # 统计结果
            string_count = sum(1 for r in results.values() if r['has_string'])
            header_count = sum(1 for r in results.values() if r['has_header'])
            audio_count = sum(1 for r in results.values() if r.get('has_audio_msg', False))

            self.log(f"   std_msgs.String支持: {string_count}/{len(results)} 个节点")
            self.log(f"   std_msgs.Header支持: {header_count}/{len(results)} 个节点")
            self.log(f"   audio_msg支持: {audio_count}/{len(results)} 个节点")

            if string_count >= 4 and header_count >= 4:
                return True, "消息类型一致性良好"
            else:
                return False, f"消息类型不完整，String支持{string_count}个，Header支持{header_count}个"

        except Exception as e:
            return False, f"消息类型验证异常: {e}"

    async def verify_environment_setup(self):
        """Verify environment setup"""
        self.log("🔍 验证环境配置...")

        try:
            # 检查Python版本
            result = subprocess.run([sys.executable, "--version"], capture_output=True, text=True)
            python_version = result.stdout.strip()
            self.log(f"   Python版本: {python_version}")

            # 检查ROS环境
            ros_distro = os.environ.get('ROS_DISTRO', '未设置')
            python_path = os.environ.get('PYTHONPATH', '未设置')

            self.log(f"   ROS发行版: {ros_distro}")
            self.log(f"   PYTHONPATH: {python_path[:100]}..." if len(python_path) > 100 else f"   PYTHONPATH: {python_path}")

            # 检查关键环境变量
            required_vars = ['ALIBABA_CLOUD_ACCESS_KEY_ID', 'ALIBABA_CLOUD_ACCESS_KEY_SECRET', 'ALIYUN_NLS_APPKEY', 'QWEN_API_KEY']
            var_count = 0
            for var in required_vars:
                value = os.environ.get(var, '未设置')
                if value != '未设置':
                    var_count += 1

            self.log(f"   关键环境变量: {var_count}/{len(required_vars)} 个已设置")

            if '3.10' in python_version and ros_distro == 'humble' and var_count >= 3:
                return True, "环境配置正确"
            else:
                return False, f"环境配置不完整，Python:{python_version}, ROS:{ros_distro}, 环境变量:{var_count}/{len(required_vars)}"

        except Exception as e:
            return False, f"环境验证异常: {e}"

    async def verify_code_structure(self):
        """Verify code structure and imports"""
        self.log("🔍 验证代码结构和导入...")

        try:
            # 检查关键文件存在
            critical_files = [
                'src/xlerobot/launch/voice_assistant.launch.py',
                'src/xlerobot/nodes/voice_assistant_coordinator.py',
                'src/xlerobot/nodes/asr_bridge_node.py',
                'src/xlerobot/nodes/llm_service_node.py',
                'src/xlerobot/nodes/tts_service_node.py',
                'src/xlerobot_vision/vision_llm_node.py'
            ]

            file_count = 0
            for file_path in critical_files:
                full_path = self.project_root / file_path
                if full_path.exists():
                    file_count += 1
                    self.log(f"   ✅ {file_path}")
                else:
                    self.log(f"   ❌ {file_path} 不存在")

            # 检查消息定义文件
            msg_files = list((self.project_root / "src" / "audio_msg" / "msg").glob("*.msg"))
            self.log(f"   自定义消息文件: {len(msg_files)} 个")

            if file_count >= 5 and len(msg_files) >= 5:
                return True, f"代码结构完整，{file_count}/{len(critical_files)}个关键文件存在"
            else:
                return False, f"代码结构不完整，{file_count}/{len(critical_files)}个文件存在"

        except Exception as e:
            return False, f"代码结构验证异常: {e}"

    async def run_complete_verification(self):
        """Run complete Stage 2 verification"""
        self.log("🎯 开始ROS2阶段二修复完整验证...")

        tests = [
            ("Launch文件语法", self.verify_launch_file_syntax),
            ("节点主题映射", self.verify_node_topic_mapping),
            ("消息类型一致性", self.verify_message_types),
            ("环境配置", self.verify_environment_setup),
            ("代码结构", self.verify_code_structure),
        ]

        results = {}
        passed = 0

        for test_name, test_func in tests:
            self.log(f"\n📋 运行测试: {test_name}")
            try:
                success, message = await test_func()
                results[test_name] = {
                    "status": "PASS" if success else "FAIL",
                    "message": message
                }
                if success:
                    passed += 1
                    self.log(f"✅ {test_name}: PASS")
                    self.log(f"   详情: {message}")
                else:
                    self.log(f"❌ {test_name}: FAIL")
                    self.log(f"   详情: {message}")
            except Exception as e:
                results[test_name] = {
                    "status": "ERROR",
                    "message": str(e)
                }
                self.log(f"💥 {test_name}: ERROR - {str(e)}")

        # 总结
        total = len(tests)
        success_rate = (passed / total) * 100

        self.log(f"\n📊 验证总结:")
        self.log(f"✅ 通过: {passed}/{total}")
        self.log(f"📈 成功率: {success_rate:.1f}%")

        if success_rate >= 80:
            self.log(f"🎉 整体状态: PASS - ROS2阶段二修复验证通过")
        else:
            self.log(f"❌ 整体状态: FAIL - ROS2阶段二修复需要进一步工作")

        # 详细结果
        self.log(f"\n📋 详细验证结果:")
        for test_name, result in results.items():
            status_icon = "✅" if result["status"] == "PASS" else "❌"
            self.log(f"{status_icon} {test_name}: {result['status']}")
            if result["status"] != "PASS":
                self.log(f"   └─ {result['message']}")

        return results, success_rate >= 80

if __name__ == "__main__":
    async def main():
        verifier = ROS2Stage2Verification()
        results, success = await verifier.run_complete_verification()

        if success:
            print("\n🎉 ROS2阶段二修复验证完成 - 通过")
            sys.exit(0)
        else:
            print("\n❌ ROS2阶段二修复验证未通过 - 需要修复")
            sys.exit(1)

    asyncio.run(main())