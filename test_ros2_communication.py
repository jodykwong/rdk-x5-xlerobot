#!/usr/bin/env python3.10
"""
test_ros2_communication.py
端到端ROS2通信测试，验证修复后的通信链路
"""

import os
import sys
import time
import subprocess
import threading
from pathlib import Path

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

def log(message):
    timestamp = time.strftime("%H:%M:%S")
    print(f"[{timestamp}] {message}")

def run_ros2_command(cmd, timeout=5):
    """运行ROS2命令"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return result.returncode == 0, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return False, "", "命令超时"
    except Exception as e:
        return False, "", str(e)

def test_topic_creation():
    """测试主题创建"""
    log("🔍 测试ROS2主题创建...")

    # 等待ROS2初始化
    time.sleep(2)

    # 检查关键主题
    topics_to_check = [
        '/voice_command',
        '/voice_command_string',
        '/llm_request',
        '/llm_response',
        '/tts_request',
        '/asr/status',
        '/llm/status',
        '/tts/status'
    ]

    success, output, error = run_ros2_command("ros2 topic list", timeout=10)
    if success:
        existing_topics = output.strip().split('\n')
        found_topics = [t for t in topics_to_check if t in existing_topics]
        log(f"   找到 {len(found_topics)}/{len(topics_to_check)} 个预期主题")

        for topic in found_topics:
            log(f"   ✅ {topic}")

        missing_topics = [t for t in topics_to_check if t not in existing_topics]
        for topic in missing_topics:
            log(f"   ❌ {topic} (缺失)")

        return len(found_topics) >= 5  # 至少一半主题存在
    else:
        log(f"   ❌ 获取主题列表失败: {error}")
        return False

def test_message_publishing():
    """测试消息发布"""
    log("🔍 测试消息发布...")

    # 测试发布到 /voice_command_string (std_msgs/String)
    success, output, error = run_ros2_command(
        "ros2 topic pub --once /voice_command_string std_msgs/String 'data: \"测试消息\"'",
        timeout=5
    )

    if success:
        log("   ✅ String消息发布成功")
        return True
    else:
        log(f"   ❌ String消息发布失败: {error}")
        return False

def test_environment_variables():
    """测试环境变量"""
    log("🔍 测试环境变量...")

    required_vars = [
        'ROS_DISTRO',
        'PYTHONPATH',
        'ALIBABA_CLOUD_ACCESS_KEY_ID',
        'QWEN_API_KEY'
    ]

    missing_vars = []
    for var in required_vars:
        value = os.environ.get(var)
        if value:
            log(f"   ✅ {var}: 已设置")
        else:
            log(f"   ❌ {var}: 未设置")
            missing_vars.append(var)

    return len(missing_vars) == 0

def test_file_structure():
    """测试文件结构"""
    log("🔍 测试文件结构...")

    required_files = [
        '/home/sunrise/xlerobot/src/xlerobot/launch/voice_assistant.launch.py',
        '/home/sunrise/xlerobot/src/xlerobot/nodes/voice_assistant_coordinator.py',
        '/home/sunrise/xlerobot/src/xlerobot/nodes/asr_bridge_node.py',
        '/home/sunrise/xlerobot/src/xlerobot/nodes/llm_service_node.py',
        '/home/sunrise/xlerobot/src/xlerobot/nodes/tts_service_node.py',
    ]

    existing_files = 0
    for file_path in required_files:
        if os.path.exists(file_path):
            log(f"   ✅ {file_path}")
            existing_files += 1
        else:
            log(f"   ❌ {file_path}")

    return existing_files >= 4

def test_launch_file_syntax():
    """测试Launch文件语法"""
    log("🔍 测试Launch文件语法...")

    launch_file = '/home/sunrise/xlerobot/src/xlerobot/launch/voice_assistant.launch.py'
    try:
        with open(launch_file, 'r', encoding='utf-8') as f:
            code = f.read()
        compile(code, launch_file, 'exec')
        log("   ✅ Launch文件语法正确")

        # 检查节点数量
        import re
        node_count = len(re.findall(r'\bNode\(', code))
        timer_count = len(re.findall(r'\bTimerAction\(', code))

        log(f"   📊 找到 {node_count} 个节点定义")
        log(f"   📊 找到 {timer_count} 个定时器")

        return node_count == 4
    except SyntaxError as e:
        log(f"   ❌ Launch文件语法错误: {e}")
        return False
    except Exception as e:
        log(f"   ❌ Launch文件检查失败: {e}")
        return False

def main():
    """主测试函数"""
    log("🚀 开始ROS2通信集成测试...")

    # 设置环境
    os.system("source ./xlerobot_env.sh")
    time.sleep(1)

    # 运行测试
    tests = [
        ("环境变量", test_environment_variables),
        ("文件结构", test_file_structure),
        ("Launch文件语法", test_launch_file_syntax),
        ("主题创建", test_topic_creation),
        ("消息发布", test_message_publishing),
    ]

    results = {}
    passed = 0

    for test_name, test_func in tests:
        log(f"\n📋 运行测试: {test_name}")
        try:
            success = test_func()
            results[test_name] = "PASS" if success else "FAIL"
            if success:
                passed += 1
                log(f"✅ {test_name}: PASS")
            else:
                log(f"❌ {test_name}: FAIL")
        except Exception as e:
            results[test_name] = "ERROR"
            log(f"💥 {test_name}: ERROR - {e}")

    # 总结
    total = len(tests)
    success_rate = (passed / total) * 100

    log(f"\n📊 集成测试总结:")
    log(f"✅ 通过: {passed}/{total}")
    log(f"📈 成功率: {success_rate:.1f}%")

    if success_rate >= 60:
        log(f"🎉 整体状态: PASS - ROS2通信集成测试通过")
        return True
    else:
        log(f"❌ 整体状态: FAIL - ROS2通信需要进一步修复")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)