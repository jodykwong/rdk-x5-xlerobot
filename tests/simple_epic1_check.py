#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Epic 1 完成度快速验证脚本
检查各Story的代码实现和配置状态
"""

import os
import sys
import subprocess
from pathlib import Path

def check_story_exists(story_name, file_patterns):
    """检查Story相关文件是否存在"""
    print(f"\n🔍 检查 {story_name}")

    found_files = []
    src_path = Path("/home/sunrise/xlerobot/src")

    for pattern in file_patterns:
        for file_path in src_path.rglob(pattern):
            if file_path.is_file():
                relative_path = file_path.relative_to(src_path)
                found_files.append(str(relative_path))
                print(f"  ✅ {relative_path}")

    if found_files:
        print(f"  📊 找到 {len(found_files)} 个文件")
        return True, found_files
    else:
        print(f"  ❌ 未找到相关文件")
        return False, []

def check_audio_hardware():
    """检查音频硬件"""
    print("\n🎤 检查音频硬件")

    try:
        result = subprocess.run(['arecord', '-l'], capture_output=True, text=True, timeout=5)
        if 'USB Audio' in result.stdout:
            print("  ✅ USB音频设备正常")
            return True
        else:
            print("  ❌ 未检测到USB音频设备")
            return False
    except Exception as e:
        print(f"  ❌ 音频硬件检查失败: {e}")
        return False

def check_ros2_environment():
    """检查ROS2环境"""
    print("\n🤖 检查ROS2环境")

    try:
        result = subprocess.run([
            'python3', '-c',
            'import rclpy; from audio_msg.msg import AudioFrame; print("OK")'
        ], capture_output=True, text=True, timeout=10, env={
            **os.environ,
            'PYTHONPATH': '/home/sunrise/xlerobot/src'
        })

        if result.returncode == 0:
            print("  ✅ ROS2环境正常")
            return True
        else:
            print(f"  ❌ ROS2环境异常: {result.stderr}")
            return False
    except Exception as e:
        print(f"  ❌ ROS2环境检查失败: {e}")
        return False

def check_aliyun_api():
    """检查阿里云API配置"""
    print("\n🔑 检查阿里云API配置")

    # 检查环境变量
    access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID')
    access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET')

    if access_key_id and access_key_secret:
        print("  ✅ 环境变量API凭证已配置")
        return True
    else:
        print("  ⚠️ 环境变量API凭证未配置")

        # 检查配置文件
        config_files = [
            "/home/sunrise/xlerobot/fixed_aliyun_config.py",
            "/home/sunrise/xlerobot/config/aliyun_config.py"
        ]

        for config_file in config_files:
            if Path(config_file).exists():
                print(f"  ✅ 找到配置文件: {config_file}")
                return True

        print("  ❌ 未找到API配置")
        return False

def main():
    print("🚀 Epic 1 完成度快速验证")
    print("=" * 50)

    results = {}

    # 检查各Story
    stories = {
        "Story 1.1 (音频采集)": ["*audio_input*", "*enhanced_audio*"],
        "Story 1.2 (语音唤醒)": ["*wake_word*", "*cantonese*", "*aliyun_asr*"],
        "Story 1.3 (语音识别)": ["*asr_client*", "*demo_story_1_3*"],
        "Story 1.4 (语音合成)": ["*tts_client*", "*audio_processor*"]
    }

    for story_name, patterns in stories.items():
        exists, files = check_story_exists(story_name, patterns)
        results[story_name] = {"exists": exists, "files": files}

    # 检查基础设施
    audio_ok = check_audio_hardware()
    ros2_ok = check_ros2_environment()
    api_ok = check_aliyun_api()

    # 汇总结果
    print("\n" + "=" * 50)
    print("📊 验证结果汇总")
    print("=" * 50)

    completed_stories = 0
    total_stories = len(stories)

    for story_name, result in results.items():
        status = "✅ 完成" if result["exists"] else "❌ 缺失"
        print(f"{story_name}: {status} ({len(result['files'])} 个文件)")
        if result["exists"]:
            completed_stories += 1

    print(f"\n🎯 Epic 1 总体状态:")
    print(f"  完成度: {completed_stories}/{total_stories} Stories")
    print(f"  音频硬件: {'✅ 正常' if audio_ok else '❌ 异常'}")
    print(f"  ROS2环境: {'✅ 正常' if ros2_ok else '❌ 异常'}")
    print(f"  API配置: {'✅ 正常' if api_ok else '⚠️ 需要配置'}")

    # 计算总体评分
    infrastructure_score = sum([audio_ok, ros2_ok, api_ok]) / 3 * 100
    story_score = completed_stories / total_stories * 100
    overall_score = (infrastructure_score + story_score) / 2

    print(f"\n🏆 总体评分: {overall_score:.1f}/100")

    if completed_stories == total_stories and audio_ok and ros2_ok:
        print("\n✅ Epic 1 基本完成，可以进行端到端测试")
        if api_ok:
            print("✅ API配置完整，可以进行完整功能验证")
        else:
            print("⚠️ 需要配置阿里云API凭证才能完整验证")
    else:
        print("\n⚠️ Epic 1 尚未完全完成，需要继续开发")

    return results

if __name__ == "__main__":
    main()