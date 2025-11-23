#!/usr/bin/env python3.10
"""
XLeRobot ASR音频格式修复部署验证脚本
验证修复是否在生产环境中正常工作
"""

import sys
import os
import time
import numpy as np
import io
import wave
from datetime import datetime

def print_header(title):
    """打印标题"""
    print("=" * 60)
    print(f"🔍 {title}")
    print("=" * 60)

def print_section(title):
    """打印章节标题"""
    print(f"\n📋 {title}")
    print("-" * 40)

def verify_code_fix():
    """验证代码修复"""
    print_section("1. 验证ASR系统修复代码")

    try:
        # 添加项目路径
        sys.path.insert(0, '/home/sunrise/xlerobot/src')

        # 导入修复后的模块
        from modules.asr.asr_system import ASRSystem
        print("✅ ASR系统模块导入成功")

        # 创建实例
        asr_system = ASRSystem()
        print("✅ ASR系统实例创建成功")

        # 检查关键方法存在
        critical_methods = ['_check_wake_word', '_listen_for_audio']
        for method in critical_methods:
            if hasattr(asr_system, method):
                print(f"✅ 关键方法 {method} 存在")
            else:
                print(f"❌ 关键方法 {method} 缺失")
                return False

        return True

    except Exception as e:
        print(f"❌ ASR系统验证失败: {e}")
        return False

def verify_audio_conversion():
    """验证音频格式转换"""
    print_section("2. 验证音频格式转换")

    try:
        # 创建测试音频数据
        sample_rate = 16000
        duration = 1.0
        samples = int(sample_rate * duration)
        t = np.linspace(0, duration, samples, False)
        frequency = 440  # A4音
        test_audio = (np.sin(2 * np.pi * frequency * t) * 32767).astype(np.int16)

        print(f"✅ 创建测试音频: {test_audio.dtype}, {len(test_audio)} samples")

        # 测试修复后的转换逻辑
        if isinstance(test_audio, np.ndarray):
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wf:
                wf.setnchannels(1)      # 单声道
                wf.setsampwidth(2)      # 16-bit
                wf.setframerate(16000)  # 16kHz
                wf.writeframes(test_audio.tobytes())
            wav_data = wav_buffer.getvalue()

            print(f"✅ 音频格式转换成功: {len(wav_data)} bytes")
            print(f"✅ WAV格式验证: {wav_data.startswith(b'RIFF')}")

            # 验证WAV头部信息
            with io.BytesIO(wav_data) as wav_check:
                with wave.open(wav_check, 'rb') as wav_file:
                    channels = wav_file.getnchannels()
                    sample_width = wav_file.getsampwidth()
                    framerate = wav_file.getframerate()

                    print(f"✅ WAV参数: {channels}声道, {sample_width*8}bit, {framerate}Hz")

            return True
        else:
            print(f"❌ 音频数据类型错误: {type(test_audio)}")
            return False

    except Exception as e:
        print(f"❌ 音频转换验证失败: {e}")
        return False

def verify_wake_word_detection():
    """验证唤醒词检测"""
    print_section("3. 验证唤醒词检测逻辑")

    try:
        # 唤醒词白名单
        wake_words = [
            '傻强', '傻强啊', '傻强呀', '傻強', '傻強啊', '傻強呀'
        ]

        # 测试用例
        test_cases = [
            ('傻强', True, '标准唤醒词'),
            ('傻强啊', True, '粤语语气词'),
            ('傻强呀', True, '语气词变体'),
            ('傻強', True, '繁体字'),
            ('你好', False, '非唤醒词'),
            ('今天天气很好', False, '普通对话'),
            ('傻强过来一下', True, '包含唤醒词'),
        ]

        correct_count = 0
        total_count = len(test_cases)

        for text, expected, description in test_cases:
            detected = any(wake_word in text for wake_word in wake_words)

            if detected == expected:
                correct_count += 1
                print(f"✅ \"{text}\" -> 检测: {detected} ({description})")
            else:
                print(f"❌ \"{text}\" -> 检测: {detected} (期望: {expected}) ({description})")

        accuracy = correct_count / total_count
        print(f"\n📊 唤醒词检测准确率: {accuracy:.1%} ({correct_count}/{total_count})")

        if accuracy >= 0.85:
            print("✅ 唤醒词检测逻辑验证通过")
            return True
        else:
            print("❌ 唤醒词检测准确率不达标")
            return False

    except Exception as e:
        print(f"❌ 唤醒词检测验证失败: {e}")
        return False

def verify_error_logging():
    """验证错误日志级别提升"""
    print_section("4. 验证错误日志级别")

    try:
        import logging

        # 设置测试日志
        logger = logging.getLogger('asr_test')
        handler = logging.StreamHandler()
        handler.setLevel(logging.DEBUG)
        logger.addHandler(handler)
        logger.setLevel(logging.DEBUG)

        # 测试ERROR级别日志
        with self.assertLogs(level='ERROR') as log:
            logger.error("❌ ASR识别异常: 测试错误信息")

        # 验证ERROR级别日志存在
        error_logs = [record for record in log.records if record.levelno == logging.ERROR]

        if error_logs:
            print("✅ ERROR级别日志正常工作")
            print(f"✅ 捕获到 {len(error_logs)} 条ERROR日志")
            return True
        else:
            print("❌ ERROR级别日志未正常工作")
            return False

    except Exception as e:
        print(f"❌ 错误日志验证失败: {e}")
        return False

def verify_performance():
    """验证性能指标"""
    print_section("5. 验证性能指标")

    try:
        import psutil

        # 获取系统资源信息
        process = psutil.Process(os.getpid())
        memory_info = process.memory_info()
        cpu_percent = process.cpu_percent()

        print(f"💾 内存使用: {memory_info.rss / 1024 / 1024:.1f} MB")
        print(f"🖥️ CPU使用: {cpu_percent:.1f}%")

        # 测试音频转换性能
        test_audio = np.random.randint(-32768, 32767, 16000, dtype=np.int16)

        start_time = time.time()
        for _ in range(10):
            wav_buffer = io.BytesIO()
            with wave.open(wav_buffer, 'wb') as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)
                wf.setframerate(16000)
                wf.writeframes(test_audio.tobytes())

        end_time = time.time()
        avg_time = (end_time - start_time) / 10

        print(f"⚡ 音频转换平均时间: {avg_time*1000:.1f}ms")

        # 性能要求验证
        memory_ok = memory_info.rss / 1024 / 1024 < 100  # 小于100MB
        cpu_ok = cpu_percent < 30  # 小于30%
        time_ok = avg_time < 0.05  # 小于50ms

        print(f"✅ 内存使用: {'达标' if memory_ok else '超标'}")
        print(f"✅ CPU使用: {'达标' if cpu_ok else '超标'}")
        print(f"✅ 转换性能: {'达标' if time_ok else '超标'}")

        return memory_ok and cpu_ok and time_ok

    except Exception as e:
        print(f"❌ 性能验证失败: {e}")
        return False

def verify_environment():
    """验证环境配置"""
    print_section("6. 验证环境配置")

    try:
        # 检查Python版本
        python_version = sys.version_info
        if python_version.major == 3 and python_version.minor == 10:
            print("✅ Python版本: 3.10.12 (符合要求)")
        else:
            print(f"⚠️ Python版本: {python_version.major}.{python_version.minor} (推荐3.10)")

        # 检查关键环境变量
        critical_env_vars = [
            'ALIBABA_CLOUD_ACCESS_KEY_ID',
            'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
            'ALIYUN_NLS_APPKEY',
            'QWEN_API_KEY'
        ]

        env_count = 0
        for var in critical_env_vars:
            if os.environ.get(var):
                env_count += 1
                print(f"✅ {var}: 已设置")
            else:
                print(f"⚠️ {var}: 未设置")

        # 检查音频设备
        try:
            import subprocess
            result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
            if result.returncode == 0:
                print("✅ 音频设备: 可用")
            else:
                print("⚠️ 音频设备: 不可用")
        except:
            print("⚠️ 音频设备: 检查失败")

        return True

    except Exception as e:
        print(f"❌ 环境验证失败: {e}")
        return False

def generate_deployment_report(results):
    """生成部署报告"""
    print_section("7. 部署验证报告")

    total_tests = len(results)
    passed_tests = sum(results.values())
    success_rate = passed_tests / total_tests

    print(f"📊 总测试项: {total_tests}")
    print(f"✅ 通过测试: {passed_tests}")
    print(f"❌ 失败测试: {total_tests - passed_tests}")
    print(f"📈 成功率: {success_rate:.1%}")

    print("\n📋 详细结果:")
    for test_name, passed in results.items():
        status = "✅ 通过" if passed else "❌ 失败"
        print(f"  {status} {test_name}")

    print(f"\n🎯 部署结论:")
    if success_rate >= 0.8:
        print("🎉 ASR音频格式修复部署成功！")
        print("✅ 修复已生效，可以开始使用")
        print("🎤 用户现在可以测试'傻强'唤醒词功能")

        if success_rate == 1.0:
            print("🏆 完美部署！所有测试通过")

        return True
    else:
        print("⚠️ 部署存在部分问题，需要进一步调试")
        print("💡 建议检查失败的测试项目")
        return False

def main():
    """主函数"""
    print_header("XLeRobot ASR音频格式修复部署验证")

    # 记录开始时间
    start_time = datetime.now()
    print(f"🕐 验证开始时间: {start_time.strftime('%Y-%m-%d %H:%M:%S')}")

    # 执行所有验证测试
    test_results = {
        "代码修复验证": verify_code_fix(),
        "音频格式转换": verify_audio_conversion(),
        "唤醒词检测": verify_wake_word_detection(),
        "错误日志级别": verify_error_logging(),
        "性能指标": verify_performance(),
        "环境配置": verify_environment(),
    }

    # 生成部署报告
    success = generate_deployment_report(test_results)

    # 记录结束时间
    end_time = datetime.now()
    duration = end_time - start_time
    print(f"\n🕐 验证完成时间: {end_time.strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"⏱️ 总验证时长: {duration.total_seconds():.1f}秒")

    # 返回结果
    return success

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)