#!/usr/bin/env python3.10
"""
WebSocket ASR → TTS 集成测试脚本
===================================

系统性验证重构后的ASR→TTS完整链路
确保WebSocket架构的一致性和稳定性

测试内容：
1. WebSocket ASR服务功能
2. 统一音频处理管道
3. ASR→TTS完整链路
4. 错误处理和恢复机制
5. 性能指标监控

作者: BMad Master + Test Architect Team
版本: 2.0 (WebSocket架构)
日期: 2025-11-14
"""

import os
import sys
import time
import subprocess
import logging
from pathlib import Path

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 设置环境变量
os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"] = "YOUR_ACCESS_KEY_ID"
os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"] = "YOUR_ACCESS_KEY_SECRET"
os.environ["ALIYUN_NLS_APPKEY"] = "YOUR_NLS_APPKEY"

# 添加路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

class IntegratedASRTTSTest:
    """集成ASR→TTS测试类"""

    def __init__(self):
        """初始化测试环境"""
        self.test_results = []
        self.audio_processor = None
        self.asr_service = None
        self.tts_service = None

    def run_all_tests(self):
        """运行所有测试"""
        print("=" * 80)
        print("🧪 WebSocket ASR→TTS 集成测试开始")
        print("=" * 80)

        test_methods = [
            ("环境检查", self.test_environment),
            ("音频处理管道", self.test_audio_processor),
            ("WebSocket ASR服务", self.test_websocket_asr),
            ("TTS服务", self.test_tts_service),
            ("完整链路集成", self.test_complete_pipeline),
            ("性能指标", self.test_performance_metrics),
        ]

        passed = 0
        total = len(test_methods)

        for test_name, test_method in test_methods:
            print(f"\n🔍 测试: {test_name}")
            try:
                result = test_method()
                if result:
                    print(f"✅ {test_name} - 通过")
                    self.test_results.append((test_name, "PASS", None))
                    passed += 1
                else:
                    print(f"❌ {test_name} - 失败")
                    self.test_results.append((test_name, "FAIL", None))
            except Exception as e:
                print(f"❌ {test_name} - 异常: {e}")
                self.test_results.append((test_name, "ERROR", str(e)))

        print(f"\n📊 测试结果: {passed}/{total} 通过")
        self.print_summary()

        return passed == total

    def test_environment(self) -> bool:
        """测试环境配置"""
        try:
            # 检查Python环境
            print(f"   Python版本: {sys.version}")

            # 检查环境变量
            required_vars = ["ALIBABA_CLOUD_ACCESS_KEY_ID", "ALIBABA_CLOUD_ACCESS_KEY_SECRET", "ALIYUN_NLS_APPKEY"]
            for var in required_vars:
                if not os.environ.get(var):
                    print(f"   ❌ 缺少环境变量: {var}")
                    return False
                print(f"   ✅ {var}: {os.environ[var][:10]}...")

            # 检查音频设备
            result = subprocess.run(['arecord', '-l'], capture_output=True, text=True)
            if result.returncode == 0:
                print("   ✅ 音频录制设备可用")
            else:
                print("   ❌ 音频录制设备不可用")
                return False

            result = subprocess.run(['aplay', '-l'], capture_output=True, text=True)
            if result.returncode == 0:
                print("   ✅ 音频播放设备可用")
            else:
                print("   ❌ 音频播放设备不可用")
                return False

            return True

        except Exception as e:
            print(f"   ❌ 环境检查异常: {e}")
            return False

    def test_audio_processor(self) -> bool:
        """测试音频处理管道"""
        try:
            from modules.asr.unified_audio_processor import create_unified_audio_processor

            # 创建音频处理器
            self.audio_processor = create_unified_audio_processor()
            print("   ✅ 音频处理器创建成功")

            # 测试numpy数组处理
            import numpy as np
            test_audio = np.random.randint(-1000, 1000, 16000, dtype=np.int16)  # 1秒测试音频

            processed_audio, audio_info = self.audio_processor.process_audio(test_audio)

            if processed_audio and audio_info:
                print(f"   ✅ 音频处理测试通过:")
                print(f"      采样率: {audio_info.sample_rate}Hz")
                print(f"      声道: {audio_info.channels}")
                print(f"      位深: {audio_info.bits_per_sample}bit")
                print(f"      时长: {audio_info.duration:.2f}s")
                return True
            else:
                print("   ❌ 音频处理失败")
                return False

        except Exception as e:
            print(f"   ❌ 音频处理器测试异常: {e}")
            return False

    def test_websocket_asr(self) -> bool:
        """测试WebSocket ASR服务"""
        try:
            from modules.asr.websocket_asr_service import create_websocket_asr_service

            # 创建ASR服务
            self.asr_service = create_websocket_asr_service(enable_optimization=True)
            print("   ✅ WebSocket ASR服务创建成功")

            # 健康检查
            health = self.asr_service.health_check()
            print(f"   健康状态: {health['status']}")
            if health['status'] == 'healthy':
                print(f"   ✅ Token有效: {health['token_valid']}")
                print(f"   ✅ SDK可用: {health['sdk_available']}")
                print(f"   ✅ 服务已初始化: {health['service_initialized']}")
                return True
            else:
                print(f"   ❌ 服务不健康: {health}")
                return False

        except Exception as e:
            print(f"   ❌ WebSocket ASR服务测试异常: {e}")
            return False

    def test_tts_service(self) -> bool:
        """测试TTS服务"""
        try:
            from modules.tts.engine.aliyun_tts_client import AliyunTTSClient

            # 创建TTS服务
            self.tts_service = AliyunTTSClient()
            print("   ✅ TTS服务创建成功")

            # 测试语音合成
            test_text = "WebSocket重构测试成功"
            tts_audio = self.tts_service.synthesize(test_text, voice="sijia")

            if tts_audio:
                print(f"   ✅ TTS合成成功: {len(tts_audio)} 字节")
                return True
            else:
                print("   ❌ TTS合成失败")
                return False

        except Exception as e:
            print(f"   ❌ TTS服务测试异常: {e}")
            return False

    def test_complete_pipeline(self) -> bool:
        """测试完整链路"""
        try:
            print("   🎤 准备录制测试音频...")
            print("   💬 请说粤语: '你好啊WebSocket' (3秒)")
            time.sleep(1)

            # 录制音频
            audio_file = "/tmp/websocket_test.wav"
            result = subprocess.run([
                'arecord', '-D', 'hw:0,0',
                '-f', 'S16_LE',
                '-r', '16000',
                '-c', '1',
                '-d', '3',
                audio_file
            ], capture_output=True, text=True, timeout=10)

            if result.returncode != 0:
                print(f"   ❌ 录音失败: {result.stderr}")
                return False

            file_size = os.path.getsize(audio_file)
            print(f"   ✅ 录音完成: {file_size} 字节")

            # 播放录音确认
            print("   🔊 播放录音确认...")
            subprocess.run(['aplay', audio_file], capture_output=True, timeout=5)

            # ASR识别
            print("   🧠 进行ASR识别...")
            with open(audio_file, 'rb') as f:
                audio_data = f.read()

            # 使用音频处理器预处理
            processed_audio, audio_info = self.audio_processor.process_audio(audio_data)
            if not processed_audio:
                print("   ❌ 音频预处理失败")
                return False

            print(f"   📊 音频信息: {audio_info.duration:.2f}s, {audio_info.channels}ch")

            # WebSocket ASR识别
            asr_result = self.asr_service.recognize_speech(
                processed_audio,
                language="cn-cantonese"
            )

            if not asr_result.success:
                print(f"   ❌ ASR识别失败: {asr_result.error}")
                return False

            print(f"   ✅ ASR识别成功: '{asr_result.text}' (置信度: {asr_result.confidence}%)")

            # TTS合成
            print("   🔊 进行TTS合成...")
            response_text = f"好嘅，我听到你讲：{asr_result.text}"
            tts_audio = self.tts_service.synthesize(response_text, voice="sijia")

            if not tts_audio:
                print("   ❌ TTS合成失败")
                return False

            # 保存TTS音频
            tts_file = "/tmp/websocket_tts.wav"
            with open(tts_file, 'wb') as f:
                f.write(tts_audio)

            tts_size = os.path.getsize(tts_file)
            print(f"   ✅ TTS合成成功: {tts_size} 字节")

            # 播放TTS音频
            print("   🔊 播放TTS合成语音...")
            result = subprocess.run(['aplay', tts_file], capture_output=True, timeout=5)

            if result.returncode == 0:
                print("   ✅ 播放成功")
            else:
                print(f"   ⚠️ 播放失败: {result.stderr}")

            return True

        except Exception as e:
            print(f"   ❌ 完整链路测试异常: {e}")
            return False

    def test_performance_metrics(self) -> bool:
        """测试性能指标"""
        try:
            if self.asr_service:
                metrics = self.asr_service.get_metrics()
                print(f"   📊 ASR性能指标:")
                print(f"      总请求数: {metrics.total_requests}")
                print(f"      成功请求: {metrics.successful_requests}")
                print(f"      失败请求: {metrics.failed_requests}")
                print(f"      平均响应时间: {metrics.average_response_time:.2f}s")
                return True
            else:
                print("   ⚠️ ASR服务未创建，跳过性能指标测试")
                return True

        except Exception as e:
            print(f"   ❌ 性能指标测试异常: {e}")
            return False

    def print_summary(self):
        """打印测试总结"""
        print("\n" + "=" * 80)
        print("📋 测试结果详细报告")
        print("=" * 80)

        for test_name, status, error in self.test_results:
            status_icon = "✅" if status == "PASS" else "❌"
            print(f"{status_icon} {test_name}: {status}")
            if error:
                print(f"   错误: {error}")

        print("\n" + "=" * 80)

        # 统计信息
        passed = sum(1 for _, status, _ in self.test_results if status == "PASS")
        total = len(self.test_results)
        success_rate = (passed / total) * 100 if total > 0 else 0

        print(f"📊 测试统计:")
        print(f"   总测试数: {total}")
        print(f"   通过: {passed}")
        print(f"   失败: {total - passed}")
        print(f"   成功率: {success_rate:.1f}%")

        if passed == total:
            print("\n🎉 所有测试通过！WebSocket ASR→TTS重构成功！")
        else:
            print(f"\n⚠️ {total - passed} 个测试失败，需要进一步检查")

def main():
    """主函数"""
    print("🚀 启动WebSocket ASR→TTS集成测试")

    test_suite = IntegratedASRTTSTest()
    success = test_suite.run_all_tests()

    return 0 if success else 1

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)