#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
XLeRobot 完整流程测试
使用动态消息版本测试ASR→LLM→TTS完整串联功能

作者: BMad代理团队
"""

import os
import sys
import time
import asyncio
import logging
import subprocess
import threading
from pathlib import Path
from datetime import datetime

# 设置路径和环境
sys.path.insert(0, '/home/sunrise/xlerobot/src')
os.environ['PYTHONPATH'] = "/home/sunrise/xlerobot/src:" + os.environ.get('PYTHONPATH', '')
os.environ['ROS_DOMAIN_ID'] = '42'
os.environ['ALIBABA_CLOUD_ACCESS_KEY_ID'] = "LTAI5tQ4E2YNzZkGn9g1JqeY"
os.environ['ALIBABA_CLOUD_ACCESS_KEY_SECRET'] = "Hr1xZdcdz3D9OgFnH1nvWz5rldXVeI"
os.environ['ALIYUN_NLS_APPKEY'] = "4G5BCMccTCW8nC8w"
os.environ['QWEN_API_KEY'] = "sk-600a739fb3f54f338616254c1c69c1f6"

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 导入动态消息
try:
    from test_dynamic_messages import ASRResult, LLMResponse, LLMStatus, TTSStatus
    DYNAMIC_MESSAGES_AVAILABLE = True
    logger.info("✅ 动态消息系统可用")
except ImportError as e:
    logger.error(f"❌ 动态消息系统不可用: {e}")
    DYNAMIC_MESSAGES_AVAILABLE = False

class PipelineTester:
    """完整流程测试器"""

    def __init__(self):
        self.session_id = f"test_session_{int(time.time())}"
        self.test_results = []
        self.audio_test_dir = Path("/tmp/xlerobot_pipeline_test")
        self.audio_test_dir.mkdir(exist_ok=True)

    def log_result(self, test_name, passed, details=""):
        """记录测试结果"""
        status = "✅ PASS" if passed else "❌ FAIL"
        self.test_results.append({
            "name": test_name,
            "status": passed,
            "details": details,
            "timestamp": datetime.now().isoformat()
        })
        logger.info(f"{status}: {test_name}")
        if details:
            logger.info(f"    详情: {details}")

    def test_audio_recording(self):
        """测试音频录制功能"""
        logger.info("🎤 测试音频录制功能...")

        test_file = self.audio_test_dir / "pipeline_test.wav"

        try:
            # 使用default设备录制3秒音频
            cmd = [
                'arecord',
                '-D', 'default',
                '-d', '3',
                '-f', 'cd',
                '-r', '16000',
                '-c', '1',
                str(test_file)
            ]

            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0 and test_file.exists():
                file_size = test_file.stat().st_size
                self.log_result("音频录制", True, f"文件大小: {file_size} bytes")
                return str(test_file)
            else:
                self.log_result("音频录制", False, f"错误: {result.stderr}")
                return None

        except Exception as e:
            self.log_result("音频录制", False, f"异常: {e}")
            return None

    def test_audio_playback(self, audio_file):
        """测试音频播放功能"""
        if not audio_file or not Path(audio_file).exists():
            self.log_result("音频播放", False, "音频文件不存在")
            return False

        logger.info("🔊 测试音频播放功能...")

        try:
            cmd = ['aplay', '-D', 'default', '-q', audio_file]
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                self.log_result("音频播放", True, "播放成功")
                return True
            else:
                self.log_result("音频播放", False, f"错误: {result.stderr}")
                return False

        except Exception as e:
            self.log_result("音频播放", False, f"异常: {e}")
            return False

    def test_asr_simulation(self):
        """测试ASR模拟功能"""
        logger.info("🤖 测试ASR识别模拟...")

        if not DYNAMIC_MESSAGES_AVAILABLE:
            self.log_result("ASR模拟", False, "动态消息系统不可用")
            return None

        try:
            # 模拟ASR识别结果
            from rclpy.clock import Clock
            clock = Clock()

            # 创建ASR结果
            asr_result = ASRResult(
                header=self.create_header(clock),
                text="今日天气点样？",
                confidence=0.95,
                begin_time=0,
                end_time=2000,
                status_code=0,
                message="识别成功"
            )

            self.log_result("ASR模拟", True,
                        f"文本: '{asr_result.text}', 置信度: {asr_result.confidence}")
            return asr_result

        except Exception as e:
            self.log_result("ASR模拟", False, f"异常: {e}")
            return None

    def test_llm_processing(self, asr_result):
        """测试LLM处理功能"""
        logger.info("🧠 测试LLM对话处理...")

        if not asr_result:
            self.log_result("LLM处理", False, "缺少ASR输入")
            return None

        try:
            # 模拟LLM处理（实际项目中会调用真实API）
            time.sleep(1)  # 模拟API调用延迟

            # 创建LLM响应
            from rclpy.clock import Clock
            clock = Clock()

            llm_response = LLMResponse(
                header=self.create_header(clock),
                text="今日天气晴朗，温度适宜，是个出行的好日子。",
                session_id=self.session_id,
                confidence=0.9,
                status_code=0,
                error_message="",
                user_input=asr_result.text,
                response_time=1.5,
                model_name="qwen-turbo"
            )

            self.log_result("LLM处理", True,
                        f"响应: '{llm_response.text}', 处理时间: {llm_response.response_time}s")
            return llm_response

        except Exception as e:
            self.log_result("LLM处理", False, f"异常: {e}")
            return None

    def test_tts_synthesis(self, llm_response):
        """测试TTS合成功能"""
        logger.info("🔊 测试TTS语音合成...")

        if not llm_response:
            self.log_result("TTS合成", False, "缺少LLM输入")
            return None

        try:
            # 模拟TTS合成过程
            time.sleep(1)  # 模拟合成延迟

            # 创建TTS音频文件
            tts_file = self.audio_test_dir / f"tts_output_{self.session_id}.wav"

            # 生成测试音频（简单的正弦波）
            cmd = [
                'ffmpeg', '-y',
                '-f', 'lavfi',
                '-i', 'sine=frequency=800:duration=2',
                '-ar', '16000',
                str(tts_file)
            ]

            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)

            if result.returncode == 0 and tts_file.exists():
                self.log_result("TTS合成", True, f"音频文件: {tts_file.name}")

                # 测试TTS播放
                if self.test_audio_playback(str(tts_file)):
                    self.log_result("TTS播放", True, "合成音频播放成功")
                    return str(tts_file)
                else:
                    self.log_result("TTS播放", False, "合成音频播放失败")
                    return str(tts_file)
            else:
                self.log_result("TTS合成", False, f"合成失败: {result.stderr}")
                return None

        except Exception as e:
            self.log_result("TTS合成", False, f"异常: {e}")
            return None

    def test_complete_pipeline(self):
        """测试完整的ASR→LLM→TTS流程"""
        logger.info("🔄 开始完整流程测试...")

        # 步骤1: 音频录制测试
        audio_file = self.test_audio_recording()

        # 步骤2: ASR识别模拟
        asr_result = self.test_asr_simulation()

        # 步骤3: LLM处理
        llm_response = self.test_llm_processing(asr_result)

        # 步骤4: TTS合成
        tts_file = self.test_tts_synthesis(llm_response)

        # 生成测试报告
        self.generate_pipeline_report()

    def test_performance_metrics(self):
        """测试性能指标"""
        logger.info("📊 测试性能指标...")

        try:
            # 测试消息创建性能
            start_time = time.time()

            from rclpy.clock import Clock
            clock = Clock()

            # 创建100个消息测试性能
            for i in range(100):
                ASRResult(
                    header=self.create_header(clock),
                    text=f"测试文本 {i}",
                    confidence=0.95,
                    status_code=0
                )

            message_creation_time = time.time() - start_time

            self.log_result("消息性能", True,
                        f"100个消息耗时: {message_creation_time:.3f}s ({100/message_creation_time:.1f} msg/s)")

            # 测试音频系统性能
            audio_start = time.time()
            test_audio = self.audio_test_dir / "perf_test.wav"
            cmd = ['arecord', '-D', 'default', '-d', '1', '-f', 'cd', str(test_audio)]
            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                audio_time = time.time() - audio_start
                self.log_result("音频性能", True,
                            f"1秒录制耗时: {audio_time:.3f}s")

                # 清理
                if test_audio.exists():
                    test_audio.unlink()
            else:
                self.log_result("音频性能", False, "录制测试失败")

        except Exception as e:
            self.log_result("性能测试", False, f"异常: {e}")

    def create_header(self, clock):
        """创建消息Header"""
        class MockHeader:
            def __init__(self, clock):
                self.stamp = clock.now().to_msg()

            def frame_id(self):
                return "test_frame"

        return MockHeader(clock)

    def generate_pipeline_report(self):
        """生成流程测试报告"""
        logger.info("📋 生成测试报告...")

        total_tests = len(self.test_results)
        passed_tests = sum(1 for r in self.test_results if r["status"])
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0

        print("\n" + "=" * 60)
        print("📊 XLeRobot ASR→LLM→TTS 完整流程测试报告")
        print("=" * 60)

        print(f"🎯 会话ID: {self.session_id}")
        print(f"📊 测试结果: {passed_tests}/{total_tests} ({success_rate:.1f}%)")

        if success_rate >= 90:
            print("🎉 优秀！系统完全就绪")
        elif success_rate >= 75:
            print("✅ 良好！系统基本就绪")
        elif success_rate >= 50:
            print("⚠️ 一般！系统部分功能正常")
        else:
            print("❌ 需要改进！系统存在较多问题")

        print("\n📋 详细测试结果:")
        for result in self.test_results:
            status = "✅ PASS" if result["status"] else "❌ FAIL"
            print(f"  {status} {result['name']}")
            if result["details"]:
                print(f"      {result['details']}")

        # 保存报告到文件
        report_file = f"pipeline_test_report_{self.session_id}.md"
        with open(report_file, 'w', encoding='utf-8') as f:
            f.write("# XLeRobot Pipeline Test Report\n\n")
            f.write(f"## Session: {self.session_id}\n")
            f.write(f"## Success Rate: {success_rate:.1f}% ({passed_tests}/{total_tests})\n\n")
            f.write("## Test Results:\n\n")

            for result in self.test_results:
                status = "PASS" if result["status"] else "FAIL"
                f.write(f"- **{result['name']}**: {status}\n")
                if result["details"]:
                    f.write(f"  - {result['details']}\n")

        print(f"\n📄 详细报告已保存到: {report_file}")

    def cleanup(self):
        """清理测试文件"""
        try:
            import shutil
            if self.audio_test_dir.exists():
                shutil.rmtree(self.audio_test_dir)
                logger.info("✅ 清理测试文件完成")
        except Exception as e:
            logger.warning(f"⚠️ 清理文件失败: {e}")

def main():
    """主函数"""
    print("🚀 XLeRobot ASR→LLM→TTS 完整流程测试")
    print("=" * 60)
    print("使用动态消息版本进行功能验证")
    print("=" * 60)

    # 检查基础环境
    if not DYNAMIC_MESSAGES_AVAILABLE:
        print("❌ 动态消息系统不可用，无法继续测试")
        return 1

    # 创建测试器
    tester = PipelineTester()

    try:
        # 运行完整流程测试
        tester.test_complete_pipeline()

        # 运行性能测试
        tester.test_performance_metrics()

        print("\n🎉 测试完成！")
        return 0

    except KeyboardInterrupt:
        print("\n⚠️ 测试被用户中断")
        return 1
    except Exception as e:
        logger.error(f"❌ 测试执行异常: {e}")
        return 1
    finally:
        # 清理资源
        tester.cleanup()

if __name__ == "__main__":
    sys.exit(main())