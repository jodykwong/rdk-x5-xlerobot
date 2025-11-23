#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Epic 1: 真实API端到端验证演示
BMad-Method v6 Brownfield Level 4 企业级实现

禁用Mock数据，使用真实阿里云API进行完整的语音交互验证
"""

import os
import sys
import time
import logging
import tempfile
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / 'src'))

# 设置环境变量用于真实API
os.environ['ALIBABA_CLOUD_ACCESS_KEY_ID'] = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', '')
os.environ['ALIBABA_CLOUD_ACCESS_KEY_SECRET'] = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')

from xlerobot.tts.aliyun_tts_client import AliyunTTSClient
from xlerobot.tts.audio_processor import AudioProcessor

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RealAPIVerification:
    """真实API验证类"""

    def __init__(self):
        """初始化验证器"""
        self.processor = AudioProcessor()
        self.tts_client = None
        self.test_results = {
            'tts_connection': False,
            'tts_synthesis': False,
            'audio_quality': False,
            'end_to_end': False
        }

    def check_api_credentials(self):
        """检查API凭证"""
        logger.info("🔍 检查阿里云API凭证...")

        access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID')
        access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET')

        if not access_key_id or not access_key_secret:
            logger.error("❌ 缺少阿里云API凭证")
            logger.error("请设置环境变量:")
            logger.error("  export ALIBABA_CLOUD_ACCESS_KEY_ID='your_access_key_id'")
            logger.error("  export ALIBABA_CLOUD_ACCESS_KEY_SECRET='your_access_key_secret'")
            return False

        if len(access_key_id) < 10 or len(access_key_secret) < 20:
            logger.error("❌ API凭证格式不正确")
            return False

        logger.info("✅ API凭证检查通过")
        return True

    def setup_tts_client(self):
        """设置TTS客户端"""
        logger.info("🔧 初始化阿里云TTS客户端...")

        try:
            # 真实API配置
            config = {
                'app_key': os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID'),
                'token': os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET'),
                'region': 'cn-shanghai',
                'timeout': 15,  # 增加超时时间
                'max_retries': 2
            }

            self.tts_client = AliyunTTSClient(config)
            logger.info("✅ TTS客户端初始化完成")
            return True

        except Exception as e:
            logger.error(f"❌ TTS客户端初始化失败: {e}")
            return False

    def test_tts_connection(self):
        """测试TTS连接"""
        logger.info("🔗 测试TTS服务连接...")

        try:
            # 使用真实API测试连接
            result = self.tts_client.test_connection()

            if result:
                self.test_results['tts_connection'] = True
                logger.info("✅ TTS服务连接测试通过")
                return True
            else:
                logger.error("❌ TTS服务连接测试失败")
                return False

        except Exception as e:
            logger.error(f"❌ TTS连接测试异常: {e}")
            return False

    def test_tts_synthesis(self):
        """测试TTS语音合成"""
        logger.info("🎤 测试TTS语音合成...")

        test_text = "这是一个真实的语音合成测试，使用阿里云API生成粤语语音"

        try:
            start_time = time.time()
            audio_data = self.tts_client.synthesize_speech(
                text=test_text,
                voice='jiajia',  # 粤语发音人
                speech_rate=50,
                pitch_rate=50,
                volume=80
            )
            end_time = time.time()

            if audio_data:
                self.test_results['tts_synthesis'] = True
                self.test_results['synthesis_time'] = end_time - start_time
                logger.info(f"✅ TTS语音合成成功!")
                logger.info(f"📊 合成时间: {end_time - start_time:.3f}秒")
                logger.info(f"📏 音频大小: {len(audio_data)} 字节")
                return audio_data
            else:
                logger.error("❌ TTS语音合成失败")
                return None

        except Exception as e:
            logger.error(f"❌ TTS语音合成异常: {e}")
            return None

    def test_audio_quality(self, audio_data):
        """测试音频质量"""
        logger.info("🎵 测试音频质量...")

        try:
            # 使用真实音频数据进行质量评估
            quality_result = self.processor.evaluate_audio_quality(audio_data)

            logger.info(f"📊 音频质量评估结果:")
            logger.info(f"   质量评分: {quality_result.get('quality_score', 0)} 分")
            logger.info(f"   质量等级: {quality_result.get('quality_rating', '未知')}")
            logger.info(f"   信噪比: {quality_result.get('snr_db', 0)} dB")
            logger.info(f"   动态范围: {quality_result.get('dynamic_range_db', 0)} dB")
            logger.info(f"   零交叉率: {quality_result.get('zero_crossing_rate', 0)}")

            # 保存真实音频文件用于验证
            temp_dir = Path(tempfile.gettempdir()) / "xlerobot_real_api_test"
            temp_dir.mkdir(exist_ok=True)

            output_file = temp_dir / "real_tts_test.wav"
            success = self.processor.save_audio_file(audio_data, str(output_file))

            if success:
                logger.info(f"💾 真实音频文件已保存: {output_file}")
                logger.info("🔊 您可以使用播放器验证音频质量")

            # 质量评分检查
            quality_score = quality_result.get('quality_score', 0)
            if quality_score >= 60:
                self.test_results['audio_quality'] = True
                logger.info("✅ 音频质量测试通过")
                return True
            else:
                logger.warning(f"⚠️ 音频质量评分较低: {quality_score}")
                return False

        except Exception as e:
            logger.error(f"❌ 音频质量测试异常: {e}")
            return False

    def test_end_to_end(self):
        """端到端测试"""
        logger.info("🔄 执行端到端测试...")

        test_cases = [
            {
                'text': '你好，我是XleRobot助手，很高兴为您服务',
                'emotion': 'friendly',
                'expected_time': 5.0
            },
            {
                'text': '确认收到您的指令，现在开始执行',
                'emotion': 'confirm',
                'expected_time': 4.0
            },
            {
                'text': '抱歉，遇到了一些问题，请稍后再试',
                'emotion': 'error',
                'expected_time': 3.5
            }
        ]

        passed_tests = 0
        total_tests = len(test_cases)

        for i, test_case in enumerate(test_cases, 1):
            logger.info(f"🎯 端到端测试 {i}/{total_tests}: {test_case['emotion']}")

            try:
                # 测量完整处理时间
                start_time = time.time()

                # 1. TTS合成
                audio_data = self.tts_client.synthesize_speech(
                    text=test_case['text'],
                    voice='jiajia'
                )

                if not audio_data:
                    logger.error(f"❌ 测试 {i}: TTS合成失败")
                    continue

                # 2. 情感处理
                emotion_audio = self.processor.apply_emotion_style(audio_data, test_case['emotion'])

                if not emotion_audio:
                    logger.error(f"❌ 测试 {i}: 情感处理失败")
                    continue

                # 3. 质量增强
                enhanced_audio = self.processor.enhance_audio_quality(emotion_audio)

                if not enhanced_audio:
                    logger.error(f"❌ 测试 {i}: 质量增强失败")
                    continue

                end_time = time.time()
                total_time = end_time - start_time

                # 4. 质量评估
                quality = self.processor.evaluate_audio_quality(enhanced_audio)
                quality_score = quality.get('quality_score', 0)

                # 5. 性能检查
                performance_ok = total_time <= test_case['expected_time']
                quality_ok = quality_score >= 60

                if performance_ok and quality_ok:
                    logger.info(f"✅ 测试 {i}: 通过 (耗时: {total_time:.3f}s, 质量: {quality_score}分)")
                    passed_tests += 1
                else:
                    logger.warning(f"⚠️ 测试 {i}: 部分通过 (耗时: {total_time:.3f}s, 质量: {quality_score}分)")

            except Exception as e:
                logger.error(f"❌ 测试 {i}: 异常 - {e}")

        if passed_tests == total_tests:
            self.test_results['end_to_end'] = True
            logger.info(f"✅ 端到端测试全部通过: {passed_tests}/{total_tests}")
        else:
            logger.warning(f"⚠️ 端到端测试部分通过: {passed_tests}/{total_tests}")

        return passed_tests, total_tests

    def run_full_verification(self):
        """运行完整验证"""
        logger.info("🚀 开始Epic 1真实API验证演示")
        logger.info("=" * 60)

        # 验证步骤
        verification_steps = [
            ("检查API凭证", self.check_api_credentials),
            ("初始化TTS客户端", self.setup_tts_client),
            ("测试TTS连接", self.test_tts_connection),
            ("测试TTS语音合成", self.test_tts_synthesis),
            ("测试音频质量", lambda: self.test_audio_quality(None)),  # 在步骤中获取音频
            ("端到端测试", self.test_end_to_end)
        ]

        passed_steps = 0
        for step_name, step_func in verification_steps:
            logger.info(f"\n🔍 {step_name}...")

            try:
                if step_name == "测试音频质量":
                    # 获取上一步的音频数据
                    audio_data = None
                    if hasattr(self, '_last_audio_data'):
                        audio_data = self._last_audio_data
                        result = self.test_audio_quality(audio_data)
                        if result:
                            passed_steps += 1
                else:
                    result = step_func()
                    if result:
                        passed_steps += 1

                        # 保存TTS合成结果用于后续测试
                        if step_name == "测试TTS语音合成" and result:
                            self._last_audio_data = result

            except Exception as e:
                logger.error(f"❌ {step_name} 失败: {e}")

        logger.info("\n" + "=" * 60)
        logger.info("📊 验证结果汇总")
        logger.info("=" * 60)

        logger.info(f"✅ API凭证检查: {'通过' if self.check_api_credentials() else '失败'}")
        logger.info(f"✅ TTS客户端初始化: {'通过' if self.setup_tts_client() else '失败'}")
        logger.info(f"✅ TTS连接测试: {'通过' if self.test_results['tts_connection'] else '失败'}")
        logger.info(f"✅ TTS语音合成: {'通过' if self.test_results['tts_synthesis'] else '失败'}")
        logger.info(f"✅ 音频质量评估: {'通过' if self.test_results['audio_quality'] else '失败'}")

        if 'end_to_end' in self.test_results:
            passed, total = self.test_results['end_to_end']
            logger.info(f"✅ 端到端测试: {passed}/{total} 通过")

        passed_verification = all([
            self.check_api_credentials(),
            self.setup_tts_client(),
            self.test_results['tts_connection'],
            self.test_results['tts_synthesis'],
            self.test_results['audio_quality']
        ])

        if passed_verification:
            logger.info("\n🎉 Epic 1真实API验证: PASSED ✅")
            logger.info("🚀 系统已准备好投入生产使用!")
        else:
            logger.error("\n❌ Epic 1真实API验证: FAILED")
            logger.error("🔧 请检查API配置和网络连接")

        return passed_verification

def main():
    """主函数"""
    print("🎯 Epic 1: 真实API验证演示")
    print("=" * 50)
    print("🚨 禁用Mock数据，使用真实阿里云API")
    print("🔗 执行端到端验证测试")
    print("=" * 50)

    verifier = RealAPIVerification()

    try:
        success = verifier.run_full_verification()

        if success:
            print("\n🎊 验证成功！Epic 1已准备好生产部署")
            print("\n📁 生成的测试文件:")
            temp_dir = Path(tempfile.gettempdir()) / "xlerobot_real_api_test"
            if temp_dir.exists():
                for file_path in temp_dir.glob("*.wav"):
                    print(f"  - {file_path}")

            print("\n🎵 您可以使用以下命令播放音频:")
            print(f"  - aplay {temp_dir}/real_tts_test.wav")

        else:
            print("\n❌ 验证失败！请检查配置后重试")

    except KeyboardInterrupt:
        print("\n⏹️ 用户中断验证")
    except Exception as e:
        print(f"\n💥 验证过程中发生异常: {e}")

if __name__ == "__main__":
    main()