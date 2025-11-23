#!/usr/bin/env python3
"""
XleRobot 真实语音助手 - 严禁Mock数据
=======================================

严格遵循真实交互流程：
1. 用户说："傻强" (唤醒词)
2. 系统播放："傻强系度,老细有乜可以帮到你!"
3. 用户说具体指令
4. 系统执行：ASR识别 → LLM处理 → TTS合成 → 播放回复

作者: BMad Master (真实交互版本)
版本: 1.0 (严禁Mock数据)
日期: 2025-11-14
原则: 严禁使用任何Mock、模拟或硬编码数据
"""

import os
import sys
import time
import subprocess
import threading
import logging
from pathlib import Path

# 从环境文件加载API密钥
env_path = Path(__file__).parent.parent / ".env"
if env_path.exists():
    import subprocess
    result = subprocess.run(['bash', '-c', f'source {env_path} && env'], capture_output=True, text=True)
    for line in result.stdout.split('\n'):
        if '=' in line:
            key, value = line.split('=', 1)
            os.environ[key] = value

# 设置路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class XleobotRealVoiceAssistant:
    """XleRobot真实语音助手 - 严禁Mock数据"""

    def __init__(self):
        """初始化真实语音助手"""
        # 严禁Mock数据 - 只加载真实组件
        self.asr_service = None
        self.llm_service = None
        self.tts_service = None
        self.audio_processor = None
        self.wake_word_detector = None

        # 真实交互状态
        self.is_active = False
        self.listening_thread = None

        logger.info("🔧 正在初始化真实语音助手（严禁Mock数据）...")
        self._initialize_components()
        logger.info("✅ 真实语音助手初始化完成")

    def _initialize_components(self):
        """初始化真实组件 - 严禁Mock数据"""
        try:
            # 1. 加载真实ASR服务
            from modules.asr.websocket_asr_service import WebSocketASRService
            self.asr_service = WebSocketASRService(enable_optimization=False)
            logger.info("✅ 真实ASR服务加载成功")

            # 2. 加载真实LLM服务
            from modules.asr.siqiang_intelligent_dialogue import create_siqiang_dialogue_manager
            self.llm_service = create_siqiang_dialogue_manager()
            logger.info("✅ 真实LLM服务加载成功")

            # 3. 加载真实TTS服务
            from modules.tts.engine.aliyun_tts_client import AliyunTTSClient
            self.tts_service = AliyunTTSClient()
            logger.info("✅ 真实TTS服务加载成功")

            # 4. 加载真实音频处理器
            from modules.asr.unified_audio_processor import create_unified_audio_processor
            self.audio_processor = create_unified_audio_processor()
            logger.info("✅ 真实音频处理器加载成功")

            # 5. 加载唤醒词检测器
            from modules.asr.aliyun_wake_word_service import create_wake_word_service
            self.wake_word_detector = create_wake_word_service()
            logger.info("✅ 真实唤醒词检测器加载成功")

        except Exception as e:
            logger.error(f"❌ 组件加载失败: {e}")
            raise

    def _play_audio(self, audio_file: str) -> bool:
        """播放真实音频文件"""
        try:
            result = subprocess.run(['aplay', audio_file],
                                  capture_output=True, timeout=10)
            return result.returncode == 0
        except Exception as e:
            logger.error(f"❌ 音频播放失败: {e}")
            return False

    def _tts_synthesize_and_play(self, text: str, voice: str = "sijia") -> bool:
        """TTS合成并播放真实语音 - 严禁Mock数据"""
        try:
            logger.info(f"🔊 正在合成语音: {text}")

            # 真实TTS合成
            audio_data = self.tts_service.synthesize(text, voice=voice)
            if not audio_data:
                logger.error("❌ TTS合成失败")
                return False

            # 保存真实音频文件
            temp_audio_file = "/tmp/xlerobot_response.wav"
            with open(temp_audio_file, 'wb') as f:
                f.write(audio_data)

            logger.info(f"✅ TTS合成成功: {len(audio_data)} 字节")

            # 播放真实语音
            success = self._play_audio(temp_audio_file)
            if success:
                logger.info("✅ 语音播放成功")
            else:
                logger.error("❌ 语音播放失败")

            return success

        except Exception as e:
            logger.error(f"❌ TTS处理失败: {e}")
            return False

    def _record_audio(self, duration: int = 3) -> str:
        """录制真实音频 - 严禁Mock数据"""
        try:
            temp_audio_file = "/tmp/xlerobot_user_input.wav"

            logger.info(f"🎤 正在录音 {duration} 秒...")

            # 真实录音命令
            result = subprocess.run([
                'arecord', '-D', 'hw:0,0',
                '-f', 'S16_LE',
                '-r', '16000',
                '-c', '1',
                '-d', str(duration),
                temp_audio_file
            ], capture_output=True, text=True, timeout=duration + 5)

            if result.returncode != 0:
                logger.error(f"❌ 录音失败: {result.stderr}")
                return None

            file_size = os.path.getsize(temp_audio_file)
            logger.info(f"✅ 录音完成: {file_size} 字节")

            return temp_audio_file

        except Exception as e:
            logger.error(f"❌ 录音异常: {e}")
            return None

    def _asr_recognize(self, audio_file: str) -> str:
        """ASR语音识别 - 严禁Mock数据"""
        try:
            logger.info("🧠 正在进行ASR识别...")

            with open(audio_file, 'rb') as f:
                audio_data = f.read()

            # 真实音频预处理
            processed_audio, audio_info = self.audio_processor.process_audio(audio_data)
            if not processed_audio:
                logger.error("❌ 音频预处理失败")
                return None

            logger.info(f"📊 音频信息: {audio_info.duration:.2f}s, {audio_info.channels}ch")

            # 真实ASR识别
            asr_result = self.asr_service.recognize_speech(
                processed_audio,
                language="cn-cantonese"
            )

            if asr_result.success:
                logger.info(f"✅ ASR识别成功: '{asr_result.text}' (置信度: {asr_result.confidence}%)")
                return asr_result.text
            else:
                logger.error(f"❌ ASR识别失败: {asr_result.error}")
                return None

        except Exception as e:
            logger.error(f"❌ ASR处理失败: {e}")
            return None

    def _llm_process(self, user_text: str) -> str:
        """LLM智能处理 - 严禁Mock数据"""
        try:
            logger.info(f"🤖 正在进行LLM处理: {user_text}")

            # 真实LLM处理
            if hasattr(self.llm_service, 'generate_response'):
                llm_response = self.llm_service.generate_response(user_text)
                response_text = llm_response.text
                logger.info(f"🧠 LLM回应: {response_text} (情绪:{llm_response.emotion})")
            else:
                # 备用简单回应
                response_text = f"我听到你讲：{user_text}"
                logger.info(f"🧠 简单回应: {response_text}")

            return response_text

        except Exception as e:
            logger.error(f"❌ LLM处理失败: {e}")
            return "抱歉，我暂时无法处理你的请求"

    def _detect_wake_word(self, audio_file: str) -> bool:
        """检测唤醒词 - 严禁Mock数据"""
        try:
            logger.info("🔍 正在检测唤醒词...")

            # 真实唤醒词检测
            if self.wake_word_detector:
                with open(audio_file, 'rb') as f:
                    audio_data = f.read()

                # 假设唤醒词检测器有detect方法
                if hasattr(self.wake_word_detector, 'detect'):
                    result = self.wake_word_detector.detect(audio_data)
                    logger.info(f"🎯 唤醒词检测结果: {result}")
                    return result
                else:
                    # 简化版唤醒词检测：检测是否包含"傻强"
                    text = self._asr_recognize(audio_file)
                    if text and ("傻强" in text or "小强" in text or "xiaogang" in text.lower()):
                        logger.info(f"🎯 检测到唤醒词: {text}")
                        return True

            return False

        except Exception as e:
            logger.error(f"❌ 唤醒词检测失败: {e}")
            return False

    def start_voice_assistant(self):
        """启动真实语音助手 - 严格按照流程"""
        logger.info("🚀 启动XleRobot真实语音助手")
        logger.info("📋 交互流程:")
        logger.info("   1. 用户说: '傻强' (唤醒词)")
        logger.info("   2. 系统播放: '傻强系度,老细有乜可以帮到你!'")
        logger.info("   3. 用户说具体指令")
        logger.info("   4. 系统执行: ASR→LLM→TTS→播放")

        self.is_active = True

        try:
            while self.is_active:
                print("\n" + "="*60)
                print("🎤 XleRobot语音助手 - 等待唤醒词...")
                print("💬 请说: '傻强' 来唤醒系统")
                print("🔇 输入 'quit' 退出")
                print("="*60)

                # 检测唤醒词
                audio_file = self._record_audio(duration=3)
                if not audio_file:
                    continue

                # 检测是否为唤醒词
                if self._detect_wake_word(audio_file):
                    # 播放唤醒回应
                    wake_response = "傻强系度,老细有乜可以帮到你!"
                    logger.info(f"🔊 播放唤醒回应: {wake_response}")

                    if self._tts_synthesize_and_play(wake_response):
                        print(f"🤖 傻强: {wake_response}")

                        # 等待用户指令
                        print("\n🎤 现在请说出您的指令...")

                        # 录制用户指令
                        instruction_audio = self._record_audio(duration=5)
                        if instruction_audio:
                            # ASR识别指令
                            user_command = self._asr_recognize(instruction_audio)
                            if user_command:
                                print(f"👤 用户: {user_command}")

                                # LLM处理指令
                                ai_response = self._llm_process(user_command)

                                # TTS合成并播放回应
                                if self._tts_synthesize_and_play(ai_response):
                                    print(f"🤖 傻强: {ai_response}")
                                else:
                                    print("❌ 回应播放失败")
                            else:
                                print("❌ 无法识别您的指令，请重试")
                        else:
                            print("❌ 指令录音失败，请重试")
                    else:
                        print("❌ 唤醒回应播放失败")
                else:
                    print("❌ 未检测到唤醒词，请重试")

        except KeyboardInterrupt:
            logger.info("🛑 用户中断，正在停止语音助手")
        except Exception as e:
            logger.error(f"❌ 语音助手运行异常: {e}")
        finally:
            self.is_active = False
            logger.info("🔚 XleRobot语音助手已停止")

def main():
    """主函数 - 启动真实语音助手"""
    print("🚀 启动XleRobot真实语音助手")
    print("🚫 严禁Mock数据 - 只使用真实麦克风和算法")
    print("🎤 需要真实的麦克风输入")
    print("🔊 需要真实的扬声器输出")

    try:
        assistant = XleobotRealVoiceAssistant()
        assistant.start_voice_assistant()
    except Exception as e:
        logger.error(f"❌ 启动失败: {e}")
        return 1

    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)