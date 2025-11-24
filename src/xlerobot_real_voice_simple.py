#!/usr/bin/env python3
"""
XleRobot 简化版真实语音助手 - 严禁Mock数据
===========================================

简化处理唤醒词问题：
1. 录音3秒
2. ASR识别文本
3. 检测是否包含"傻强"（或相关词汇）
4. 如果是唤醒词，播放回应并等待指令
5. 处理指令并回应

原则：严禁Mock数据 - 只使用真实麦克风和算法
"""

import os
import sys
import time
import subprocess
import logging
import threading

# 严禁Mock数据
os.environ["ALIBABA_CLOUD_ACCESS_KEY_ID"] = "YOUR_ACCESS_KEY_ID"
os.environ["ALIBABA_CLOUD_ACCESS_KEY_SECRET"] = "YOUR_ACCESS_KEY_SECRET"
os.environ["ALIYUN_NLS_APPKEY"] = "YOUR_NLS_APPKEY"

sys.path.insert(0, '/home/sunrise/xlerobot/src')

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class XleobotSimpleVoiceAssistant:
    """XleRobot简化版真实语音助手"""

    def __init__(self):
        """初始化简化版语音助手"""
        self.asr_service = None
        self.llm_service = None
        self.tts_service = None
        self.audio_processor = None

        # 方案1：播放时禁用录音的机制
        self.is_playing = False
        self.state_lock = threading.Lock()

        # 方案3：状态机
        self.current_state = "IDLE"

        logger.info("🔧 正在初始化简化版语音助手...")
        self._initialize_components()
        logger.info("✅ 简化版语音助手初始化完成")

    def _initialize_components(self):
        """初始化真实组件"""
        try:
            # ASR服务
            from modules.asr.websocket_asr_service import WebSocketASRService
            self.asr_service = WebSocketASRService(enable_optimization=False)
            logger.info("✅ ASR服务加载成功")

            # LLM服务
            from modules.asr.siqiang_intelligent_dialogue import create_siqiang_dialogue_manager
            self.llm_service = create_siqiang_dialogue_manager()
            logger.info("✅ LLM服务加载成功")

            # TTS服务
            from modules.tts.engine.aliyun_tts_client import AliyunTTSClient
            self.tts_service = AliyunTTSClient()
            logger.info("✅ TTS服务加载成功")

            # 音频处理器
            from modules.asr.unified_audio_processor import create_unified_audio_processor
            self.audio_processor = create_unified_audio_processor()
            logger.info("✅ 音频处理器加载成功")

        except Exception as e:
            logger.error(f"❌ 组件加载失败: {e}")
            raise

    def _is_valid_wake_word(self, text: str) -> bool:
        """
        严格判断是否是唤醒词（方案2改进）
        针对全在线架构优化，防止误触发
        """
        if not text:
            return False

        text_clean = text.strip()

        # 检查1：文本长度（唤醒词"傻强"只有2个字，严格限制）
        if len(text_clean) > 4:  # 更严格的长度限制
            logger.debug(f"❌ 文本太长，不是唤醒词: '{text_clean}' (长度: {len(text_clean)})")
            return False

        # 检查2：严格的唤醒词匹配（只接受真正的唤醒词）
        # 移除过于宽泛的词汇如"你好"
        wake_words = [
            "傻强", "傻強",  # 主要唤醒词
            "沙强", "沙強",  # 可能的识别偏差
            "小强"  # 备用（短一些）
        ]

        # 检查3：精确匹配而不是包含匹配
        for word in wake_words:
            if text_clean == word:
                logger.info(f"✅ 检测到有效唤醒词: '{text_clean}'")
                return True

        # 检查4：允许带简单语气词的情况，但要更严格
        if len(text_clean) <= 5:
            for word in wake_words:
                if text_clean.startswith(word) and len(text_clean) <= len(word) + 1:
                    logger.info(f"✅ 检测到有效唤醒词(带语气): '{text_clean}' -> '{word}'")
                    return True

        logger.debug(f"❌ 未检测到有效唤醒词: '{text_clean}'")
        return False

    def _is_wake_word(self, text: str) -> bool:
        """兼容性函数，调用严格判断函数"""
        return self._is_valid_wake_word(text)

    def _record_and_recognize(self, duration: int = 3) -> str:
        """录音并识别文本（方案1改进：播放时禁用录音）"""
        try:
            # 方案1：检查是否正在播放TTS
            with self.state_lock:
                if self.is_playing:
                    logger.info("🔇 TTS播放中 - 暂停录音")
                    return None

            # 录音
            temp_audio_file = "/tmp/xlerobot_audio.wav"
            logger.info(f"🎤 正在录音 {duration} 秒...")

            result = subprocess.run([
                'arecord', '-D', 'plughw:0,0',  # 使用USB麦克风
                '-f', 'S16_LE',
                '-r', '16000',
                '-c', '2',  # 立体声（USB设备支持）
                '-d', str(duration),
                temp_audio_file
            ], capture_output=True, text=True, timeout=duration + 5)

            if result.returncode != 0:
                logger.error(f"❌ 录音失败: {result.stderr}")
                return None

            file_size = os.path.getsize(temp_audio_file)
            logger.info(f"✅ 录音完成: {file_size} 字节")

            # ASR识别
            logger.info("🧠 正在进行ASR识别...")
            with open(temp_audio_file, 'rb') as f:
                audio_data = f.read()

            # 音频预处理
            processed_audio, audio_info = self.audio_processor.process_audio(audio_data)
            if not processed_audio:
                logger.error("❌ 音频预处理失败")
                return None

            logger.info(f"📊 音频信息: {audio_info.duration:.2f}s, {audio_info.channels}ch")

            # ASR识别
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
            logger.error(f"❌ 录音识别失败: {e}")
            return None

    def _speak_and_play(self, text: str, voice: str = "sijia") -> bool:
        """TTS合成并播放（方案1改进：播放时禁用录音）"""
        try:
            # 方案1：设置播放状态，阻止录音
            with self.state_lock:
                self.is_playing = True
                self.current_state = "PLAYING"
            logger.info("🔇 Microphone disabled - Playing TTS")

            logger.info(f"🔊 正在合成语音: {text}")

            # TTS合成
            audio_data = self.tts_service.synthesize(text, voice=voice)
            if not audio_data:
                logger.error("❌ TTS合成失败")
                with self.state_lock:
                    self.is_playing = False
                    self.current_state = "IDLE"
                return False

            # 保存音频文件
            temp_audio_file = "/tmp/xlerobot_speak.wav"
            with open(temp_audio_file, 'wb') as f:
                f.write(audio_data)

            logger.info(f"✅ TTS合成成功: {len(audio_data)} 字节")

            # 播放语音
            result = subprocess.run(['aplay', temp_audio_file],  # 使用默认播放设备
                                  capture_output=True, timeout=10)

            success = result.returncode == 0
            if success:
                logger.info("✅ 语音播放成功")
            else:
                logger.error("❌ 语音播放失败")

            # 方案1：播放完成后，等待0.5秒静默期再启用录音
            logger.info("⏸️ TTS completed - Waiting 0.5s before enabling microphone")
            time.sleep(0.5)

            # 清除播放状态，重新启用录音
            with self.state_lock:
                self.is_playing = False
                self.current_state = "IDLE"
            logger.info("🎤 TTS completed - Microphone enabled")

            return success

        except Exception as e:
            logger.error(f"❌ 语音处理失败: {e}")
            # 确保在异常情况下也清除播放状态
            with self.state_lock:
                self.is_playing = False
                self.current_state = "IDLE"
            logger.info("🎤 Exception occurred - Microphone enabled")
            return False

    def _process_user_command(self, user_text: str) -> str:
        """处理用户指令"""
        try:
            logger.info(f"🤖 正在处理用户指令: {user_text}")

            # LLM处理
            if hasattr(self.llm_service, 'generate_response'):
                llm_response = self.llm_service.generate_response(user_text)
                response_text = llm_response.text
                logger.info(f"🧠 LLM回应: {response_text} (情绪:{llm_response.emotion})")
            else:
                # 备用回应
                response_text = f"我听到你讲：{user_text}，让我想想怎么帮你..."
                logger.info(f"🧠 备用回应: {response_text}")

            return response_text

        except Exception as e:
            logger.error(f"❌ 指令处理失败: {e}")
            return "抱歉，我暂时无法处理你的请求"

    def run_simple_voice_loop(self):
        """运行简化语音交互循环"""
        logger.info("🚀 启动简化语音交互循环")
        logger.info("📋 简化流程:")
        logger.info("   1. 录音3秒，识别语音")
        logger.info("   2. 检测是否为唤醒词")
        logger.info("   3. 如果是唤醒词，播放回应")
        logger.info("   4. 等待用户指令，处理并回应")

        try:
            while True:
                print("\n" + "="*60)
                print("🎤 XleRobot简化语音助手")
                print("💬 正在录音，请说话（3秒）...")
                print("🔇 输入 'quit' 退出")
                print("="*60)

                # 录音并识别
                user_text = self._record_and_recognize(duration=3)
                if not user_text:
                    print("❌ 无法识别您的语音，请重试")
                    continue

                print(f"👤 识别结果: {user_text}")

                # 检查退出命令
                if "quit" in user_text.lower() or "退出" in user_text:
                    logger.info("🛑 用户请求退出")
                    break

                # 检查唤醒词
                if self._is_wake_word(user_text):
                    # 播放唤醒回应
                    wake_response = "傻强系度，老细有乜可以帮到你!"
                    logger.info(f"🔊 播放唤醒回应: {wake_response}")

                    print(f"🤖 傻强: {wake_response}")

                    if self._speak_and_play(wake_response):
                        # 等待用户指令
                        print("\n🎤 傻强已唤醒，请说出您的指令...")
                        print("💬 录音5秒，请说出具体指令")

                        # 录制用户指令
                        user_command = self._record_and_recognize(duration=5)
                        if user_command:
                            print(f"👤 用户指令: {user_command}")

                            # 处理指令
                            ai_response = self._process_user_command(user_command)

                            # 回应用户
                            print(f"🤖 傻强: {ai_response}")
                            self._speak_and_play(ai_response)
                        else:
                            print("❌ 无法识别您的指令，请重试")
                    else:
                        print("❌ 唤醒回应播放失败")
                else:
                    print("❌ 未检测到唤醒词，请说 '傻强' 或 '你好'")

        except KeyboardInterrupt:
            logger.info("🛑 用户中断")
        except Exception as e:
            logger.error(f"❌ 运行异常: {e}")
        finally:
            logger.info("🔚 简化语音助手已停止")

def main():
    """主函数"""
    print("🚀 启动XleRobot简化版真实语音助手")
    print("🚫 严禁Mock数据 - 只使用真实麦克风输入")
    print("🎯 简化流程解决唤醒词问题")

    try:
        assistant = XleobotSimpleVoiceAssistant()
        assistant.run_simple_voice_loop()
    except Exception as e:
        logger.error(f"❌ 启动失败: {e}")
        return 1

    return 0

if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)