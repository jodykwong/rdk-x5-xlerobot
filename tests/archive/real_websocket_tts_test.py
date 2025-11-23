#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
基于WebSocket的真实阿里云TTS验证
严格遵循阿里云NLS WebSocket连接指南

禁止Mock数据 - 使用真实API
"""

import os
import sys
import time
import json
import base64
import wave
import numpy as np
import logging
from pathlib import Path

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / 'src'))
sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')

# 导入阿里云NLS SDK
from nls.token import getToken
from nls.speech_synthesizer import NlsSpeechSynthesizer

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RealWebSocketTTS:
    """基于WebSocket的真实阿里云TTS客户端"""

    def __init__(self):
        """初始化TTS客户端"""
        # API凭证 (来自文档)
        self.access_key_id = "YOUR_ACCESS_KEY_ID"
        self.access_key_secret = "YOUR_ACCESS_KEY_SECRET"
        self.app_key = "YOUR_NLS_APPKEY"

        # WebSocket端点
        self.ws_url = "wss://nls-gateway.cn-shanghai.aliyuncs.com/ws/v1"

        # 状态
        self.token = None
        self.synthesizer = None
        self.audio_data = None
        self.synthesis_completed = False

        logger.info("🔧 初始化WebSocket TTS客户端...")

    def get_token(self):
        """获取访问Token"""
        try:
            logger.info("🔐 获取访问Token...")
            self.token = getToken(self.access_key_id, self.access_key_secret)

            if self.token:
                logger.info(f"✅ Token获取成功: {self.token[:16]}...")
                return True
            else:
                logger.error("❌ Token获取失败")
                return False

        except Exception as e:
            logger.error(f"❌ Token获取异常: {e}")
            return False

    def setup_synthesizer(self):
        """设置语音合成器"""
        try:
            logger.info("🎙️ 设置语音合成器...")

            self.synthesizer = NlsSpeechSynthesizer(
                token=self.token,
                appkey=self.app_key,
                on_metainfo=self._on_metainfo,
                on_data=self._on_data,
                on_completed=self._on_completed,
                on_error=self._on_error
            )

            logger.info("✅ 语音合成器设置完成")
            return True

        except Exception as e:
            logger.error(f"❌ 语音合成器设置失败: {e}")
            return False

    def _on_metainfo(self, message, *args):
        """合成元信息回调"""
        logger.info("🎤 语音合成开始")
        logger.info(f"   消息: {message}")

    def _on_data(self, message, *args):
        """合成数据回调 - 这里包含实际的音频数据"""
        logger.info("🔄 合成进行中...")

        try:
            # 尝试解析消息
            if isinstance(message, str):
                result = json.loads(message)
            else:
                result = message

            # 检查是否包含音频数据
            if 'payload' in result and 'binary_data' in result['payload']:
                # 解码音频数据
                audio_base64 = result['payload']['binary_data']
                if isinstance(audio_base64, str):
                    self.audio_data = base64.b64decode(audio_base64)
                else:
                    self.audio_data = audio_base64
                logger.info(f"📊 音频数据获取成功: {len(self.audio_data)} 字节")
            elif isinstance(message, bytes):
                # 消息本身就是音频数据
                self.audio_data = message
                logger.info(f"📊 音频数据获取成功 (直接): {len(self.audio_data)} 字节")
            elif 'data' in result:
                # 尝试从data字段获取
                audio_data = result['data']
                if isinstance(audio_data, str):
                    self.audio_data = base64.b64decode(audio_data)
                else:
                    self.audio_data = audio_data
                logger.info(f"📊 音频数据获取成功 (data字段): {len(self.audio_data)} 字节")

        except Exception as e:
            logger.error(f"❌ 音频数据处理失败: {e}")

    def _on_completed(self, message, *args):
        """合成完成回调"""
        logger.info("✅ 语音合成完成")
        logger.info(f"   完成消息: {message}")

        try:
            # 尝试解析消息
            if isinstance(message, str):
                result = json.loads(message)
            else:
                result = message

            logger.info(f"   解析结果: {result}")

            # 检查不同的可能格式
            if 'payload' in result:
                payload = result['payload']
                if 'binary_data' in payload:
                    # 格式1: payload.binary_data
                    audio_base64 = payload['binary_data']
                    self.audio_data = base64.b64decode(audio_base64)
                    logger.info(f"📊 音频数据获取成功 (格式1): {len(self.audio_data)} 字节")
                elif 'data' in payload:
                    # 格式2: payload.data
                    audio_base64 = payload['data']
                    self.audio_data = base64.b64decode(audio_base64)
                    logger.info(f"📊 音频数据获取成功 (格式2): {len(self.audio_data)} 字节")
                elif isinstance(payload, str):
                    # 格式3: payload就是base64字符串
                    self.audio_data = base64.b64decode(payload)
                    logger.info(f"📊 音频数据获取成功 (格式3): {len(self.audio_data)} 字节")
                else:
                    logger.warning(f"⚠️ 未知的payload格式: {type(payload)}")
            elif 'data' in result:
                # 格式4: 直接在result中
                audio_base64 = result['data']
                self.audio_data = base64.b64decode(audio_base64)
                logger.info(f"📊 音频数据获取成功 (格式4): {len(self.audio_data)} 字节")
            else:
                logger.warning(f"⚠️ 未找到音频数据，结果键: {list(result.keys())}")

            self.synthesis_completed = True

        except Exception as e:
            logger.error(f"❌ 合成结果处理失败: {e}")
            logger.error(f"   原始消息类型: {type(message)}")
            logger.error(f"   原始消息内容: {message}")
            self.synthesis_completed = True

    def _on_error(self, message, *args):
        """合成错误回调"""
        logger.error(f"❌ 语音合成错误: {message}")
        self.synthesis_completed = True

    def synthesize_speech(self, text, voice="xiaoyun", sample_rate=16000):
        """合成语音"""
        try:
            logger.info(f"🎯 开始语音合成...")
            logger.info(f"   文本: {text}")
            logger.info(f"   发音人: {voice}")
            logger.info(f"   采样率: {sample_rate}Hz")

            # 启动合成
            self.synthesizer.start(
                text=text,
                voice=voice,
                aformat="wav",
                sample_rate=sample_rate
            )

            # 等待合成完成
            timeout = 30
            start_time = time.time()

            while not self.synthesis_completed and (time.time() - start_time) < timeout:
                time.sleep(0.1)

            # 停止合成 (检查方法是否存在)
            if hasattr(self.synthesizer, 'stop'):
                self.synthesizer.stop()

            if self.audio_data:
                logger.info("✅ 语音合成成功")
                return self.audio_data
            else:
                logger.error("❌ 未获取到音频数据")
                return None

        except Exception as e:
            logger.error(f"❌ 语音合成异常: {e}")
            return None

        finally:
            # 清理连接
            try:
                if self.synthesizer:
                    self.synthesizer.shutdown()
            except:
                pass

    def save_audio_file(self, audio_data, filename):
        """保存音频文件"""
        try:
            with open(filename, 'wb') as f:
                f.write(audio_data)
            logger.info(f"💾 音频文件已保存: {filename}")
            return True
        except Exception as e:
            logger.error(f"❌ 音频文件保存失败: {e}")
            return False

def run_real_websocket_tts_test():
    """运行真实WebSocket TTS测试"""
    print("🎯 基于WebSocket的真实阿里云TTS验证")
    print("=" * 60)
    print("🚨 禁用Mock数据，使用真实WebSocket API")
    print("🔗 遵循阿里云NLS WebSocket连接指南")
    print("=" * 60)

    # 创建TTS客户端
    tts_client = RealWebSocketTTS()

    # 测试用例
    test_cases = [
        {
            "text": "你好，我是XleRobot助手",
            "voice": "xiaoyun",
            "filename": "websocket_tts_test_1.wav"
        },
        {
            "text": "欢迎使用阿里云智能语音服务",
            "voice": "xiaoyun",
            "filename": "websocket_tts_test_2.wav"
        },
        {
            "text": "这是真实的语音合成测试",
            "voice": "xiaoyun",
            "filename": "websocket_tts_test_3.wav"
        }
    ]

    # 创建输出目录
    output_dir = Path("/tmp/xlerobot_websocket_tts")
    output_dir.mkdir(exist_ok=True)

    success_count = 0
    total_tests = len(test_cases)

    # 执行测试
    for i, test_case in enumerate(test_cases, 1):
        print(f"\n🎵 测试 {i}/{total_tests}: {test_case['text'][:20]}...")

        # 重置状态
        tts_client.audio_data = None
        tts_client.synthesis_completed = False

        try:
            # 1. 获取Token
            if not tts_client.get_token():
                continue

            # 2. 设置合成器
            if not tts_client.setup_synthesizer():
                continue

            # 3. 执行语音合成
            audio_data = tts_client.synthesize_speech(
                text=test_case['text'],
                voice=test_case['voice']
            )

            if audio_data:
                # 4. 保存音频文件
                output_file = output_dir / test_case['filename']
                if tts_client.save_audio_file(audio_data, str(output_file)):
                    success_count += 1
                    print(f"   ✅ 成功保存: {output_file}")

                    # 播放音频
                    try:
                        import subprocess
                        result = subprocess.run(['aplay', str(output_file)],
                                              capture_output=True, text=True, timeout=10)
                        if result.returncode == 0:
                            print(f"   🔊 音频播放成功")
                        else:
                            print(f"   ⚠️ 音频播放失败: {result.stderr}")
                    except Exception as e:
                        print(f"   ⚠️ 无法播放音频: {e}")
                else:
                    print(f"   ❌ 音频保存失败")
            else:
                print(f"   ❌ 语音合成失败")

        except Exception as e:
            print(f"   ❌ 测试异常: {e}")

    # 测试结果汇总
    print(f"\n" + "=" * 60)
    print(f"📊 WebSocket TTS测试结果汇总")
    print(f"=" * 60)
    print(f"总测试数: {total_tests}")
    print(f"成功数量: {success_count}")
    print(f"成功率: {success_count/total_tests*100:.1f}%")

    if success_count == total_tests:
        print(f"\n🎉 所有WebSocket TTS测试通过!")
        print(f"🚀 系统已验证具备真实的阿里云TTS能力!")
        print(f"📁 音频文件位于: {output_dir}")

        # 列出生成的文件
        print(f"\n📁 生成的音频文件:")
        for file_path in sorted(output_dir.glob("*.wav")):
            print(f"   - {file_path}")

        return True
    else:
        print(f"\n❌ WebSocket TTS测试部分失败")
        return False

if __name__ == "__main__":
    try:
        success = run_real_websocket_tts_test()

        if success:
            print(f"\n✅ Epic 1 真实WebSocket TTS验证: 成功完成!")
        else:
            print(f"\n❌ Epic 1 真实WebSocket TTS验证: 失败!")

    except KeyboardInterrupt:
        print(f"\n⏹️ 用户中断测试")
    except Exception as e:
        print(f"\n💥 测试过程中发生异常: {e}")