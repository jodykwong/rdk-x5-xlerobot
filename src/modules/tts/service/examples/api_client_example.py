"""
TTS API客户端使用示例

演示如何使用TTS RESTful API。
"""

import base64
import json
import aiohttp
import asyncio
from typing import Optional


class TTSAPIClient:
    """TTS API客户端"""

    def __init__(self, base_url: str = "http://localhost:8000"):
        """
        初始化客户端

        Args:
            base_url: API基础URL
        """
        self.base_url = base_url.rstrip('/')

    async def health_check(self, detailed: bool = False) -> dict:
        """
        健康检查

        Args:
            detailed: 是否返回详细信息

        Returns:
            健康检查结果
        """
        url = f"{self.base_url}/api/v1/tts/health?detailed={detailed}"
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                return await response.json()

    async def get_voices(
        self,
        language: Optional[str] = None,
        gender: Optional[str] = None,
        emotion_support: Optional[bool] = None
    ) -> dict:
        """
        获取音色列表

        Args:
            language: 过滤语言
            gender: 过滤性别
            emotion_support: 是否支持情感

        Returns:
            音色列表
        """
        params = {}
        if language:
            params['language'] = language
        if gender:
            params['gender'] = gender
        if emotion_support is not None:
            params['emotion_support'] = emotion_support

        url = f"{self.base_url}/api/v1/tts/voices"
        async with aiohttp.ClientSession() as session:
            async with session.get(url, params=params) as response:
                return await response.json()

    async def synthesize(
        self,
        text: str,
        voice_id: str = "default",
        language: str = "zh-CN",
        emotion: str = "neutral",
        emotion_intensity: float = 0.5,
        speed: float = 1.0,
        pitch: float = 1.0,
        volume: float = 1.0,
        audio_format: str = "wav",
        audio_quality: str = "medium",
        sample_rate: int = 22050,
        cache_enabled: bool = True
    ) -> bytes:
        """
        文本转语音

        Args:
            text: 文本内容
            voice_id: 音色ID
            language: 语言代码
            emotion: 情感类型
            emotion_intensity: 情感强度
            speed: 语速
            pitch: 音调
            volume: 音量
            audio_format: 音频格式
            audio_quality: 音频质量
            sample_rate: 采样率
            cache_enabled: 是否启用缓存

        Returns:
            音频数据
        """
        url = f"{self.base_url}/api/v1/tts/synthesize"
        data = {
            "text": text,
            "voice_id": voice_id,
            "language": language,
            "emotion": emotion,
            "emotion_intensity": emotion_intensity,
            "speed": speed,
            "pitch": pitch,
            "volume": volume,
            "audio_format": audio_format,
            "audio_quality": audio_quality,
            "sample_rate": sample_rate,
            "cache_enabled": cache_enabled
        }

        async with aiohttp.ClientSession() as session:
            async with session.post(url, json=data) as response:
                result = await response.json()

                if result.get('success'):
                    # 解码音频数据
                    audio_base64 = result.get('audio_data')
                    audio_data = base64.b64decode(audio_base64)
                    return audio_data
                else:
                    raise Exception(f"TTS合成失败: {result.get('error_message')}")


async def example_basic_usage():
    """基本使用示例"""
    client = TTSAPIClient("http://localhost:8000")

    # 健康检查
    print("执行健康检查...")
    health = await client.health_check(detailed=True)
    print(f"服务状态: {health['status']}")
    print(f"运行时间: {health['uptime_hours']:.2f} 小时")

    # 获取音色列表
    print("\n获取音色列表...")
    voices = await client.get_voices()
    print(f"可用音色数量: {voices['total']}")
    for voice in voices['voices'][:3]:  # 只显示前3个
        print(f"  - {voice['name']} ({voice['voice_id']})")

    # 文本转语音
    print("\n执行文本转语音...")
    audio = await client.synthesize(
        text="你好，欢迎使用TTS服务！",
        voice_id="default",
        emotion="happy",
        emotion_intensity=0.8
    )
    print(f"合成音频大小: {len(audio)} 字节")

    # 保存音频文件
    with open("output.wav", "wb") as f:
        f.write(audio)
    print("音频已保存到 output.wav")


async def example_batch_synthesis():
    """批量合成示例"""
    client = TTSAPIClient("http://localhost:8000")

    texts = [
        "这是第一句话",
        "这是第二句话",
        "这是第三句话"
    ]

    print(f"\n批量合成 {len(texts)} 句话...")
    for i, text in enumerate(texts, 1):
        print(f"  正在合成第 {i} 句...")
        audio = await client.synthesize(text)
        with open(f"output_{i}.wav", "wb") as f:
            f.write(audio)
    print("批量合成完成！")


async def example_performance_test():
    """性能测试示例"""
    import time

    client = TTSAPIClient("http://localhost:8000")

    # 性能测试
    print("\n执行性能测试...")
    start_time = time.time()

    for i in range(10):
        audio = await client.synthesize(f"这是第 {i+1} 次测试")

    end_time = time.time()
    total_time = end_time - start_time
    avg_time = total_time / 10

    print(f"总耗时: {total_time:.2f} 秒")
    print(f"平均耗时: {avg_time:.2f} 秒/次")
    print(f"QPS: {10 / total_time:.2f}")


if __name__ == "__main__":
    # 运行示例
    print("=== TTS API客户端使用示例 ===\n")

    # 基本使用
    asyncio.run(example_basic_usage())

    # 批量合成
    # asyncio.run(example_batch_synthesis())

    # 性能测试
    # asyncio.run(example_performance_test())

    print("\n示例运行完成！")
