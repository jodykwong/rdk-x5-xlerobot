"""
TTS WebSocket客户端使用示例

演示如何使用TTS WebSocket流式接口。
"""

import asyncio
import json
import base64
import websockets
from typing import Optional


class TTSWebSocketClient:
    """TTS WebSocket客户端"""

    def __init__(self, ws_url: str = "ws://localhost:8000/ws/v1/tts/stream"):
        """
        初始化客户端

        Args:
            ws_url: WebSocket连接URL
        """
        self.ws_url = ws_url

    async def synthesize_stream(
        self,
        text: str,
        voice_id: str = "default",
        language: str = "zh-CN",
        emotion: str = "neutral",
        emotion_intensity: float = 0.5,
        speed: float = 1.0,
        chunk_size: int = 1024,
        output_file: Optional[str] = None
    ) -> bytes:
        """
        流式文本转语音

        Args:
            text: 文本内容
            voice_id: 音色ID
            language: 语言代码
            emotion: 情感类型
            emotion_intensity: 情感强度
            speed: 语速
            chunk_size: 音频块大小
            output_file: 输出文件路径（可选）

        Returns:
            完整的音频数据
        """
        async with websockets.connect(self.ws_url) as websocket:
            print(f"WebSocket连接已建立: {self.ws_url}")

            # 发送TTS请求
            request = {
                "type": "request",
                "data": {
                    "text": text,
                    "voice_id": voice_id,
                    "language": language,
                    "emotion": emotion,
                    "emotion_intensity": emotion_intensity,
                    "speed": speed,
                    "chunk_size": chunk_size
                }
            }
            await websocket.send(json.dumps(request))
            print(f"发送请求: {text[:20]}...")

            # 接收音频分块
            audio_chunks = []
            chunk_count = 0

            try:
                while True:
                    response = await websocket.recv()
                    data = json.loads(response)

                    # 处理心跳
                    if data.get('type') == 'heartbeat':
                        continue

                    # 处理音频分块
                    if data.get('type') == 'response':
                        chunk_count += 1
                        audio_base64 = data.get('audio_data')
                        audio_chunk = base64.b64decode(audio_base64)
                        audio_chunks.append(audio_chunk)

                        print(f"接收分块 {chunk_count}: {len(audio_chunk)} 字节")

                        # 如果是最后一个分块，退出
                        if data.get('is_final'):
                            break

                    # 处理错误
                    elif data.get('type') == 'error':
                        raise Exception(f"服务器错误: {data.get('message')}")

            except websockets.exceptions.ConnectionClosed:
                print("WebSocket连接已关闭")

            # 合并音频分块
            full_audio = b''.join(audio_chunks)
            print(f"完整音频大小: {len(full_audio)} 字节")

            # 保存到文件
            if output_file:
                with open(output_file, "wb") as f:
                    f.write(full_audio)
                print(f"音频已保存到: {output_file}")

            return full_audio

    async def monitor_connection(self):
        """监控连接状态（发送心跳）"""
        try:
            async with websockets.connect(self.ws_url) as websocket:
                print("监控连接状态...")

                while True:
                    # 发送心跳
                    heartbeat = {
                        "type": "heartbeat"
                    }
                    await websocket.send(json.dumps(heartbeat))

                    # 等待响应
                    response = await websocket.recv()
                    data = json.loads(response)

                    if data.get('type') == 'heartbeat':
                        print(f"心跳响应: {data}")

                    await asyncio.sleep(5)  # 每5秒发送一次心跳

        except Exception as e:
            print(f"监控连接失败: {e}")


async def example_basic_streaming():
    """基本流式TTS示例"""
    client = TTSWebSocketClient()

    print("=== 基本流式TTS示例 ===\n")

    # 流式文本转语音
    audio = await client.synthesize_stream(
        text="你好，这是流式TTS测试！",
        voice_id="default",
        emotion="happy",
        emotion_intensity=0.8,
        output_file="stream_output.wav"
    )

    print(f"\n流式TTS完成，音频大小: {len(audio)} 字节")


async def example_emotion_streaming():
    """情感流式TTS示例"""
    client = TTSWebSocketClient()

    print("=== 情感流式TTS示例 ===\n")

    # 不同情感的语音
    emotions = [
        ("neutral", "这是一句平静的话。"),
        ("happy", "今天天气真好，我很开心！"),
        ("sad", "突然下起了雨，心情有点低落。"),
        ("angry", "这真是太让人生气了！"),
        ("excited", "太棒了，我们成功了！")
    ]

    for emotion, text in emotions:
        print(f"\n合成情感语音: {emotion}")
        output_file = f"emotion_{emotion}.wav"
        await client.synthesize_stream(
            text=text,
            voice_id="default",
            emotion=emotion,
            emotion_intensity=0.9,
            output_file=output_file
        )


async def example_batch_streaming():
    """批量流式TTS示例"""
    client = TTSWebSocketClient()

    print("=== 批量流式TTS示例 ===\n")

    texts = [
        "第一句话：欢迎使用TTS服务。",
        "第二句话：我们提供高质量的语音合成。",
        "第三句话：支持多种情感和音色。",
        "第四句话：谢谢您的使用！"
    ]

    for i, text in enumerate(texts, 1):
        print(f"\n合成第 {i} 句...")
        output_file = f"batch_{i}.wav"
        await client.synthesize_stream(
            text=text,
            voice_id="default",
            output_file=output_file
        )


async def example_connection_monitor():
    """连接监控示例"""
    client = TTSWebSocketClient()

    print("=== 连接监控示例 ===\n")

    # 监控连接状态
    await client.monitor_connection()


if __name__ == "__main__":
    # 运行示例
    import sys

    example = sys.argv[1] if len(sys.argv) > 1 else "basic"

    if example == "basic":
        asyncio.run(example_basic_streaming())
    elif example == "emotion":
        asyncio.run(example_emotion_streaming())
    elif example == "batch":
        asyncio.run(example_batch_streaming())
    elif example == "monitor":
        asyncio.run(example_connection_monitor())
    else:
        print(f"未知示例: {example}")
        print("可用示例: basic, emotion, batch, monitor")
