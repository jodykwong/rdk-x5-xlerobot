"""
DashScope TTS客户端
==================

使用DashScope API进行TTS语音合成。

作者: Dev Agent
**严禁Mock数据** - 使用真实API和密钥
"""

import os
import json
import base64
import requests
import time
from typing import Optional, Dict, Any, Tuple
import numpy as np
import soundfile as sf
import logging
from pathlib import Path
import tempfile


class DashScopeTTSClient:
    """DashScope TTS客户端"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化TTS客户端
        
        Args:
            config: 配置字典
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)
        
        # 获取API密钥
        self.api_key = os.getenv('QWEN_API_KEY') or os.getenv('DASHSCOPE_API_KEY')
        if not self.api_key:
            raise ValueError("未设置QWEN_API_KEY环境变量")
        
        # DashScope TTS API端点 - 使用正确的端点
        self.endpoint = "https://dashscope.aliyuncs.com/api/v1/services/aigc/t2a_v2/audio-generation"
        
        # 默认配置
        self.cantonese_voices = {
            'female-shaonv': '少女音',
            'male-qingshu': '青年男声',
            'xiaoxiao': '晓晓',
        }
        self.default_voice = self.config.get('voice', 'female-shaonv')
        self.default_format = self.config.get('format', 'wav')
        self.default_sample_rate = self.config.get('sample_rate', 22050)
        
        self.logger.info("✓ DashScope TTS客户端初始化完成")
        self.logger.info(f"  - 端点: {self.endpoint}")
        self.logger.info(f"  - API密钥: {self.api_key[:20]}...")
        self.logger.info(f"  - 默认音色: {self.default_voice}")

    def _make_request(self, text: str, voice: str = None, **params) -> Optional[bytes]:
        """
        发送TTS请求
        
        Args:
            text: 待合成文本
            voice: 音色
            **params: 其他参数
        
        Returns:
            音频数据（bytes）或None
        """
        if not voice:
            voice = self.default_voice
        
        # 构造请求参数
        request_data = {
            "model": "qwen-tts",
            "input": {
                "text": text
            },
            "parameters": {
                "voice": voice,
                "format": params.get('format', self.default_format),
                "sample_rate": params.get('sample_rate', self.default_sample_rate),
                "speed": params.get('speed', 1.0),
                "vol": params.get('volume', 1.0),
                "pitch": params.get('pitch', 0.0)
            }
        }
        
        headers = {
            'Authorization': f'Bearer {self.api_key}',
            'Content-Type': 'application/json'
        }
        
        try:
            # 发送POST请求
            response = requests.post(
                self.endpoint,
                json=request_data,
                headers=headers,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                
                # 检查响应格式
                if 'output' in result and 'audio' in result['output']:
                    audio_base64 = result['output']['audio']
                    audio_data = base64.b64decode(audio_base64)
                    return audio_data
                else:
                    self.logger.error(f"响应格式错误: {result}")
                    return None
            else:
                self.logger.error(f"API请求失败: {response.status_code}, {response.text}")
                return None
                
        except Exception as e:
            self.logger.error(f"请求异常: {e}")
            return None

    def synthesize(self, text: str, **kwargs) -> Tuple[np.ndarray, float]:
        """
        语音合成
        
        Args:
            text: 输入文本
            **kwargs: 合成参数
        
        Returns:
            元组: (音频数据, 合成时间)
        """
        start_time = time.time()
        
        try:
            # 发送TTS请求
            audio_data = self._make_request(text, **kwargs)
            
            if audio_data is None:
                raise ValueError("TTS请求失败，未获取到音频数据")
            
            # 解析音频数据
            with tempfile.NamedTemporaryFile(suffix=f'.{self.default_format}', delete=False) as f:
                f.write(audio_data)
                temp_file = f.name
            
            try:
                audio, sample_rate = sf.read(temp_file)
                synthesis_time = time.time() - start_time
                self.logger.info(f"✓ 语音合成成功: '{text[:20]}...' -> {len(audio)}采样点 -> {synthesis_time:.3f}s")
                return audio, synthesis_time
            finally:
                Path(temp_file).unlink(missing_ok=True)
        
        except Exception as e:
            self.logger.error(f"✗ 语音合成失败: {e}")
            raise

    def synthesize_to_file(self, text: str, output_path: str, **kwargs) -> bool:
        """
        直接合成到文件
        
        Args:
            text: 输入文本
            output_path: 输出文件路径
            **kwargs: 合成参数
        
        Returns:
            是否成功
        """
        try:
            audio, synthesis_time = self.synthesize(text, **kwargs)
            
            # 确保输出目录存在
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            
            # 写入音频文件
            sf.write(output_path, audio, 22050)
            
            self.logger.info(f"✓ 音频已保存: {output_path} (耗时: {synthesis_time:.3f}s)")
            return True
        
        except Exception as e:
            self.logger.error(f"✗ 合成失败: {e}")
            return False

    def get_available_voices(self) -> Dict[str, str]:
        """获取可用音色"""
        return self.cantonese_voices.copy()

    def switch_voice(self, voice_id: str) -> bool:
        """
        切换音色
        
        Args:
            voice_id: 音色ID
        
        Returns:
            切换是否成功
        """
        if voice_id not in self.cantonese_voices:
            self.logger.error(f"✗ 音色不存在: {voice_id}")
            return False
        
        self.default_voice = voice_id
        self.logger.info(f"✓ 音色切换成功: {voice_id}")
        return True

    def get_client_info(self) -> Dict[str, Any]:
        """获取客户端信息"""
        return {
            'client_type': 'DashScope TTS',
            'endpoint': self.endpoint,
            'api_key_configured': bool(self.api_key),
            'default_voice': self.default_voice,
            'available_voices': list(self.cantonese_voices.keys()),
            'format': self.default_format,
        }


# 便利函数
def create_dashscope_tts_client(config: Optional[Dict[str, Any]] = None) -> DashScopeTTSClient:
    """创建DashScope TTS客户端"""
    return DashScopeTTSClient(config)


if __name__ == "__main__":
    import sys
    sys.path.insert(0, '/home/sunrise/xlerobot/src')
    
    # 测试
    try:
        client = create_dashscope_tts_client()
        
        print(f"客户端信息: {client.get_client_info()}")
        print(f"可用音色: {client.get_available_voices()}")
        
        # 测试合成
        test_text = "早晨，你好啊！"
        output_file = "/tmp/test_dashscope.wav"
        
        if client.synthesize_to_file(test_text, output_file):
            print(f"✓ 测试成功: {output_file}")
        else:
            print("✗ 测试失败")
        
    except Exception as e:
        print(f"错误: {e}")
