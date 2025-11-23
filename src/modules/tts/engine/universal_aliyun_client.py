"""
通用阿里云TTS客户端
==================

支持多种认证方式的阿里云TTS客户端：
1. DASHSCOPE_API_KEY (通用)
2. QWEN_API_KEY (现有LLM使用)
3. ALIBABA_CLOUD_ACCESS_KEY_ID/SECRET + APP_KEY
4. 语音服务专用凭据

作者: Dev Agent
"""

import os
import json
import base64
import requests
import uuid
import time
import hashlib
import hmac
from typing import Optional, Dict, Any, Tuple, List
import numpy as np
import soundfile as sf
import logging
from pathlib import Path
import tempfile


class UniversalAliyunTTSClient:
    """通用阿里云TTS客户端"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化TTS客户端
        
        Args:
            config: 配置字典
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)
        
        # 尝试多种认证方式
        self.auth_methods = self._detect_auth_methods()
        
        # 默认配置
        self.cantonese_voices = {
            'xiaoxiao': '晓晓（标准女声）',
            'xiaoyi': '晓伊（温柔女声）',
            'xiaoming': '晓峰（稳重男声）',
            'xiaoyun': '晓云（知性女声）',
        }
        self.default_voice = self.config.get('default_voice', 'xiaoxiao')
        self.default_format = self.config.get('default_format', 'wav')
        self.default_sample_rate = self.config.get('default_sample_rate', 22050)
        
        # 当前认证方式
        self.current_auth = None
        
        # 验证和选择认证方式
        self._select_auth_method()
        
        self.logger.info("✓ 通用阿里云TTS客户端初始化完成")
        self.logger.info(f"  - 可用认证方式: {list(self.auth_methods.keys())}")
        self.logger.info(f"  - 当前认证: {self.current_auth}")
        self.logger.info(f"  - 默认粤语音色: {self.default_voice}")

    def _detect_auth_methods(self) -> Dict[str, Any]:
        """检测可用的认证方式"""
        methods = {}
        
        # 方式1: DASHSCOPE_API_KEY
        dashscope_key = os.getenv('DASHSCOPE_API_KEY') or os.getenv('QWEN_API_KEY')
        if dashscope_key:
            methods['DASHSCOPE_API_KEY'] = {
                'key': dashscope_key,
                'type': 'bearer',
                'priority': 1
            }
        
        # 方式2: 阿里云账号级别认证
        access_key_id = self.config.get('access_key_id') or os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID')
        access_key_secret = self.config.get('access_key_secret') or os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET')
        app_key = self.config.get('app_key') or os.getenv('ALIBABA_CLOUD_APP_KEY')
        
        if access_key_id and access_key_secret:
            methods['ACCESS_KEY'] = {
                'access_key_id': access_key_id,
                'access_key_secret': access_key_secret,
                'app_key': app_key,
                'type': 'signature',
                'priority': 2
            }
        
        # 方式3: 语音服务专用凭据
        voice_app_key = self.config.get('voice_app_key') or os.getenv('ALIBABA_VOICE_APP_KEY')
        voice_api_key = self.config.get('voice_api_key') or os.getenv('ALIBABA_VOICE_API_KEY')
        
        if voice_app_key and voice_api_key:
            methods['VOICE_SERVICE'] = {
                'app_key': voice_app_key,
                'api_key': voice_api_key,
                'type': 'appkey',
                'priority': 3
            }
        
        return methods

    def _select_auth_method(self):
        """选择认证方式"""
        if not self.auth_methods:
            raise ValueError("未检测到任何可用的阿里云认证方式")
        
        # 按优先级排序
        sorted_methods = sorted(
            self.auth_methods.items(), 
            key=lambda x: x[1].get('priority', 999)
        )
        
        # 选择第一个可用的方法
        self.current_auth, config = sorted_methods[0]
        self.auth_config = config
        
        self.logger.info(f"✓ 选择认证方式: {self.current_auth}")

    def _make_request_dashscope(self, text: str, voice: str = None, **params) -> Optional[bytes]:
        """使用DASHSCOPE_API_KEY请求"""
        # 注意：这不是标准API，需要用户配置正确的TTS端点
        # 这里是演示用的伪代码
        
        # 构造请求
        request_data = {
            'model': 'speech-synthesis',  # 假设的TTS模型
            'input': {
                'text': text,
                'voice': voice or self.default_voice
            },
            'parameters': {
                'format': self.default_format,
                'sample_rate': params.get('sample_rate', self.default_sample_rate),
                'speed': params.get('speed', 0),
                'volume': params.get('volume', 50)
            }
        }
        
        headers = {
            'Authorization': f"Bearer {self.auth_config['key']}",
            'Content-Type': 'application/json'
        }
        
        # 注意：这里需要实际的TTS端点URL
        # 通常需要使用阿里云语音服务的专用端点
        self.logger.warning("⚠️ DASHSCOPE_API_KEY认证可能需要配置TTS专用端点")
        return None

    def _make_request_voice_service(self, text: str, voice: str = None, **params) -> Optional[bytes]:
        """使用语音服务专用凭据请求"""
        endpoint = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts"
        
        request_data = {
            'appkey': self.auth_config['app_key'],
            'timestamp': int(time.time() * 1000),
            'sign_type': 'HMAC-SHA1',
            'sign_version': '1.0',
            'method': 'POST',
            'version': '2.0',
            'audio': {
                'voice': voice or self.default_voice,
                'text': text,
                'audio_format': self.default_format,
                'audio_sample_rate': params.get('sample_rate', self.default_sample_rate),
                'voice_speed': params.get('speed', 0),
                'voice_volume': params.get('volume', 50),
                'text_encoding': 'utf8'
            }
        }
        
        if 'api_key' in self.auth_config:
            request_data['token'] = self.auth_config['api_key']
        
        headers = {'Content-Type': 'application/json; charset=UTF-8'}
        
        try:
            response = requests.post(endpoint, json=request_data, headers=headers, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                if 'audio_file' in result:
                    return base64.b64decode(result['audio_file'])
            
            self.logger.error(f"TTS请求失败: {response.status_code}, {response.text}")
            return None
            
        except Exception as e:
            self.logger.error(f"TTS请求异常: {e}")
            return None

    def _make_request_access_key(self, text: str, voice: str = None, **params) -> Optional[bytes]:
        """使用access_key签名请求"""
        if 'app_key' not in self.auth_config:
            self.logger.error("ACCESS_KEY认证方式需要app_key")
            return None
        
        endpoint = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts"
        
        # 构造带签名的请求
        # 这里省略复杂的签名逻辑，简化实现
        
        request_data = {
            'appkey': self.auth_config['app_key'],
            'audio': {
                'voice': voice or self.default_voice,
                'text': text,
                'audio_format': self.default_format,
                'audio_sample_rate': params.get('sample_rate', self.default_sample_rate),
                'voice_speed': params.get('speed', 0),
                'voice_volume': params.get('volume', 50),
                'text_encoding': 'utf8'
            }
        }
        
        # 简化的签名实现
        # 实际使用时需要完整的签名算法
        
        try:
            response = requests.post(endpoint, json=request_data, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                if 'audio_file' in result:
                    return base64.b64decode(result['audio_file'])
            
            return None
            
        except Exception as e:
            self.logger.error(f"TTS请求异常: {e}")
            return None

    def synthesize(self, text: str, **kwargs) -> Tuple[np.ndarray, float]:
        """语音合成"""
        start_time = time.time()
        
        try:
            # 根据认证方式选择请求方法
            if self.current_auth == 'VOICE_SERVICE':
                audio_data = self._make_request_voice_service(text, **kwargs)
            elif self.current_auth == 'ACCESS_KEY':
                audio_data = self._make_request_access_key(text, **kwargs)
            elif self.current_auth == 'DASHSCOPE_API_KEY':
                audio_data = self._make_request_dashscope(text, **kwargs)
            else:
                raise ValueError(f"不支持的认证方式: {self.current_auth}")
            
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
        """直接合成到文件"""
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

    def get_client_info(self) -> Dict[str, Any]:
        """获取客户端信息"""
        return {
            'client_type': 'Universal Aliyun TTS',
            'current_auth': self.current_auth,
            'available_auths': list(self.auth_methods.keys()),
            'default_voice': self.default_voice,
            'default_format': self.default_format,
            'voices': list(self.cantonese_voices.keys()),
        }

    def print_auth_status(self):
        """打印认证状态"""
        self.logger.info("="*60)
        self.logger.info("阿里云TTS认证状态")
        self.logger.info("="*60)
        
        for auth_name, config in self.auth_methods.items():
            status = "✓" if auth_name == self.current_auth else " "
            key_preview = config.get('key', config.get('app_key', config.get('access_key_id', '')))
            if key_preview:
                key_preview = key_preview[:4] + '*' * (len(key_preview) - 4)
            
            self.logger.info(f"{status} {auth_name}: {key_preview}")
        
        self.logger.info("="*60)
        
        if self.current_auth == 'DASHSCOPE_API_KEY':
            self.logger.warning("⚠️ 当前使用DASHSCOPE_API_KEY，可能需要配置TTS专用端点")
            self.logger.info("建议设置阿里云语音服务专用凭据:")
            self.logger.info("  export ALIBABA_VOICE_APP_KEY=your_voice_app_key")
            self.logger.info("  export ALIBABA_VOICE_API_KEY=your_voice_api_key")


# 便利函数
def create_universal_tts_client(config: Optional[Dict[str, Any]] = None) -> UniversalAliyunTTSClient:
    """创建通用TTS客户端"""
    return UniversalAliyunTTSClient(config)


if __name__ == "__main__":
    # 测试
    import sys
    sys.path.insert(0, '/home/sunrise/xlerobot/src')
    
    from modules.tts import create_universal_tts_client
    
    try:
        client = create_universal_tts_client()
        client.print_auth_status()
        print(f"\n客户端信息: {client.get_client_info()}")
    except Exception as e:
        print(f"错误: {e}")
        print("\n配置建议:")
        print("1. 设置QWEN_API_KEY (现有LLM使用)")
        print("2. 设置ALIBABA_VOICE_APP_KEY和ALIBABA_VOICE_API_KEY (推荐)")
        print("3. 设置ALIBABA_CLOUD_ACCESS_KEY_ID/SECRET + ALIBABA_CLOUD_APP_KEY")
