"""
阿里云TTS客户端
==============

集成阿里云在线TTS服务，提供高质量的粤语语音合成功能。
简化版本，使用标准HTTP请求，兼容Python 3.10 + ROS2环境。

作者: Dev Agent
"""

import json
import base64
import requests
import uuid
import time
import hashlib
import hmac
from typing import Optional, Dict, Any, Tuple
import numpy as np
import soundfile as sf
import logging
from pathlib import Path
import tempfile
import os


class AliyunTTSClient:
    """阿里云TTS客户端"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化阿里云TTS客户端

        Args:
            config: 配置字典
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)

        # 阿里云配置
        self.access_key_id = self.config.get('access_key_id', os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', ''))
        self.access_key_secret = self.config.get('access_key_secret', os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', ''))
        self.region = self.config.get('region', 'cn-shanghai')
        self.endpoint = f"https://nls-gateway.{self.region}.aliyuncs.com/stream/v1/tts"
        # ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
        # 使用安全配置管理器获取appkey
        try:
            from core.security.security_config_manager import get_security_manager
            security_manager = get_security_manager()
            self.app_key = self.config.get('app_key', security_manager.get_config('aliyun_nls_appkey'))
        except Exception as e:
            self.logger.warning(f"安全配置管理器不可用，使用环境变量: {e}")
            self.app_key = self.config.get('app_key', os.getenv('ALIYUN_NLS_APPKEY', ''))
        # 动态获取Token - 使用Token管理器
        try:
            from aliyun_nls_token_manager import AliyunNLSTokenManager
            token_manager = AliyunNLSTokenManager()
            self.token_manager = token_manager
            self.token = token_manager.get_token()
            if self.token:
                self.logger.info("✅ Token管理器集成成功")
            else:
                self.logger.warning("⚠️ Token管理器暂时无法获取Token")
        except Exception as e:
            self.logger.warning(f"⚠️ Token管理器初始化失败: {e}")
            self.token_manager = None
            self.token = self.config.get('token', os.getenv('ALIBABA_CLOUD_TOKEN', ''))

        # 验证token
        if not self.token or self.token == 'your_token_here':
            self.logger.warning("⚠️ 未配置真实的ALIBABA_CLOUD_TOKEN，需要从阿里云智能语音服务控制台获取")

        # 默认粤语音色
        self.cantonese_voices = {
            'xiaoxiao': '晓晓（标准女声）',
            'xiaoyi': '晓伊（温柔女声）',
            'xiaoming': '晓峰（稳重男声）',
            'xiaoyun': '晓云（知性女声）',
        }
        self.default_voice = self.config.get('default_voice', 'xiaoxiao')
        self.default_volume = self.config.get('default_volume', 50)
        self.default_format = self.config.get('default_format', 'wav')
        self.default_sample_rate = self.config.get('default_sample_rate', 22050)

        # 当前音色状态
        self.current_voice_id: Optional[str] = self.default_voice

        # 验证配置
        self._validate_config()

        self.logger.info("✓ 阿里云TTS客户端初始化完成")
        self.logger.info(f"  - 区域: {self.region}")
        self.logger.info(f"  - 默认粤语音色: {self.default_voice}")
        self.logger.info(f"  - 可用粤语音色: {list(self.cantonese_voices.keys())}")
        self.logger.info(f"  - 访问密钥配置: {'✓' if self.access_key_id else '✗'}")
        self.logger.info(f"  - App Key配置: {'✓' if self.app_key else '✗'}")

    def _validate_config(self):
        """验证配置"""
        if not self.access_key_id:
            self.logger.warning("⚠️ 未配置阿里云AccessKeyID (ALIBABA_CLOUD_ACCESS_KEY_ID)")
        if not self.access_key_secret:
            self.logger.warning("⚠️ 未配置阿里云AccessKeySecret (ALIBABA_CLOUD_ACCESS_KEY_SECRET)")
        if not self.app_key:
            self.logger.warning("⚠️ 未配置阿里云App Key (ALIBABA_CLOUD_APP_KEY)")

    def _make_request(self, text: str, voice: str = None, **params) -> Optional[bytes]:
        """
        发送TTS请求

        Args:
            text: 待合成文本
            voice: 粤语音色
            **params: 其他参数

        Returns:
            音频数据（bytes）或None
        """
        if not voice:
            voice = self.default_voice

        # 构造请求参数（使用正确的NLS Gateway TTS格式）
        timestamp = int(time.time() * 1000)
        request_data = {
            'appkey': self.app_key,
            'timestamp': timestamp,
            'sign_type': 'HMAC-SHA1',
            'sign_version': '1.0',
            'format': params.get('format', self.default_format),
            'sample_rate': params.get('sample_rate', self.default_sample_rate),
            'voice': voice,
            'text': text,
            'volume': params.get('volume', self.default_volume),
            'speed': params.get('speed', 0),
            'text_encoding': 'utf8'
        }

        # 动态获取token
        current_token = self._get_valid_token()
        if current_token:
            request_data['token'] = current_token
        else:
            self.logger.error("❌ 无法获取有效的Token")
            return None

        # 生成签名（使用正确格式）
        # 签名字符串格式: key=value&key=value
        sign_parts = []
        sign_keys = ['appkey', 'timestamp', 'token', 'sign_type', 'sign_version', 'format', 'sample_rate', 'voice', 'text', 'volume', 'speed', 'text_encoding']
        for key in sign_keys:
            if key in request_data:
                sign_parts.append(f'{key}={request_data[key]}')

        string_to_sign = '&'.join(sign_parts)

        # 生成HMAC-SHA1签名
        signature = base64.b64encode(
            hmac.new(self.access_key_secret.encode(), string_to_sign.encode(), hashlib.sha1).digest()
        ).decode()
        request_data['signature'] = signature

        self.logger.info(f"✓ TTS请求参数: {json.dumps(request_data, ensure_ascii=False)}")

        try:
            # 添加Basic Auth头
            auth_string = base64.b64encode(f'{self.access_key_id}:{self.access_key_secret}'.encode()).decode()
            headers = {
                'Authorization': f'Basic {auth_string}',
                'Content-Type': 'application/json; charset=UTF-8'
            }

            # 发送POST请求
            response = requests.post(
                self.endpoint,
                json=request_data,
                headers=headers,
                timeout=10
            )

            # 检查响应状态
            if response.status_code != 200:
                self.logger.error(f"HTTP请求失败: {response.status_code}, {response.text}")
                return None

            # 解析响应
            result = response.json()

            # 检查业务状态
            if 'status' in result:
                if result['status'] == 'SUCCESS':
                    if 'audio_file' in result:
                        # 解码音频数据
                        audio_data = base64.b64decode(result['audio_file'])
                        return audio_data
                    else:
                        self.logger.error("响应中缺少音频数据")
                        return None
                else:
                    self.logger.error(f"TTS请求失败: {result}")
                    return None

            # 兼容旧版本格式
            if 'audio_file' in result:
                audio_data = base64.b64decode(result['audio_file'])
                return audio_data

            self.logger.error(f"未知响应格式: {result}")
            return None

        except requests.exceptions.Timeout:
            self.logger.error("阿里云TTS请求超时")
            return None
        except requests.exceptions.RequestException as e:
            self.logger.error(f"网络请求异常: {e}")
            return None
        except Exception as e:
            self.logger.error(f"阿里云TTS请求失败: {e}")
            return None

    def _get_valid_token(self) -> Optional[str]:
        """获取有效的Token"""
        try:
            # 如果有Token管理器，使用动态获取
            if self.token_manager:
                token = self.token_manager.get_token()
                if token:
                    self.logger.debug(f"✅ 从Token管理器获取Token成功: {token[:10]}...")
                    return token
                else:
                    self.logger.warning("⚠️ Token管理器暂时无法获取Token")

            # 回退到静态Token
            if self.token and self.token != 'your_token_here':
                self.logger.debug(f"✅ 使用静态Token: {self.token[:10]}...")
                return self.token

            self.logger.error("❌ 无法获取有效的Token")
            return None

        except Exception as e:
            self.logger.error(f"❌ 获取Token失败: {e}")
            return None

    def synthesize(self, text: str, **kwargs) -> Tuple[np.ndarray, float]:
        """
        语音合成

        Args:
            text: 输入文本
            **kwargs: 合成参数
                - voice: 粤语音色
                - sample_rate: 采样率
                - speed: 语速 (-500 到 500)
                - volume: 音量 (0-100)
                - pitch: 音调 (0.5-2.0)

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

            # 读取音频文件
            try:
                audio, sample_rate = sf.read(temp_file)
                synthesis_time = time.time() - start_time
                self.logger.info(f"✓ 语音合成成功: '{text[:20]}...' -> {len(audio)}采样点 -> {synthesis_time:.3f}s")
                return audio, synthesis_time
            finally:
                # 清理临时文件
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

    def switch_voice(self, voice_id: str) -> bool:
        """
        切换粤语音色

        Args:
            voice_id: 粤语音色ID

        Returns:
            切换是否成功
        """
        start_time = time.time()

        # 检查音色是否在支持列表中
        if voice_id not in self.cantonese_voices:
            self.logger.error(f"✗ 粤语音色不存在: {voice_id}")
            return False

        self.current_voice_id = voice_id
        switch_time = (time.time() - start_time) * 1000

        voice_name = self.cantonese_voices.get(voice_id, voice_id)
        self.logger.info(f"✓ 粤语音色切换成功: {voice_id} ({voice_name}) (耗时: {switch_time:.2f}ms)")
        return True

    def get_available_voices(self) -> Dict[str, str]:
        """
        获取可用的粤语音色列表

        Returns:
            音色ID到音色名称的映射
        """
        return self.cantonese_voices.copy()

    def benchmark(self, test_texts: list, **kwargs) -> Dict[str, float]:
        """
        性能基准测试

        Args:
            test_texts: 测试文本列表
            **kwargs: 合成参数

        Returns:
            性能指标字典
        """
        times = []
        for text in test_texts:
            start = time.time()
            try:
                self.synthesize(text, **kwargs)
                times.append(time.time() - start)
            except Exception as e:
                self.logger.error(f"基准测试失败: {e}")
                continue

        if not times:
            return {'error': 'no successful syntheses'}

        return {
            'avg_time': np.mean(times),
            'min_time': np.min(times),
            'max_time': np.max(times),
            'std_time': np.std(times),
            'total_tests': len(times),
        }

    def get_client_info(self) -> Dict[str, Any]:
        """获取客户端信息"""
        return {
            'client_type': 'Aliyun Online TTS',
            'region': self.region,
            'default_voice': self.default_voice,
            'available_voices': list(self.cantonese_voices.keys()),
            'default_format': self.default_format,
            'default_sample_rate': self.default_sample_rate,
            'app_key_configured': bool(self.app_key),
            'access_key_configured': bool(self.access_key_id),
            'token_configured': bool(self.token),
        }

    def __repr__(self) -> str:
        return f"AliyunTTSClient(region={self.region}, voice={self.default_voice}, config={'OK' if self.access_key_id and self.app_key else 'INCOMPLETE'})"
