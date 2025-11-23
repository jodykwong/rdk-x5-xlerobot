"""
阿里云TTS引擎
============

集成阿里云在线TTS服务，提供高质量的粤语语音合成功能。

作者: Dev Agent
"""

import json
import base64
import uuid
import time
import hashlib
import hmac
import urllib.parse
import urllib.request
import ssl
from typing import Optional, Dict, Any, Tuple
import numpy as np
import soundfile as sf
import logging
from pathlib import Path

# 导入阿里云官方SDK
try:
    from nls.token import getToken
    from nls.speech_synthesizer import NlsSpeechSynthesizer
    logger = logging.getLogger(__name__)
    logger.info("✅ 阿里云NLS官方SDK已加载")
except ImportError:
    getToken = None
    NlsSpeechSynthesizer = None
    logger = logging.getLogger(__name__)
    logger.warning("⚠️ 阿里云NLS官方SDK未找到，将使用HTTP方式")

# 导入音色相关模块
from ..voices.voice_manager import VoiceManager
from ..voices.voice_controller import VoiceController, VoiceParameters, BlendMode
from ..voices.voice_cache import VoiceCache
from ..voices.voice_quality import VoiceQualityEvaluator


class AliyunTTSEngine:
    """阿里云TTS引擎"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化阿里云TTS引擎

        Args:
            config: 配置字典
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)

        # 阿里云配置
        self.access_key_id = self.config.get('access_key_id', '')
        self.access_key_secret = self.config.get('access_key_secret', '')
        self.region = self.config.get('region', 'cn-shanghai')
        self.endpoint = f"https://nls-gateway.aliyuncs.com/stream/v1/tts"
        self.app_key = self.config.get('app_key', '')
        self.token = self.config.get('token', '')

        # 优先使用官方SDK获取Token
        if not self.token and getToken:
            try:
                self.token = getToken(self.access_key_id, self.access_key_secret)
                if self.token:
                    self.logger.info(f"✅ 官方SDK获取Token成功: {self.token[:20]}...")
                else:
                    self.logger.warning("⚠️ 官方SDK获取Token返回空值")
            except Exception as e:
                self.logger.warning(f"⚠️ 官方SDK获取Token失败: {e}")

        # 默认语音配置
        self.default_voice = self.config.get('default_voice', 'xiaoxiao')
        self.default_volume = self.config.get('default_volume', 50)
        self.default_format = self.config.get('default_format', 'wav')

        # 初始化音色系统（适配阿里云）
        self.voice_manager = VoiceManager(config.get('voices') if config else None)
        self.voice_controller = VoiceController()
        self.voice_cache = VoiceCache()
        self.quality_evaluator = VoiceQualityEvaluator()

        # 当前音色状态
        self.current_voice_id: Optional[str] = None
        self.current_voice_profile: Optional[VoiceProfile] = None

        self.logger.info("✓ 阿里云TTS引擎初始化完成")
        self.logger.info(f"  - 区域: {self.region}")
        self.logger.info(f"  - 默认音色: {self.default_voice}")
        self.logger.info(f"  - 音色数量: {len(self.voice_manager.voices)}")

    def _sign_url(self, url: str, method: str = 'GET') -> str:
        """
        签名URL

        Args:
            url: 请求URL
            method: HTTP方法

        Returns:
            签名的URL
        """
        if not self.access_key_id or not self.access_key_secret:
            return url

        # 解析URL
        parsed_url = urllib.parse.urlparse(url)
        query = urllib.parse.parse_qs(parsed_url.query)

        # 添加公共参数
        query['SignatureMethod'] = ['HMAC-SHA1']
        query['SignatureVersion'] = ['1.0']
        query['SignatureNonce'] = [str(uuid.uuid4())]
        query['AccessKeyId'] = [self.access_key_id]
        query['Timestamp'] = [time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())]

        # 构造待签名字符串
        sorted_params = []
        for key in sorted(query.keys()):
            if key == 'Signature':
                continue
            value = query[key][0] if isinstance(query[key], list) else query[key]
            sorted_params.append(urllib.parse.quote(key, safe='') + '=' + urllib.parse.quote(value, safe=''))

        string_to_sign = method + '&' + urllib.parse.quote(parsed_url.path, safe='') + '&' + urllib.parse.quote('&'.join(sorted_params), safe='')

        # 计算签名
        key = (self.access_key_secret + '&').encode('utf-8')
        string_to_sign = string_to_sign.encode('utf-8')
        signature = base64.b64encode(hmac.new(key, string_to_sign, hashlib.sha1).digest()).decode('utf-8')

        # 添加签名到查询参数
        query['Signature'] = [signature]

        # 重新构造URL
        new_query = []
        for key in sorted(query.keys()):
            value = query[key][0] if isinstance(query[key], list) else query[key]
            new_query.append(key + '=' + urllib.parse.quote(value, safe=''))

        new_url = parsed_url.scheme + '://' + parsed_url.netloc + parsed_url.path + '?' + '&'.join(new_query)
        return new_url

    def _send_request(self, text: str, voice: str = None, **params) -> Optional[bytes]:
        """
        发送TTS请求 - 使用阿里云REST API正确格式

        Args:
            text: 待合成文本
            voice: 音色
            **params: 其他参数

        Returns:
            音频数据（bytes）或None
        """
        if not voice:
            voice = self.default_voice

        # 阿里云TTS REST API正确格式：通过URL查询参数传递
        query_params = {
            'appkey': self.app_key,
            'token': self.token,
            'text': text,
            'format': self.default_format,
            'voice': voice,
            'sample_rate': str(params.get('sample_rate', 16000)),
            'volume': str(params.get('volume', self.default_volume)),
            'speech_rate': str(params.get('speed', 0)),
            'pitch_rate': str(params.get('pitch_rate', 0)),
        }

        try:
            # 构造请求URL
            encoded_params = urllib.parse.urlencode(query_params)
            url = f"{self.endpoint}?{encoded_params}"

            self.logger.debug(f"TTS请求URL: {self.endpoint}?appkey=***&token=***&text={text[:20]}...")

            # 创建请求
            req = urllib.request.Request(url, method='GET')

            # 创建SSL上下文
            ssl_context = ssl.create_default_context()

            # 发送请求
            with urllib.request.urlopen(req, timeout=30, context=ssl_context) as response:
                # 检查响应类型
                content_type = response.headers.get('Content-Type', '')

                if 'audio' in content_type or 'octet-stream' in content_type:
                    # 直接返回音频数据
                    audio_data = response.read()
                    self.logger.info(f"✅ TTS合成成功，音频大小: {len(audio_data)} 字节")
                    return audio_data
                else:
                    # 可能是错误响应
                    result = response.read()
                    try:
                        response_data = json.loads(result.decode('utf-8'))
                        self.logger.error(f"TTS请求失败: {response_data}")
                    except:
                        self.logger.error(f"TTS请求失败，响应: {result[:200]}")
                    return None

        except urllib.error.HTTPError as e:
            error_body = e.read().decode('utf-8') if e.fp else ''
            self.logger.error(f"阿里云TTS HTTP错误 {e.code}: {e.reason}")
            self.logger.error(f"错误详情: {error_body[:500]}")
            return None
        except Exception as e:
            self.logger.error(f"阿里云TTS请求失败: {e}")
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
            # 检查缓存
            cache_key = self._generate_cache_key(text, **kwargs)
            cached_audio = self.voice_cache.get(cache_key)
            if cached_audio is not None:
                self.logger.info(f"✓ 从缓存获取音频: {len(text)}字符")
                return cached_audio, 0.0

            # 发送TTS请求
            audio_data = self._send_request(text, **kwargs)

            if audio_data is None:
                raise ValueError("TTS请求失败，未获取到音频数据")

            # 保存到缓存
            self.voice_cache.set(cache_key, (audio_data, 0.0))

            # 解析音频数据
            # 阿里云返回的音频数据需要写入临时文件再读取
            temp_file = f"/tmp/tts_{uuid.uuid4()}.{self.default_format}"
            with open(temp_file, 'wb') as f:
                f.write(audio_data)

            # 读取音频文件
            audio, sample_rate = sf.read(temp_file)

            # 清理临时文件
            Path(temp_file).unlink(missing_ok=True)

            synthesis_time = time.time() - start_time
            self.logger.info(f"语音合成完成: {len(text)}字符 -> {len(audio)}采样点 -> {synthesis_time:.3f}s")

            return audio, synthesis_time

        except Exception as e:
            self.logger.error(f"✗ 语音合成失败: {e}")
            raise

    def _generate_cache_key(self, text: str, **kwargs) -> str:
        """
        生成缓存键

        Args:
            text: 文本
            **kwargs: 参数

        Returns:
            缓存键
        """
        key_str = text + str(sorted(kwargs.items()))
        return hashlib.md5(key_str.encode('utf-8')).hexdigest()

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

            # 写入音频文件
            sf.write(output_path, audio, 22050)

            self.logger.info(f"✓ 音频已保存: {output_path} (耗时: {synthesis_time:.3f}s)")

            return True

        except Exception as e:
            self.logger.error(f"✗ 合成失败: {e}")
            return False

    def switch_voice(self, voice_id: str) -> bool:
        """
        切换音色

        Args:
            voice_id: 音色ID

        Returns:
            切换是否成功
        """
        start_time = time.time()

        # 尝试切换到指定音色
        if self.voice_manager.switch_voice(voice_id):
            self.current_voice_id = voice_id
            self.current_voice_profile = self.voice_manager.get_current_voice()

            switch_time = (time.time() - start_time) * 1000

            if switch_time < 100:
                self.logger.info(f"✓ 音色切换成功: {voice_id} (耗时: {switch_time:.2f}ms)")
                return True
            else:
                self.logger.warning(f"⚠️ 音色切换较慢: {switch_time:.2f}ms (目标: <100ms)")
                return True
        else:
            self.logger.error(f"✗ 音色切换失败: {voice_id}")
            return False

    def set_voice_parameters(self, params: VoiceParameters, transition_duration: float = 0.0) -> bool:
        """
        设置音色参数

        Args:
            params: 音色参数
            transition_duration: 过渡时间（秒）

        Returns:
            设置是否成功
        """
        # 更新当前音色的特征
        if self.current_voice_profile:
            self.current_voice_profile.characteristics.update(params.to_dict())

        return self.voice_controller.set_parameters(params, transition_duration)

    def blend_voices(self, voice1_id: str, voice2_id: str, blend_mode: BlendMode,
                     alpha: float, output_voice_id: str) -> bool:
        """
        混合两种音色

        Args:
            voice1_id: 音色1 ID
            voice2_id: 音色2 ID
            blend_mode: 混合模式
            alpha: 混合因子
            output_voice_id: 输出音色ID

        Returns:
            混合是否成功
        """
        voice1 = self.voice_manager.get_voice(voice1_id)
        voice2 = self.voice_manager.get_voice(voice2_id)

        if not voice1 or not voice2:
            self.logger.error(f"✗ 音色不存在: {voice1_id} 或 {voice2_id}")
            return False

        # 创建音色参数
        params1 = VoiceParameters(**voice1.characteristics)
        params2 = VoiceParameters(**voice2.characteristics)

        # 混合参数
        blended_params = self.voice_controller.blend_voices(params1, params2, blend_mode, alpha)

        # 创建新的音色档案
        from ..voices.voice_manager import VoiceType
        blended_voice = VoiceProfile(
            voice_id=output_voice_id,
            name=f"混合音色_{voice1.name}_{voice2.name}",
            type=VoiceType.CUSTOM,
            description=f"混合自 {voice1.name} 和 {voice2.name}",
            model_path=voice1.model_path,
            config_path=f"/home/sunrise/xlerobot/src/modules/tts/voices/configs/{output_voice_id}.yaml",
            sample_rate=voice1.sample_rate,
            speaker_id=voice1.speaker_id + 50,
            quality_score=(voice1.quality_score + voice2.quality_score) / 2,
            characteristics=blended_params.to_dict(),
            supported_languages=list(set(voice1.supported_languages + voice2.supported_languages)),
            created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            updated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            version="1.0.0",
            is_active=True
        )

        # 注册混合音色
        return self.voice_manager.register_voice(blended_voice)

    def apply_voice_to_synthesis(self, audio: np.ndarray, voice_id: str) -> np.ndarray:
        """
        在语音合成时应用音色

        Args:
            audio: 原始音频
            voice_id: 音色ID

        Returns:
            应用音色后的音频
        """
        voice = self.voice_manager.get_voice(voice_id)
        if not voice:
            self.logger.warning(f"⚠️ 音色不存在，使用默认参数: {voice_id}")
            voice_params = VoiceParameters()
        else:
            voice_params = VoiceParameters(**voice.characteristics)

        # 应用音色参数
        processed_audio = self.voice_controller.apply_voice_to_audio(
            audio, voice_params, voice.sample_rate
        )

        return processed_audio

    def evaluate_voice_quality(self, audio: np.ndarray, voice_id: str,
                              sample_rate: int) -> Dict[str, Any]:
        """
        评估音色质量

        Args:
            audio: 音频数据
            voice_id: 音色ID
            sample_rate: 采样率

        Returns:
            质量评估结果
        """
        metrics = self.quality_evaluator.evaluate_voice_quality(audio, sample_rate, voice_id)
        report = self.quality_evaluator.get_quality_report(voice_id)

        if report:
            report['metrics'] = {
                'mos_score': metrics.mos_score,
                'snr': metrics.snr,
                'thd': metrics.thd,
                'dynamic_range': metrics.dynamic_range,
                'spectral_centroid': metrics.spectral_centroid,
                'zero_crossing_rate': metrics.zero_crossing_rate,
                'mfcc_distortion': metrics.mfcc_distortion,
                'evaluation_time': metrics.evaluation_time
            }

        return report

    def preload_all_voices(self) -> Dict[str, bool]:
        """
        预加载所有音色

        Returns:
            预加载结果字典
        """
        results = {}
        for voice_id in self.voice_manager.voices.keys():
            results[voice_id] = self.voice_cache.preload_voice(voice_id)

        self.logger.info(f"✓ 已预加载 {sum(results.values())} / {len(results)} 个音色")
        return results

    def get_voice_statistics(self) -> Dict[str, Any]:
        """
        获取音色统计信息

        Returns:
            统计信息字典
        """
        voice_stats = self.voice_manager.get_voice_statistics()
        cache_stats = self.voice_cache.get_cache_stats()
        controller_stats = self.voice_controller.get_parameter_statistics()

        return {
            'voices': voice_stats,
            'cache': cache_stats,
            'controller': controller_stats,
            'current_voice': {
                'voice_id': self.current_voice_id,
                'name': self.current_voice_profile.name if self.current_voice_profile else None
            }
        }

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

    def get_model_info(self) -> Dict[str, Any]:
        """
        获取引擎信息

        Returns:
            引擎信息字典
        """
        return {
            'engine_type': 'Aliyun Online TTS',
            'region': self.region,
            'default_voice': self.default_voice,
            'default_format': self.default_format,
            'app_key_configured': bool(self.app_key),
            'access_key_configured': bool(self.access_key_id),
            'token_configured': bool(self.token),
        }

    def __repr__(self) -> str:
        return f"AliyunTTSEngine(region={self.region}, voice={self.default_voice}, app_key={'configured' if self.app_key else 'not configured'})"
