"""
TTS缓存管理器

提供TTS音频缓存和文本缓存功能，支持LRU淘汰策略。
"""

import hashlib
import json
import base64
import pickle
from typing import Optional, Dict, Any, Tuple
from datetime import datetime, timedelta
from collections import OrderedDict
import threading
import logging

logger = logging.getLogger(__name__)


class TTSCacheManager:
    """TTS缓存管理器"""

    def __init__(self, max_size: int = 1000, ttl: int = 3600):
        """
        初始化缓存管理器

        Args:
            max_size: 最大缓存条目数
            ttl: 缓存生存时间（秒）
        """
        self.max_size = max_size
        self.ttl = ttl
        self._text_cache = OrderedDict()
        self._audio_cache = OrderedDict()
        self._text_metadata = {}
        self._audio_metadata = {}
        self._lock = threading.RLock()

    def _generate_key(self, text: str, voice_id: str, **params) -> str:
        """
        生成缓存键

        Args:
            text: 文本内容
            voice_id: 音色ID
            **params: 其他参数

        Returns:
            缓存键
        """
        key_data = {
            'text': text,
            'voice_id': voice_id,
            'language': params.get('language', 'zh-CN'),
            'emotion': params.get('emotion', 'neutral'),
            'emotion_intensity': params.get('emotion_intensity', 0.5),
            'speed': params.get('speed', 1.0),
            'pitch': params.get('pitch', 1.0),
            'volume': params.get('volume', 1.0),
            'audio_format': params.get('audio_format', 'wav'),
            'audio_quality': params.get('audio_quality', 'medium'),
            'sample_rate': params.get('sample_rate', 22050)
        }
        key_str = json.dumps(key_data, sort_keys=True)
        return hashlib.sha256(key_str.encode()).hexdigest()

    def get_audio(self, text: str, voice_id: str, **params) -> Optional[Tuple[bytes, Dict[str, Any]]]:
        """
        获取缓存的音频数据

        Args:
            text: 文本内容
            voice_id: 音色ID
            **params: 其他参数

        Returns:
            (音频数据, 元数据) 或 None
        """
        key = self._generate_key(text, voice_id, **params)

        with self._lock:
            if key in self._audio_cache:
                metadata = self._audio_metadata.get(key)
                if metadata:
                    # 检查是否过期
                    if datetime.now() < metadata['expires_at']:
                        # 更新访问顺序
                        self._audio_cache.move_to_end(key)
                        audio_data = self._audio_cache[key]
                        logger.debug(f"缓存命中: {key}")
                        return audio_data, metadata
                    else:
                        # 过期，删除
                        del self._audio_cache[key]
                        del self._audio_metadata[key]
                        logger.debug(f"缓存过期: {key}")

        return None

    def set_audio(self, text: str, voice_id: str, audio_data: bytes, **params) -> None:
        """
        设置音频缓存

        Args:
            text: 文本内容
            voice_id: 音色ID
            audio_data: 音频数据
            **params: 其他参数
        """
        key = self._generate_key(text, voice_id, **params)
        metadata = {
            'created_at': datetime.now(),
            'expires_at': datetime.now() + timedelta(seconds=self.ttl),
            'text': text,
            'voice_id': voice_id,
            'audio_size': len(audio_data),
            **params
        }

        with self._lock:
            # 如果已存在，先删除
            if key in self._audio_cache:
                del self._audio_cache[key]
                del self._audio_metadata[key]

            # 添加新缓存
            self._audio_cache[key] = audio_data
            self._audio_metadata[key] = metadata

            # 检查大小限制并淘汰
            self._evict_audio_cache()

        logger.debug(f"缓存设置: {key}, 大小: {len(audio_data)} bytes")

    def _evict_audio_cache(self) -> None:
        """淘汰过期或超限的缓存"""
        now = datetime.now()

        # 删除过期缓存
        expired_keys = [
            key for key, metadata in self._audio_metadata.items()
            if metadata['expires_at'] <= now
        ]
        for key in expired_keys:
            del self._audio_cache[key]
            del self._audio_metadata[key]
            logger.debug(f"删除过期缓存: {key}")

        # 如果还是超限，删除最久未使用的
        while len(self._audio_cache) > self.max_size:
            oldest_key = next(iter(self._audio_cache))
            del self._audio_cache[oldest_key]
            del self._audio_metadata[oldest_key]
            logger.debug(f"LRU淘汰缓存: {oldest_key}")

    def clear_cache(self) -> None:
        """清空所有缓存"""
        with self._lock:
            self._text_cache.clear()
            self._audio_cache.clear()
            self._text_metadata.clear()
            self._audio_metadata.clear()
        logger.info("缓存已清空")

    def get_cache_stats(self) -> Dict[str, Any]:
        """
        获取缓存统计信息

        Returns:
            统计信息字典
        """
        with self._lock:
            audio_size = sum(len(data) for data in self._audio_cache.values())
            return {
                'text_cache_count': len(self._text_cache),
                'audio_cache_count': len(self._audio_cache),
                'audio_cache_size': audio_size,
                'max_size': self.max_size,
                'ttl': self.ttl,
                'usage_ratio': len(self._audio_cache) / self.max_size
            }

    def preload_voices(self, common_texts: list, voice_ids: list) -> None:
        """
        预加载常用文本和音色

        Args:
            common_texts: 常用文本列表
            voice_ids: 音色ID列表
        """
        logger.info(f"开始预加载缓存: {len(common_texts)} 文本 x {len(voice_ids)} 音色")

        for text in common_texts:
            for voice_id in voice_ids:
                key = self._generate_key(text, voice_id)
                # 这里可以实际加载TTS引擎进行预加载
                # 目前只是标记
                pass

        logger.info("缓存预加载完成")
