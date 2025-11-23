"""
音色预加载器

负责TTS音色的预加载和管理，确保快速响应。
"""

import logging
from typing import Dict, List, Optional, Any
from concurrent.futures import ThreadPoolExecutor, as_completed
import threading

logger = logging.getLogger(__name__)


class VoiceLoader:
    """音色预加载器"""

    def __init__(self, max_workers: int = 4):
        """
        初始化音色预加载器

        Args:
            max_workers: 最大工作线程数
        """
        self.max_workers = max_workers
        self._loaded_voices = set()
        self._loading_voices = set()
        self._lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=max_workers)

    def load_voice(self, voice_id: str, voice_manager) -> bool:
        """
        加载指定音色

        Args:
            voice_id: 音色ID
            voice_manager: 音色管理器实例

        Returns:
            是否加载成功
        """
        with self._lock:
            if voice_id in self._loaded_voices:
                return True

            if voice_id in self._loading_voices:
                logger.info(f"音色 {voice_id} 正在加载中...")
                return False

            self._loading_voices.add(voice_id)

        try:
            # 实际加载音色
            if hasattr(voice_manager, 'load_voice'):
                voice_manager.load_voice(voice_id)
            elif hasattr(voice_manager, 'get_voice'):
                voice_manager.get_voice(voice_id)

            with self._lock:
                self._loaded_voices.add(voice_id)
                self._loading_voices.discard(voice_id)

            logger.info(f"音色 {voice_id} 加载成功")
            return True

        except Exception as e:
            logger.error(f"音色 {voice_id} 加载失败: {e}")
            with self._lock:
                self._loading_voices.discard(voice_id)
            return False

    def preload_voices(self, voice_ids: List[str], voice_manager) -> Dict[str, bool]:
        """
        批量预加载音色

        Args:
            voice_ids: 音色ID列表
            voice_manager: 音色管理器实例

        Returns:
            加载结果字典 {voice_id: success}
        """
        logger.info(f"开始预加载 {len(voice_ids)} 个音色")

        results = {}
        futures = {}

        # 提交加载任务
        with self._lock:
            for voice_id in voice_ids:
                if voice_id not in self._loaded_voices and voice_id not in self._loading_voices:
                    self._loading_voices.add(voice_id)
                    future = self._executor.submit(self._load_voice_safe, voice_id, voice_manager)
                    futures[future] = voice_id

        # 等待结果
        for future in as_completed(futures):
            voice_id = futures[future]
            try:
                success = future.result()
                results[voice_id] = success
                if success:
                    with self._lock:
                        self._loaded_voices.add(voice_id)
                with self._lock:
                    self._loading_voices.discard(voice_id)
            except Exception as e:
                logger.error(f"预加载音色 {voice_id} 失败: {e}")
                results[voice_id] = False
                with self._lock:
                    self._loading_voices.discard(voice_id)

        # 添加已加载的音色
        for voice_id in voice_ids:
            if voice_id in self._loaded_voices and voice_id not in results:
                results[voice_id] = True

        success_count = sum(1 for success in results.values() if success)
        logger.info(f"音色预加载完成: {success_count}/{len(voice_ids)} 成功")

        return results

    def _load_voice_safe(self, voice_id: str, voice_manager) -> bool:
        """
        安全加载音色

        Args:
            voice_id: 音色ID
            voice_manager: 音色管理器实例

        Returns:
            是否加载成功
        """
        try:
            if hasattr(voice_manager, 'load_voice'):
                voice_manager.load_voice(voice_id)
            elif hasattr(voice_manager, 'get_voice'):
                voice_manager.get_voice(voice_id)
            return True
        except Exception as e:
            logger.error(f"加载音色 {voice_id} 失败: {e}")
            return False

    def unload_voice(self, voice_id: str) -> bool:
        """
        卸载音色

        Args:
            voice_id: 音色ID

        Returns:
            是否卸载成功
        """
        with self._lock:
            if voice_id in self._loaded_voices:
                self._loaded_voices.remove(voice_id)
                logger.info(f"音色 {voice_id} 已卸载")
                return True
            return False

    def get_loaded_voices(self) -> List[str]:
        """
        获取已加载的音色列表

        Returns:
            音色ID列表
        """
        with self._lock:
            return list(self._loaded_voices)

    def is_voice_loaded(self, voice_id: str) -> bool:
        """
        检查音色是否已加载

        Args:
            voice_id: 音色ID

        Returns:
            是否已加载
        """
        with self._lock:
            return voice_id in self._loaded_voices

    def is_voice_loading(self, voice_id: str) -> bool:
        """
        检查音色是否正在加载

        Args:
            voice_id: 音色ID

        Returns:
            是否正在加载
        """
        with self._lock:
            return voice_id in self._loading_voices

    def shutdown(self) -> None:
        """关闭预加载器"""
        self._executor.shutdown(wait=True)
        logger.info("音色预加载器已关闭")
