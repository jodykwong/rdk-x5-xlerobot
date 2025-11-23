"""
阿里云TTS系统
=============

统一的TTS系统，集成阿里云在线TTS服务，替代本地VITS模型。

作者: Dev Agent
"""

import os
import logging
from typing import Optional, Dict, Any, Tuple
import numpy as np

# 导入阿里云TTS客户端
from .engine.aliyun_tts_client import AliyunTTSClient
from .config.aliyun_config_manager import AliyunConfigManager


class AliyunTTSSystem:
    """阿里云TTS系统"""

    def __init__(self, config_path: Optional[str] = None):
        """
        初始化阿里云TTS系统

        Args:
            config_path: 配置文件路径
        """
        self.logger = logging.getLogger(__name__)
        self.logger.info("正在初始化阿里云TTS系统...")

        # 加载配置
        self.config_manager = AliyunConfigManager(config_path)
        self.config_manager.print_config_status()

        # 初始化阿里云TTS客户端
        aliyun_config = self.config_manager.get_aliyun_config()
        self.client = AliyunTTSClient(aliyun_config)

        # 系统状态
        self.is_initialized = False
        self.initialized_time = None

        # 统计信息
        self.synthesis_count = 0
        self.total_synthesis_time = 0.0
        self.cache_hit_count = 0

        # 初始化系统
        self._initialize_system()

    def _initialize_system(self):
        """初始化系统"""
        try:
            # 检查配置
            if not self.config_manager.validate():
                self.logger.error("✗ 配置验证失败")
                return

            # 获取客户端信息
            client_info = self.client.get_client_info()
            self.logger.info(f"✓ 客户端信息: {client_info}")

            # 预热系统（如果需要）
            self._warm_up()

            # 初始化成功
            self.is_initialized = True
            self.initialized_time = self.config_manager.get('performance.timeout')

            self.logger.info("✓ 阿里云TTS系统初始化完成")
            self.logger.info(f"  - 引擎类型: {client_info['client_type']}")
            self.logger.info(f"  - 默认粤语音色: {client_info['default_voice']}")
            self.logger.info(f"  - 可用音色数: {len(client_info['available_voices'])}")
            self.logger.info(f"  - 音频格式: {client_info['default_format']}")

        except Exception as e:
            self.logger.error(f"✗ 系统初始化失败: {e}")
            raise

    def _warm_up(self):
        """预热系统"""
        try:
            self.logger.info("正在进行系统预热...")
            # 可以在这里添加预热逻辑，比如测试API连接等
            self.logger.info("✓ 系统预热完成")
        except Exception as e:
            self.logger.warning(f"⚠️ 系统预热失败: {e}")

    def synthesize(self, text: str, **kwargs) -> Tuple[np.ndarray, float]:
        """
        语音合成

        Args:
            text: 输入文本（粤语）
            **kwargs: 合成参数
                - voice: 粤语音色ID
                - sample_rate: 采样率
                - speed: 语速 (-500 到 500)
                - volume: 音量 (0-100)
                - pitch: 音调 (0.5-2.0)

        Returns:
            元组: (音频数据, 合成时间)
        """
        if not self.is_initialized:
            raise RuntimeError("TTS系统未初始化")

        try:
            # 调用客户端合成
            audio, synthesis_time = self.client.synthesize(text, **kwargs)

            # 更新统计
            self.synthesis_count += 1
            self.total_synthesis_time += synthesis_time

            self.logger.info(f"✓ 语音合成完成: '{text[:20]}...' (第{self.synthesis_count}次)")
            return audio, synthesis_time

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
        if not self.is_initialized:
            self.logger.error("TTS系统未初始化")
            return False

        try:
            return self.client.synthesize_to_file(text, output_path, **kwargs)
        except Exception as e:
            self.logger.error(f"✗ 合成到文件失败: {e}")
            return False

    def switch_voice(self, voice_id: str) -> bool:
        """
        切换粤语音色

        Args:
            voice_id: 粤语音色ID

        Returns:
            切换是否成功
        """
        if not self.is_initialized:
            self.logger.error("TTS系统未初始化")
            return False

        return self.client.switch_voice(voice_id)

    def get_available_voices(self) -> Dict[str, str]:
        """
        获取可用的粤语音色列表

        Returns:
            音色ID到音色名称的映射
        """
        if not self.is_initialized:
            return {}

        return self.client.get_available_voices()

    def benchmark(self, test_texts: list, **kwargs) -> Dict[str, float]:
        """
        性能基准测试

        Args:
            test_texts: 测试文本列表
            **kwargs: 合成参数

        Returns:
            性能指标字典
        """
        if not self.is_initialized:
            self.logger.error("TTS系统未初始化")
            return {}

        return self.client.benchmark(test_texts, **kwargs)

    def get_system_info(self) -> Dict[str, Any]:
        """
        获取系统信息

        Returns:
            系统信息字典
        """
        if not self.is_initialized:
            return {}

        client_info = self.client.get_client_info()

        return {
            'system_type': 'Aliyun TTS',
            'client_info': client_info,
            'is_initialized': self.is_initialized,
            'initialized_time': self.initialized_time,
            'statistics': {
                'synthesis_count': self.synthesis_count,
                'total_synthesis_time': self.total_synthesis_time,
                'avg_synthesis_time': self.total_synthesis_time / max(self.synthesis_count, 1),
                'cache_hit_count': self.cache_hit_count,
            },
            'config_status': {
                'access_key_configured': bool(self.config_manager.get('access_key_id')),
                'app_key_configured': bool(self.config_manager.get('app_key')),
                'token_configured': bool(self.config_manager.get('token')),
                'is_ready': self.is_initialized,
            }
        }

    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        获取性能指标

        Returns:
            性能指标字典
        """
        if not self.is_initialized or self.synthesis_count == 0:
            return {
                'synthesis_count': 0,
                'avg_synthesis_time': 0.0,
                'total_synthesis_time': 0.0,
            }

        return {
            'synthesis_count': self.synthesis_count,
            'total_synthesis_time': self.total_synthesis_time,
            'avg_synthesis_time': self.total_synthesis_time / self.synthesis_count,
            'cache_hit_count': self.cache_hit_count,
            'cache_hit_rate': self.cache_hit_count / max(self.synthesis_count, 1),
        }

    def shutdown(self):
        """关闭系统"""
        self.logger.info("正在关闭阿里云TTS系统...")
        # 这里可以添加清理逻辑，比如关闭连接等
        self.is_initialized = False
        self.logger.info("✓ 阿里云TTS系统已关闭")

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()

    def __repr__(self) -> str:
        status = "已初始化" if self.is_initialized else "未初始化"
        return f"AliyunTTSSystem({status}, voices={len(self.client.cantonese_voices) if self.client else 0})"


# 便利函数
def create_aliyun_tts_system(config_path: Optional[str] = None) -> AliyunTTSSystem:
    """
    创建阿里云TTS系统实例

    Args:
        config_path: 配置文件路径

    Returns:
        阿里云TTS系统实例
    """
    return AliyunTTSSystem(config_path)


def synthesize_text(text: str, output_path: str, config_path: Optional[str] = None, **kwargs) -> bool:
    """
    便利函数：直接合成文本到文件

    Args:
        text: 输入文本
        output_path: 输出文件路径
        config_path: 配置文件路径
        **kwargs: 合成参数

    Returns:
        是否成功
    """
    with create_aliyun_tts_system(config_path) as tts:
        return tts.synthesize_to_file(text, output_path, **kwargs)


# 示例用法
if __name__ == "__main__":
    # 配置日志
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # 创建TTS系统
    tts_system = create_aliyun_tts_system()

    if tts_system.is_initialized:
        # 显示系统信息
        print("\n=== 阿里云TTS系统信息 ===")
        system_info = tts_system.get_system_info()
        print(f"系统类型: {system_info['system_type']}")
        print(f"客户端信息: {system_info['client_info']}")

        # 显示可用粤语音色
        print("\n=== 可用粤语音色 ===")
        voices = tts_system.get_available_voices()
        for voice_id, voice_name in voices.items():
            print(f"  {voice_id}: {voice_name}")

        # 测试合成
        print("\n=== 测试语音合成 ===")
        test_text = "早晨，你好啊！今日天气好好。"
        output_file = "/tmp/test_cantonese.wav"

        if tts_system.synthesize_to_file(test_text, output_file):
            print(f"✓ 语音合成成功: {output_file}")
        else:
            print("✗ 语音合成失败")
    else:
        print("✗ TTS系统初始化失败")
