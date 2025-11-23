"""
TTS配置管理模块
===============

提供TTS系统的配置管理和参数调优功能。

作者: Dev Agent
"""

import os
import yaml
from typing import Dict, Any, Optional
from pathlib import Path


class TTSConfig:
    """TTS配置管理器"""

    DEFAULT_CONFIG = {
        'model': {
            'name': 'vits_cantonese_x5',
            'model_path': '/home/sunrise/xlerobot/MODELS/tts/vits_cantonese_x5_deployment/vits_cantonese_x5.bin',
            'config_path': '/home/sunrise/xlerobot/MODELS/tts/vits_cantonese_x5_deployment/vits_cantonese_x5_config_final.yaml',
            'input_shape': [1, 1, 1, 50],
            'device': 'cpu',  # cpu or npu
        },
        'text': {
            'max_length': 100,
            'min_length': 1,
            'normalize_punctuation': True,
            'split_sentences': True,
        },
        'audio': {
            'sample_rate': 22050,
            'hop_length': 256,
            'win_length': 1024,
            'format': 'wav',  # wav or mp3
            'quality': 'high',  # high, medium, low
        },
        'synthesis': {
            'noise_scale': 0.667,
            'length_scale': 1.0,
            'speaker_id': 0,
            'max_cache_size': 100,
        },
        'performance': {
            'batch_size': 1,
            'num_workers': 4,
            'use_cache': True,
            'cache_size': 1000,
        },
        'logging': {
            'level': 'INFO',
            'enable_profiling': False,
        }
    }

    def __init__(self, config_path: Optional[str] = None):
        """
        初始化配置管理器

        Args:
            config_path: 配置文件路径
        """
        self.config = self.DEFAULT_CONFIG.copy()
        self.config_path = config_path

        if config_path and os.path.exists(config_path):
            self.load_config(config_path)

    def load_config(self, config_path: str) -> None:
        """
        从文件加载配置

        Args:
            config_path: 配置文件路径
        """
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                user_config = yaml.safe_load(f)

            # 深度合并配置
            self._merge_config(self.config, user_config)
            print(f"✓ 配置已加载: {config_path}")
        except Exception as e:
            print(f"⚠️ 配置加载失败: {e}")

    def save_config(self, config_path: str) -> None:
        """
        保存配置到文件

        Args:
            config_path: 配置文件路径
        """
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True)
            print(f"✓ 配置已保存: {config_path}")
        except Exception as e:
            print(f"✗ 配置保存失败: {e}")

    def _merge_config(self, base: Dict[str, Any], update: Dict[str, Any]) -> None:
        """
        深度合并配置字典

        Args:
            base: 基础配置
            update: 更新配置
        """
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(base[key], value)
            else:
                base[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        """
        获取配置值

        Args:
            key: 配置键，支持点号分隔 (e.g., 'model.name')
            default: 默认值

        Returns:
            配置值
        """
        keys = key.split('.')
        value = self.config

        for k in keys:
            if isinstance(value, dict) and k in value:
                value = value[k]
            else:
                return default

        return value

    def set(self, key: str, value: Any) -> None:
        """
        设置配置值

        Args:
            key: 配置键，支持点号分隔
            value: 配置值
        """
        keys = key.split('.')
        config = self.config

        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]

        config[keys[-1]] = value

    def update(self, updates: Dict[str, Any]) -> None:
        """
        批量更新配置

        Args:
            updates: 更新字典
        """
        self._merge_config(self.config, updates)

    def validate(self) -> bool:
        """
        验证配置有效性

        Returns:
            配置是否有效
        """
        required_keys = [
            'model.name',
            'model.model_path',
            'audio.sample_rate',
        ]

        for key in required_keys:
            if self.get(key) is None:
                print(f"✗ 缺少必要配置: {key}")
                return False

        return True

    def get_model_config(self) -> Dict[str, Any]:
        """获取模型配置"""
        return self.get('model')

    def get_audio_config(self) -> Dict[str, Any]:
        """获取音频配置"""
        return self.get('audio')

    def get_text_config(self) -> Dict[str, Any]:
        """获取文本配置"""
        return self.get('text')

    def get_synthesis_config(self) -> Dict[str, Any]:
        """获取合成配置"""
        return self.get('synthesis')

    def __repr__(self) -> str:
        return f"TTSConfig(model={self.get('model.name')}, device={self.get('model.device')})"
