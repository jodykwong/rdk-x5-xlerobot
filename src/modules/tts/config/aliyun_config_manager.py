"""
阿里云TTS配置管理器
==================

提供阿里云TTS系统的配置管理和参数调优功能。

作者: Dev Agent
"""

import os
import yaml
from typing import Dict, Any, Optional
from pathlib import Path
import logging


class AliyunConfigManager:
    """阿里云TTS配置管理器"""

    DEFAULT_CONFIG = {
        'access_key_id': '',
        'access_key_secret': '',
        'app_key': '',
        'token': '',
        'region': 'cn-shanghai',
        'endpoint': 'https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts',
        'default_voice': 'xiaoxiao',
        'default_volume': 50,
        'default_format': 'wav',
        'default_sample_rate': 22050,
        'cantonese_voices': {
            'xiaoxiao': '晓晓（标准女声）',
            'xiaoyi': '晓伊（温柔女声）',
            'xiaoming': '晓峰（稳重男声）',
            'xiaoyun': '晓云（知性女声）',
        },
        'synthesis_params': {
            'speed_range': [-500, 500],
            'volume_range': [0, 100],
            'pitch_range': [0.5, 2.0],
            'sample_rates': [16000, 22050, 24000, 44100],
        },
        'cache': {
            'enabled': True,
            'cache_dir': '/tmp/aliyun_tts_cache',
            'max_size': 500,
            'ttl': 3600,
        },
        'performance': {
            'timeout': 10,
            'max_retries': 3,
            'retry_delay': 1.0,
        },
        'logging': {
            'level': 'INFO',
            'enable_profiling': False,
        },
        'audio': {
            'format': 'wav',
            'sample_rate': 22050,
            'channels': 1,
            'bit_depth': 16,
        },
        'security': {
            'verify_ssl': True,
            'max_text_length': 1000,
            'rate_limit': 100,
        }
    }

    def __init__(self, config_path: Optional[str] = None):
        """
        初始化配置管理器

        Args:
            config_path: 配置文件路径
        """
        self.logger = logging.getLogger(__name__)
        self.config = self.DEFAULT_CONFIG.copy()
        self.config_path = config_path or '/home/sunrise/xlerobot/src/modules/tts/config/aliyun_config.yaml'

        if os.path.exists(self.config_path):
            self.load_config(self.config_path)
        else:
            self.logger.warning(f"配置文件不存在: {self.config_path}")

        # 加载环境变量
        self._load_from_env()

    def _load_from_env(self):
        """从环境变量加载配置"""
        env_mappings = {
            'ALIBABA_CLOUD_ACCESS_KEY_ID': 'access_key_id',
            'ALIBABA_CLOUD_ACCESS_KEY_SECRET': 'access_key_secret',
            'ALIBABA_CLOUD_APP_KEY': 'app_key',
            'ALIBABA_CLOUD_TOKEN': 'token',
        }

        for env_var, config_key in env_mappings.items():
            value = os.getenv(env_var)
            if value:
                self.config[config_key] = value
                self.logger.info(f"从环境变量加载: {env_var}")

    def load_config(self, config_path: str) -> None:
        """
        从文件加载配置

        Args:
            config_path: 配置文件路径
        """
        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                content = f.read()

                # 处理环境变量替换
                import re
                env_pattern = r'\$\{([^}]+)\}'
                def replace_env_var(match):
                    env_var = match.group(1)
                    return os.getenv(env_var, match.group(0))

                content = re.sub(env_pattern, replace_env_var, content)

                user_config = yaml.safe_load(content)

            # 深度合并配置
            self._merge_config(self.config, user_config)
            self.logger.info(f"✓ 配置已加载: {config_path}")
        except Exception as e:
            self.logger.error(f"✗ 配置加载失败: {e}")

    def save_config(self, config_path: str) -> None:
        """
        保存配置到文件

        Args:
            config_path: 配置文件路径
        """
        try:
            os.makedirs(os.path.dirname(config_path), exist_ok=True)
            with open(config_path, 'w', encoding='utf-8') as f:
                yaml.dump(self.config, f, default_flow_style=False, allow_unicode=True, indent=2)
            self.logger.info(f"✓ 配置已保存: {config_path}")
        except Exception as e:
            self.logger.error(f"✗ 配置保存失败: {e}")

    def _merge_config(self, base: Dict[str, Any], update: Dict[str, Any]) -> None:
        """深度合并配置字典"""
        for key, value in update.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                self._merge_config(base[key], value)
            else:
                base[key] = value

    def get(self, key: str, default: Any = None) -> Any:
        """
        获取配置值

        Args:
            key: 配置键，支持点号分隔
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
            'app_key',
        ]

        missing_keys = []
        for key in required_keys:
            if not self.get(key):
                missing_keys.append(key)

        if missing_keys:
            self.logger.error(f"✗ 缺少必要配置: {missing_keys}")
            return False

        return True

    def get_aliyun_config(self) -> Dict[str, Any]:
        """获取阿里云配置"""
        return {
            'access_key_id': self.get('access_key_id'),
            'access_key_secret': self.get('access_key_secret'),
            'app_key': self.get('app_key'),
            'token': self.get('token'),
            'region': self.get('region'),
            'endpoint': self.get('endpoint'),
            'default_voice': self.get('default_voice'),
            'default_volume': self.get('default_volume'),
            'default_format': self.get('default_format'),
            'default_sample_rate': self.get('default_sample_rate'),
            'cache_dir': self.get('cache.cache_dir'),
            'max_cache_size': self.get('cache.max_size'),
        }

    def is_configured(self) -> bool:
        """
        检查是否已配置

        Returns:
            是否已配置所有必要参数
        """
        return bool(self.get('access_key_id') and self.get('app_key'))

    def print_config_status(self) -> None:
        """打印配置状态"""
        self.logger.info("=== 阿里云TTS配置状态 ===")
        self.logger.info(f"Access Key ID: {'✓ 已配置' if self.get('access_key_id') else '✗ 未配置'}")
        self.logger.info(f"App Key: {'✓ 已配置' if self.get('app_key') else '✗ 未配置'}")
        self.logger.info(f"Token: {'✓ 已配置' if self.get('token') else 'ℹ 未配置（可选）'}")
        self.logger.info(f"默认音色: {self.get('default_voice')}")
        self.logger.info(f"默认格式: {self.get('default_format')}")
        self.logger.info(f"缓存目录: {self.get('cache.cache_dir')}")
        self.logger.info("=" * 30)

    def __repr__(self) -> str:
        configured = "已配置" if self.is_configured() else "未配置"
        return f"AliyunConfigManager({configured}, voice={self.get('default_voice')})"
