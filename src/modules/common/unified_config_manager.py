#!/usr/bin/env python3.10
"""
XLeRobot 统一配置管理器
======================

整合所有配置模块，提供统一的配置管理接口。
解决配置分散问题，提供系统级配置管理。

主要功能：
- 统一配置接口管理
- 多格式配置文件支持 (YAML/JSON)
- 环境变量自动映射
- 配置验证和热重载
- 配置缓存和性能优化

作者: Claude Code Agent
版本: 1.0
日期: 2025-11-19
"""

import os
import yaml
import json
import logging
import threading
import time
from typing import Dict, Any, Optional, Union, List
from pathlib import Path
from dataclasses import dataclass, field
from enum import Enum
import hashlib

logger = logging.getLogger(__name__)

class ConfigFormat(Enum):
    """配置文件格式枚举"""
    YAML = "yaml"
    JSON = "json"
    AUTO = "auto"  # 自动检测

@dataclass
class ConfigSource:
    """配置源定义"""
    name: str
    path: str
    format: ConfigFormat
    required: bool = True
    priority: int = 0  # 优先级，数字越大优先级越高
    checksum: Optional[str] = None

class UnifiedConfigManager:
    """
    统一配置管理器

    整合所有XLeRobot配置，提供统一的配置访问接口。
    支持多格式配置文件、环境变量覆盖、配置验证等功能。
    """

    def __init__(self, config_dir: Optional[str] = None):
        """
        初始化统一配置管理器

        Args:
            config_dir: 配置文件目录路径
        """
        self.config_dir = config_dir or self._get_default_config_dir()
        self._lock = threading.RLock()

        # 配置缓存
        self._config_cache: Dict[str, Any] = {}
        self._config_sources: Dict[str, ConfigSource] = {}
        self._last_load_time: Dict[str, float] = {}

        # 环境变量映射
        self._env_mappings = {
            # 阿里云NLS配置
            'aliyun_nls.appkey': 'ALIYUN_NLS_APPKEY',
            'aliyun_nls.authentication.access_key_id': 'ALIBABA_CLOUD_ACCESS_KEY_ID',
            'aliyun_nls.authentication.access_key_secret': 'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
            'aliyun_nls.region': 'ALIYUN_NLS_REGION',

            # 通用配置
            'system.debug': 'XLEROBOT_DEBUG',
            'system.log_level': 'XLEROBOT_LOG_LEVEL',

            # TTS配置
            'tts.voice': 'XLEROBOT_TTS_VOICE',
            'tts.volume': 'XLEROBOT_TTS_VOLUME',

            # ASR配置
            'asr.language': 'XLEROBOT_ASR_LANGUAGE',

            # LLM配置
            'llm.api_key': 'QWEN_API_KEY',
        }

        # 初始化配置源
        self._init_config_sources()

        # 加载所有配置
        self._load_all_configs()

        logger.info("✅ 统一配置管理器初始化完成")
        logger.info(f"  - 配置目录: {self.config_dir}")
        logger.info(f"  - 配置源数量: {len(self._config_sources)}")

    def _get_default_config_dir(self) -> str:
        """获取默认配置目录"""
        # 项目根目录下的config目录
        project_root = Path(__file__).parent.parent.parent.parent
        config_dir = project_root / "config"

        if not config_dir.exists():
            # 尝试创建配置目录
            config_dir.mkdir(parents=True, exist_ok=True)

        return str(config_dir)

    def _init_config_sources(self):
        """初始化配置源"""
        # 主要配置文件列表
        config_files = {
            'aliyun_nls': 'aliyun_nls_config.yaml',
            'tts': 'tts_config.yaml',
            'wake_word': 'wake_word_config.json',
            'audio': 'audio_config.json',
            'llm': 'llm_config.json',
            'system': 'system_config.yaml',
        }

        # 注册配置源
        for name, filename in config_files.items():
            file_path = os.path.join(self.config_dir, filename)
            if os.path.exists(file_path):
                self._register_config_source(name, file_path)
            else:
                logger.warning(f"⚠️ 配置文件不存在: {file_path}")
                # 创建空的配置源（可选）
                self._register_config_source(name, file_path, required=False)

    def _register_config_source(self, name: str, file_path: str,
                               format: ConfigFormat = ConfigFormat.AUTO,
                               required: bool = True, priority: int = 0):
        """
        注册配置源

        Args:
            name: 配置源名称
            file_path: 配置文件路径
            format: 配置文件格式
            required: 是否必需
            priority: 优先级
        """
        # 自动检测格式
        if format == ConfigFormat.AUTO:
            if file_path.endswith('.yaml') or file_path.endswith('.yml'):
                format = ConfigFormat.YAML
            elif file_path.endswith('.json'):
                format = ConfigFormat.JSON
            else:
                logger.warning(f"无法自动检测配置文件格式: {file_path}")
                format = ConfigFormat.YAML  # 默认使用YAML

        config_source = ConfigSource(
            name=name,
            path=file_path,
            format=format,
            required=required,
            priority=priority
        )

        self._config_sources[name] = config_source
        logger.debug(f"注册配置源: {name} -> {file_path}")

    def _load_all_configs(self):
        """加载所有配置"""
        for name, source in self._config_sources.items():
            try:
                self._load_config(name)
            except Exception as e:
                if source.required:
                    logger.error(f"❌ 必需配置加载失败: {name}, 错误: {e}")
                else:
                    logger.warning(f"⚠️ 可选配置加载失败: {name}, 错误: {e}")

    def _load_config(self, config_name: str) -> bool:
        """
        加载指定配置

        Args:
            config_name: 配置名称

        Returns:
            bool: 是否加载成功
        """
        if config_name not in self._config_sources:
            logger.error(f"❌ 未知的配置源: {config_name}")
            return False

        source = self._config_sources[config_name]

        # 检查文件是否存在
        if not os.path.exists(source.path):
            if source.required:
                logger.error(f"❌ 配置文件不存在: {source.path}")
                return False
            else:
                logger.debug(f"可选配置文件不存在: {source.path}")
                self._config_cache[config_name] = {}
                return True

        # 检查文件是否已修改（计算checksum）
        current_checksum = self._calculate_file_checksum(source.path)
        if current_checksum == source.checksum and config_name in self._config_cache:
            logger.debug(f"配置文件未修改，跳过重新加载: {config_name}")
            return True

        try:
            with open(source.path, 'r', encoding='utf-8') as f:
                if source.format == ConfigFormat.YAML:
                    config_data = yaml.safe_load(f) or {}
                elif source.format == ConfigFormat.JSON:
                    config_data = json.load(f)
                else:
                    logger.error(f"❌ 不支持的配置格式: {source.format}")
                    return False

            # 应用环境变量覆盖
            config_data = self._apply_env_overrides(config_name, config_data)

            # 验证配置
            if self._validate_config(config_name, config_data):
                self._config_cache[config_name] = config_data
                source.checksum = current_checksum
                self._last_load_time[config_name] = time.time()

                logger.info(f"✅ 配置加载成功: {config_name}")
                return True
            else:
                logger.error(f"❌ 配置验证失败: {config_name}")
                return False

        except Exception as e:
            logger.error(f"❌ 配置加载异常: {config_name}, 错误: {e}")
            return False

    def _calculate_file_checksum(self, file_path: str) -> str:
        """计算文件checksum"""
        try:
            with open(file_path, 'rb') as f:
                content = f.read()
                return hashlib.sha256(content).hexdigest()
        except Exception:
            return ""

    def _apply_env_overrides(self, config_name: str, config_data: Dict[str, Any]) -> Dict[str, Any]:
        """
        应用环境变量覆盖

        Args:
            config_name: 配置名称
            config_data: 原始配置数据

        Returns:
            应用环境变量后的配置数据
        """
        # 构建配置键到环境变量的映射
        for config_key, env_var in self._env_mappings.items():
            if config_key.startswith(config_name + '.'):
                if env_var in os.environ:
                    # 设置环境变量值
                    env_value = os.environ[env_var]

                    # 转换键路径
                    key_path = config_key[len(config_name) + 1:].split('.')

                    # 设置嵌套值
                    self._set_nested_value(config_data, key_path, env_value)

                    logger.debug(f"应用环境变量覆盖: {config_key} = {env_value}")

        return config_data

    def _set_nested_value(self, data: Dict[str, Any], key_path: List[str], value: str):
        """设置嵌套字典值"""
        current = data
        for key in key_path[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]

        # 尝试转换值的类型
        current[key_path[-1]] = self._convert_value(value)

    def _convert_value(self, value: str) -> Union[str, int, float, bool]:
        """转换值的类型"""
        # 布尔值
        if value.lower() in ('true', 'false'):
            return value.lower() == 'true'

        # 数字
        try:
            if '.' in value:
                return float(value)
            else:
                return int(value)
        except ValueError:
            pass

        # 字符串
        return value

    def _validate_config(self, config_name: str, config_data: Dict[str, Any]) -> bool:
        """
        验证配置

        Args:
            config_name: 配置名称
            config_data: 配置数据

        Returns:
            bool: 验证是否通过
        """
        try:
            if config_name == 'aliyun_nls':
                return self._validate_aliyun_nls_config(config_data)
            elif config_name == 'tts':
                return self._validate_tts_config(config_data)
            elif config_name == 'wake_word':
                return self._validate_wake_word_config(config_data)
            else:
                # 默认验证
                return True
        except Exception as e:
            logger.error(f"配置验证异常: {config_name}, 错误: {e}")
            return False

    def _validate_aliyun_nls_config(self, config_data: Dict[str, Any]) -> bool:
        """验证阿里云NLS配置"""
        auth = config_data.get('authentication', {})

        # 检查必需字段
        required_fields = ['appkey', 'access_key_id', 'access_key_secret']
        for field in required_fields:
            if not auth.get(field):
                logger.error(f"❌ 阿里云NLS配置缺少必需字段: {field}")
                return False

        return True

    def _validate_tts_config(self, config_data: Dict[str, Any]) -> bool:
        """验证TTS配置"""
        if not config_data.get('voice'):
            logger.warning("⚠️ TTS配置缺少voice字段")

        return True

    def _validate_wake_word_config(self, config_data: Dict[str, Any]) -> bool:
        """验证唤醒词配置"""
        if not config_data.get('wake_word'):
            logger.error("❌ 唤醒词配置缺少wake_word字段")
            return False

        return True

    def get_config(self, config_name: str, key_path: Optional[str] = None,
                  default: Any = None) -> Any:
        """
        获取配置值

        Args:
            config_name: 配置名称
            key_path: 配置键路径 (用.分隔，如 'authentication.access_key_id')
            default: 默认值

        Returns:
            配置值
        """
        with self._lock:
            # 确保配置已加载
            if config_name not in self._config_cache:
                if not self._load_config(config_name):
                    return default

            config_data = self._config_cache[config_name]

            # 如果没有指定键路径，返回整个配置
            if not key_path:
                return config_data

            # 获取嵌套值
            try:
                keys = key_path.split('.')
                value = config_data
                for key in keys:
                    value = value[key]
                return value
            except (KeyError, TypeError):
                logger.debug(f"配置键不存在: {config_name}.{key_path}")
                return default

    def set_config(self, config_name: str, key_path: str, value: Any) -> bool:
        """
        设置配置值（内存中，不持久化）

        Args:
            config_name: 配置名称
            key_path: 配置键路径
            value: 配置值

        Returns:
            bool: 是否设置成功
        """
        with self._lock:
            if config_name not in self._config_cache:
                self._config_cache[config_name] = {}

            keys = key_path.split('.')
            current = self._config_cache[config_name]

            for key in keys[:-1]:
                if key not in current:
                    current[key] = {}
                current = current[key]

            current[keys[-1]] = value
            logger.debug(f"设置配置: {config_name}.{key_path} = {value}")
            return True

    def reload_config(self, config_name: Optional[str] = None) -> bool:
        """
        重新加载配置

        Args:
            config_name: 配置名称，None表示重新加载所有配置

        Returns:
            bool: 是否重新加载成功
        """
        with self._lock:
            if config_name:
                return self._load_config(config_name)
            else:
                success_count = 0
                total_count = len(self._config_sources)

                for name in self._config_sources.keys():
                    if self._load_config(name):
                        success_count += 1

                success_rate = success_count / total_count if total_count > 0 else 0
                logger.info(f"配置重新加载完成: {success_count}/{total_count} ({success_rate:.1%})")
                return success_rate > 0

    def save_config(self, config_name: str) -> bool:
        """
        保存配置到文件

        Args:
            config_name: 配置名称

        Returns:
            bool: 是否保存成功
        """
        if config_name not in self._config_sources:
            logger.error(f"❌ 未知的配置源: {config_name}")
            return False

        source = self._config_sources[config_name]

        if config_name not in self._config_cache:
            logger.error(f"❌ 配置数据不存在: {config_name}")
            return False

        try:
            with open(source.path, 'w', encoding='utf-8') as f:
                if source.format == ConfigFormat.YAML:
                    yaml.dump(self._config_cache[config_name], f,
                             default_flow_style=False, allow_unicode=True)
                elif source.format == ConfigFormat.JSON:
                    json.dump(self._config_cache[config_name], f,
                             indent=2, ensure_ascii=False)

            # 更新checksum
            source.checksum = self._calculate_file_checksum(source.path)

            logger.info(f"✅ 配置保存成功: {config_name}")
            return True

        except Exception as e:
            logger.error(f"❌ 配置保存失败: {config_name}, 错误: {e}")
            return False

    def get_all_configs(self) -> Dict[str, Dict[str, Any]]:
        """获取所有配置"""
        with self._lock:
            return self._config_cache.copy()

    def get_config_sources(self) -> Dict[str, ConfigSource]:
        """获取所有配置源信息"""
        return self._config_sources.copy()

    def get_status(self) -> Dict[str, Any]:
        """获取配置管理器状态"""
        with self._lock:
            return {
                'config_dir': self.config_dir,
                'config_count': len(self._config_cache),
                'config_sources': {
                    name: {
                        'path': source.path,
                        'format': source.format.value,
                        'required': source.required,
                        'loaded': name in self._config_cache,
                        'last_load_time': self._last_load_time.get(name)
                    }
                    for name, source in self._config_sources.items()
                },
                'env_mappings_count': len(self._env_mappings)
            }

# 全局配置管理器实例
_config_manager: Optional[UnifiedConfigManager] = None
_config_lock = threading.Lock()

def get_config_manager(config_dir: Optional[str] = None) -> UnifiedConfigManager:
    """获取全局配置管理器实例"""
    global _config_manager

    if _config_manager is None:
        with _config_lock:
            if _config_manager is None:
                _config_manager = UnifiedConfigManager(config_dir)

    return _config_manager

def get_config(config_name: str, key_path: Optional[str] = None, default: Any = None) -> Any:
    """便捷函数：获取配置值"""
    manager = get_config_manager()
    return manager.get_config(config_name, key_path, default)

def set_config(config_name: str, key_path: str, value: Any) -> bool:
    """便捷函数：设置配置值"""
    manager = get_config_manager()
    return manager.set_config(config_name, key_path, value)

# 测试函数
def test_config_manager():
    """测试配置管理器"""
    logging.basicConfig(level=logging.INFO)

    print("🧪 测试统一配置管理器...")

    manager = UnifiedConfigManager()

    # 获取状态
    status = manager.get_status()
    print(f"📊 配置管理器状态: {status['config_count']} 个配置已加载")

    # 获取阿里云NLS配置
    appkey = manager.get_config('aliyun_nls', 'authentication.appkey')
    print(f"🔑 阿里云AppKey: {appkey}")

    # 获取TTS配置
    voice = manager.get_config('tts', 'voice', 'xiaoyun')
    print(f"🔊 TTS发音人: {voice}")

    print("🎉 配置管理器测试完成")

if __name__ == "__main__":
    test_config_manager()