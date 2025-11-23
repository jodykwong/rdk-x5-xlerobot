#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
XLeRobot Phase 1 唤醒词配置管理器
用于管理唤醒词检测的配置参数

Epic: 1 - 语音唤醒和基础识别
作者: Claude Code
创建日期: 2025-11-19
"""

import logging
import json
import os
import yaml
from typing import Dict, Any, Optional, List
from pathlib import Path
from dataclasses import dataclass, field, asdict

logger = logging.getLogger(__name__)

@dataclass
class WakeWordConfig:
    """唤醒词配置"""
    wake_word: str = "傻强"
    threshold: float = 0.7
    cooldown_period: float = 2.0
    audio_sample_rate: int = 16000
    audio_channels: int = 1
    audio_format: str = "pcm_16"
    min_audio_length: float = 0.5
    max_audio_length: float = 5.0
    enable_asr_detection: bool = True
    enable_fallback_detection: bool = True
    confidence_threshold: float = 0.6

    # 唤醒词变体
    wake_word_variants: List[str] = field(default_factory=lambda: [
        "傻强", "傻强呀", "傻强啊", "傻强仔", "阿强", "强仔"
    ])

    # 检测参数
    energy_threshold: int = 1000
    silence_threshold: float = 0.1
    min_segment_length: float = 0.3

class WakeWordConfigManager:
    """唤醒词配置管理器"""

    def __init__(self, config_file: Optional[str] = None):
        """
        初始化配置管理器

        Args:
            config_file: 配置文件路径
        """
        self.config_file = config_file or self._get_default_config_path()
        self.config = WakeWordConfig()

        # 加载配置
        self._load_config()

        logger.info("✅ 唤醒词配置管理器初始化完成")
        logger.info(f"  - 配置文件: {self.config_file}")
        logger.info(f"  - 唤醒词: {self.config.wake_word}")

    def _get_default_config_path(self) -> str:
        """获取默认配置文件路径"""
        # 在项目根目录的config目录下
        project_root = Path(__file__).parent.parent.parent
        config_dir = project_root / "config"
        config_dir.mkdir(exist_ok=True)

        return str(config_dir / "wake_word_config.yaml")

    def _load_config(self):
        """加载配置"""
        try:
            if os.path.exists(self.config_file):
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    if self.config_file.endswith('.yaml') or self.config_file.endswith('.yml'):
                        config_data = yaml.safe_load(f)
                    else:
                        config_data = json.load(f)

                if config_data:
                    # 更新配置对象
                    for key, value in config_data.items():
                        if hasattr(self.config, key):
                            setattr(self.config, key, value)

                    logger.info("✅ 配置文件加载成功")
            else:
                # 创建默认配置文件
                self._save_config()
                logger.info("✅ 创建默认配置文件")

        except Exception as e:
            logger.error(f"❌ 配置加载失败: {e}")
            logger.info("📝 使用默认配置")

    def _save_config(self):
        """保存配置"""
        try:
            # 确保目录存在
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)

            config_dict = asdict(self.config)

            with open(self.config_file, 'w', encoding='utf-8') as f:
                if self.config_file.endswith('.yaml') or self.config_file.endswith('.yml'):
                    yaml.dump(config_dict, f, default_flow_style=False, allow_unicode=True)
                else:
                    json.dump(config_dict, f, indent=2, ensure_ascii=False)

            logger.info(f"✅ 配置文件已保存: {self.config_file}")

        except Exception as e:
            logger.error(f"❌ 配置保存失败: {e}")

    def get_config(self) -> WakeWordConfig:
        """
        获取配置

        Returns:
            WakeWordConfig: 配置对象
        """
        return self.config

    def update_config(self, **kwargs):
        """
        更新配置参数

        Args:
            **kwargs: 要更新的配置参数
        """
        updated = False

        for key, value in kwargs.items():
            if hasattr(self.config, key):
                old_value = getattr(self.config, key)
                setattr(self.config, key, value)

                if old_value != value:
                    logger.info(f"✅ 配置更新: {key} = {value} (原值: {old_value})")
                    updated = True
            else:
                logger.warning(f"⚠️ 未知配置项: {key}")

        if updated:
            self._save_config()

    def set_wake_word(self, wake_word: str, variants: Optional[List[str]] = None):
        """
        设置唤醒词

        Args:
            wake_word: 唤醒词
            variants: 唤醒词变体列表
        """
        self.update_config(
            wake_word=wake_word,
            wake_word_variants=variants or [
                wake_word, f"{wake_word}呀", f"{wake_word}啊",
                f"{wake_word}仔"
            ]
        )

    def set_threshold(self, threshold: float):
        """
        设置检测阈值

        Args:
            threshold: 检测阈值（0.0-1.0）
        """
        if 0.0 <= threshold <= 1.0:
            self.update_config(threshold=threshold)
        else:
            logger.error(f"❌ 无效的检测阈值: {threshold}")

    def set_cooldown_period(self, cooldown_seconds: float):
        """
        设置冷却时间

        Args:
            cooldown_seconds: 冷却时间（秒）
        """
        if cooldown_seconds >= 0:
            self.update_config(cooldown_period=cooldown_seconds)
        else:
            logger.error(f"❌ 无效的冷却时间: {cooldown_seconds}")

    def add_wake_word_variant(self, variant: str):
        """
        添加唤醒词变体

        Args:
            variant: 变体文本
        """
        if variant not in self.config.wake_word_variants:
            self.config.wake_word_variants.append(variant)
            self._save_config()
            logger.info(f"✅ 添加唤醒词变体: '{variant}'")

    def remove_wake_word_variant(self, variant: str):
        """
        移除唤醒词变体

        Args:
            variant: 变体文本
        """
        if variant in self.config.wake_word_variants:
            self.config.wake_word_variants.remove(variant)
            self._save_config()
            logger.info(f"🗑️ 移除唤醒词变体: '{variant}'")

    def validate_config(self) -> Dict[str, Any]:
        """
        验证配置

        Returns:
            验证结果字典
        """
        validation_result = {
            'valid': True,
            'errors': [],
            'warnings': []
        }

        # 验证阈值
        if not (0.0 <= self.config.threshold <= 1.0):
            validation_result['errors'].append("检测阈值必须在0.0-1.0之间")
            validation_result['valid'] = False

        # 验证冷却时间
        if self.config.cooldown_period < 0:
            validation_result['errors'].append("冷却时间不能为负数")
            validation_result['valid'] = False

        # 验证音频参数
        if self.config.audio_sample_rate <= 0:
            validation_result['errors'].append("音频采样率必须大于0")
            validation_result['valid'] = False

        if self.config.audio_channels not in [1, 2]:
            validation_result['warnings'].append("音频通道数建议为1或2")

        # 验证音频长度
        if self.config.min_audio_length >= self.config.max_audio_length:
            validation_result['errors'].append("最小音频长度不能大于等于最大音频长度")
            validation_result['valid'] = False

        # 验证唤醒词
        if not self.config.wake_word.strip():
            validation_result['errors'].append("唤醒词不能为空")
            validation_result['valid'] = False

        return validation_result

    def get_config_summary(self) -> Dict[str, Any]:
        """
        获取配置摘要

        Returns:
            配置摘要字典
        """
        return {
            'wake_word': self.config.wake_word,
            'variants_count': len(self.config.wake_word_variants),
            'threshold': self.config.threshold,
            'cooldown_period': self.config.cooldown_period,
            'audio_sample_rate': self.config.audio_sample_rate,
            'audio_channels': self.config.audio_channels,
            'enable_asr': self.config.enable_asr_detection,
            'enable_fallback': self.config.enable_fallback_detection
        }

    def reset_to_default(self):
        """重置为默认配置"""
        self.config = WakeWordConfig()
        self._save_config()
        logger.info("🔄 配置已重置为默认值")

    def export_config(self, export_path: str) -> bool:
        """
        导出配置到文件

        Args:
            export_path: 导出文件路径

        Returns:
            是否成功导出
        """
        try:
            config_dict = asdict(self.config)

            with open(export_path, 'w', encoding='utf-8') as f:
                json.dump(config_dict, f, indent=2, ensure_ascii=False)

            logger.info(f"✅ 配置已导出到: {export_path}")
            return True

        except Exception as e:
            logger.error(f"❌ 配置导出失败: {e}")
            return False

    def import_config(self, import_path: str) -> bool:
        """
        从文件导入配置

        Args:
            import_path: 导入文件路径

        Returns:
            是否成功导入
        """
        try:
            if not os.path.exists(import_path):
                logger.error(f"❌ 配置文件不存在: {import_path}")
                return False

            with open(import_path, 'r', encoding='utf-8') as f:
                config_data = json.load(f)

            # 验证导入的配置
            temp_config = WakeWordConfig()
            for key, value in config_data.items():
                if hasattr(temp_config, key):
                    setattr(temp_config, key, value)

            # 临时验证配置
            original_config = self.config
            self.config = temp_config

            validation = self.validate_config()
            if validation['valid']:
                self._save_config()
                logger.info(f"✅ 配置已从文件导入: {import_path}")
                return True
            else:
                # 验证失败，恢复原配置
                self.config = original_config
                logger.error(f"❌ 导入的配置无效: {validation['errors']}")
                return False

        except Exception as e:
            logger.error(f"❌ 配置导入失败: {e}")
            return False


# 全局实例
_config_manager = None

def get_wake_word_config_manager(config_file: Optional[str] = None) -> WakeWordConfigManager:
    """获取全局唤醒词配置管理器实例"""
    global _config_manager

    if _config_manager is None:
        _config_manager = WakeWordConfigManager(config_file)

    return _config_manager


# 便捷函数
def get_wake_word_config() -> WakeWordConfig:
    """获取唤醒词配置"""
    manager = get_wake_word_config_manager()
    return manager.get_config()


def set_wake_word(wake_word: str, variants: Optional[List[str]] = None):
    """设置唤醒词"""
    manager = get_wake_word_config_manager()
    manager.set_wake_word(wake_word, variants)


def set_detection_threshold(threshold: float):
    """设置检测阈值"""
    manager = get_wake_word_config_manager()
    manager.set_threshold(threshold)


# 测试和验证函数
def test_wake_word_config():
    """测试唤醒词配置管理器"""
    logger.info("🧪 测试唤醒词配置管理器功能")

    try:
        # 创建配置管理器
        manager = WakeWordConfigManager()

        # 获取配置摘要
        summary = manager.get_config_summary()
        logger.info(f"📊 配置摘要: {summary}")

        # 验证配置
        validation = manager.validate_config()
        if validation['valid']:
            logger.info("✅ 配置验证通过")
        else:
            logger.error(f"❌ 配置验证失败: {validation['errors']}")
            if validation['warnings']:
                logger.warning(f"⚠️ 配置警告: {validation['warnings']}")

        # 测试配置更新
        manager.set_threshold(0.8)
        manager.set_cooldown_period(3.0)
        manager.add_wake_word_variant("测试变体")

        # 清理测试变体
        manager.remove_wake_word_variant("测试变体")

        logger.info("🎉 唤醒词配置管理器测试完成")
        return True

    except Exception as e:
        logger.error(f"❌ 唤醒词配置管理器测试失败: {e}")
        return False


if __name__ == "__main__":
    # 运行测试
    test_wake_word_config()