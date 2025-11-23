#!/usr/bin/env python3
"""
ASR模块配置管理

管理ASR模块的配置参数和模型路径

作者: Dev Agent
日期: 2025-11-02
Epic: 1 - ASR语音识别模块
Story: 1.1 - 粤语语音识别基础功能
"""

import os
import json
import yaml
from typing import Dict, Any, Optional


class ASRConfig:
    """ASR配置管理类"""
    
    DEFAULT_CONFIG = {
        "model": {
            "name": "iic/SenseVoiceSmall",
            "path": None,  # 模型文件路径
            "use_npu": True,
            "framework": "onnx"  # onnx 或 pytorch
        },
        "audio": {
            "sample_rate": 16000,
            "channels": 1,
            "bit_depth": 16,
            "format": "PCM"
        },
        "language": {
            "primary": "cantonese",
            "supported": ["cantonese", "mandarin", "english"]
        },
        "performance": {
            "confidence_threshold": 0.5,
            "max_processing_time_ms": 300,
            "enable_vad": True
        },
        "paths": {
            "model_dir": "/home/sunrise/xlerobot/models/asr",
            "data_dir": "/home/sunrise/xlerobot/data/asr",
            "log_dir": "/home/sunrise/xlerobot/logs"
        }
    }
    
    def __init__(self, config_path: Optional[str] = None):
        """
        初始化配置
        
        Args:
            config_path: 配置文件路径
        """
        self.config_path = config_path
        self.config = self.DEFAULT_CONFIG.copy()
        
        if config_path and os.path.exists(config_path):
            self.load_from_file(config_path)
            
    def load_from_file(self, config_path: str) -> None:
        """
        从文件加载配置
        
        Args:
            config_path: 配置文件路径
        """
        with open(config_path, 'r', encoding='utf-8') as f:
            if config_path.endswith('.json'):
                user_config = json.load(f)
            elif config_path.endswith(('.yaml', '.yml')):
                user_config = yaml.safe_load(f)
            else:
                raise ValueError(f"不支持的配置文件格式: {config_path}")
                
        self._merge_config(user_config)
        
    def _merge_config(self, user_config: Dict[str, Any]) -> None:
        """合并用户配置"""
        def deep_merge(base: dict, update: dict) -> dict:
            for key, value in update.items():
                if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                    deep_merge(base[key], value)
                else:
                    base[key] = value
            return base
            
        self.config = deep_merge(self.config, user_config)
        
    def get(self, key: str, default: Any = None) -> Any:
        """
        获取配置值
        
        Args:
            key: 配置键（支持点号分隔的嵌套键）
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
            key: 配置键（支持点号分隔的嵌套键）
            value: 配置值
        """
        keys = key.split('.')
        config = self.config
        
        for k in keys[:-1]:
            if k not in config:
                config[k] = {}
            config = config[k]
            
        config[keys[-1]] = value
        
    def save_to_file(self, config_path: str) -> None:
        """
        保存配置到文件
        
        Args:
            config_path: 配置文件路径
        """
        with open(config_path, 'w', encoding='utf-8') as f:
            if config_path.endswith('.json'):
                json.dump(self.config, f, indent=2, ensure_ascii=False)
            elif config_path.endswith(('.yaml', '.yml')):
                yaml.dump(self.config, f, indent=2, allow_unicode=True)
            else:
                raise ValueError(f"不支持的配置文件格式: {config_path}")
                
    def to_dict(self) -> Dict[str, Any]:
        """获取配置字典"""
        return self.config.copy()
