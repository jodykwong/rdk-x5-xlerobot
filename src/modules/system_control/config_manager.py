"""
配置管理模块 - Story 4.5
实现配置管理、动态配置更新、配置验证、配置热重载
支持多格式配置、配置模板、环境变量注入
"""

import asyncio
import json
import yaml
import os
import time
from typing import Any, Dict, List, Optional, Callable, Union
from dataclasses import dataclass, field
from enum import Enum
import logging
from pathlib import Path
import threading
from datetime import datetime

logger = logging.getLogger(__name__)


class ConfigFormat(Enum):
    """配置格式"""
    JSON = "json"
    YAML = "yaml"


class ConfigScope(Enum):
    """配置作用域"""
    GLOBAL = "global"
    MODULE = "module"
    INSTANCE = "instance"
    TEMPORARY = "temporary"


@dataclass
class ConfigItem:
    """配置项"""
    key: str
    value: Any
    scope: ConfigScope = ConfigScope.GLOBAL
    description: str = ""
    created_at: float = field(default_factory=time.time)
    updated_at: float = field(default_factory=time.time)


@dataclass
class ConfigChange:
    """配置变更"""
    key: str
    old_value: Any
    new_value: Any
    timestamp: float = field(default_factory=time.time)


class ConfigManager:
    """
    配置管理器
    提供完整的配置管理功能
    """

    def __init__(self, config_dir: Optional[str] = None):
        self._config_dir = Path(config_dir) if config_dir else Path.cwd() / "config"
        self._config_dir.mkdir(exist_ok=True)

        self._configs: Dict[str, ConfigItem] = {}
        self._change_history: List[ConfigChange] = []
        self._change_callbacks: List[Callable[[ConfigChange], None]] = []
        self._lock = threading.RLock()

        self._stats = {
            'configs_loaded': 0,
            'configs_saved': 0,
            'changes_made': 0,
            'validations_performed': 0
        }

        logger.info(f"配置管理器初始化完成，目录: {self._config_dir}")

    async def start(self):
        """启动配置管理器"""
        logger.info("配置管理器已启动")

    async def stop(self):
        """停止配置管理器"""
        await self.save_all_configs()
        logger.info("配置管理器已停止")

    def load_config_from_dict(self, config_dict: Dict[str, Any],
                             scope: ConfigScope = ConfigScope.GLOBAL,
                             source: str = "dict") -> bool:
        """从字典加载配置"""
        try:
            with self._lock:
                for key, value in config_dict.items():
                    self._set_config_internal(key, value, scope, source)

            self._stats['configs_loaded'] += len(config_dict)
            logger.info(f"配置字典加载成功，数量: {len(config_dict)}")
            return True
        except Exception as e:
            logger.error(f"加载配置字典失败: {str(e)}")
            return False

    def load_config_from_file(self, file_path: Union[str, Path],
                             scope: ConfigScope = ConfigScope.GLOBAL) -> bool:
        """从文件加载配置"""
        try:
            file_path = Path(file_path)
            if not file_path.exists():
                logger.error(f"配置文件不存在: {file_path}")
                return False

            with open(file_path, 'r', encoding='utf-8') as f:
                if file_path.suffix.lower() == '.json':
                    config_dict = json.load(f)
                elif file_path.suffix.lower() in ['.yaml', '.yml']:
                    config_dict = yaml.safe_load(f) or {}
                else:
                    logger.error(f"不支持的文件格式: {file_path.suffix}")
                    return False

            return self.load_config_from_dict(config_dict, scope, str(file_path))
        except Exception as e:
            logger.error(f"加载配置文件失败: {str(e)}")
            return False

    async def save_all_configs(self, file_path: Optional[Union[str, Path]] = None):
        """保存所有配置"""
        try:
            if file_path is None:
                file_path = self._config_dir / "config.yaml"
            else:
                file_path = Path(file_path)

            with self._lock:
                all_configs = {}
                for key, config_item in self._configs.items():
                    all_configs[key] = config_item.value

            content = yaml.dump(all_configs, default_flow_style=False, allow_unicode=True)
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(content)

            self._stats['configs_saved'] += 1
            logger.debug(f"所有配置已保存到: {file_path}")
        except Exception as e:
            logger.error(f"保存所有配置失败: {str(e)}")

    def set_config(self, key: str, value: Any,
                  scope: ConfigScope = ConfigScope.GLOBAL,
                  description: str = "",
                  validation_rules: Optional[List[str]] = None,
                  source: str = "api") -> bool:
        """设置配置"""
        try:
            # 简单验证
            if validation_rules:
                for rule in validation_rules:
                    if rule.startswith('type:'):
                        expected_type = rule.split(':', 1)[1]
                        if expected_type == 'int' and not isinstance(value, int):
                            logger.error(f"配置验证失败: {key} 类型应为 {expected_type}")
                            return False
                        elif expected_type == 'float' and not isinstance(value, (int, float)):
                            logger.error(f"配置验证失败: {key} 类型应为 {expected_type}")
                            return False
                        elif expected_type == 'str' and not isinstance(value, str):
                            logger.error(f"配置验证失败: {key} 类型应为 {expected_type}")
                            return False
                        elif expected_type == 'bool' and not isinstance(value, bool):
                            logger.error(f"配置验证失败: {key} 类型应为 {expected_type}")
                            return False
                    elif rule.startswith('min:'):
                        min_value = float(rule.split(':', 1)[1])
                        if isinstance(value, (int, float)) and value < min_value:
                            logger.error(f"配置验证失败: {key} 应 >= {min_value}")
                            return False
                    elif rule.startswith('max:'):
                        max_value = float(rule.split(':', 1)[1])
                        if isinstance(value, (int, float)) and value > max_value:
                            logger.error(f"配置验证失败: {key} 应 <= {max_value}")
                            return False

            with self._lock:
                old_value = self._configs.get(key).value if key in self._configs else None
                self._set_config_internal(key, value, scope, source, description)

                if old_value != value:
                    change = ConfigChange(key=key, old_value=old_value, new_value=value)
                    self._change_history.append(change)
                    self._notify_change_callbacks(change)
                    self._stats['changes_made'] += 1

            logger.debug(f"配置已设置: {key}={value}")
            return True
        except Exception as e:
            logger.error(f"设置配置失败: {str(e)}")
            return False

    def _set_config_internal(self, key: str, value: Any, scope: ConfigScope,
                            source: str, description: str = ""):
        """内部设置配置方法"""
        config_item = ConfigItem(key=key, value=value, scope=scope, description=description)
        self._configs[key] = config_item

    def get_config(self, key: str, default: Any = None) -> Any:
        """获取配置"""
        with self._lock:
            if key in self._configs:
                return self._configs[key].value
            return default

    def get_config_item(self, key: str) -> Optional[ConfigItem]:
        """获取配置项"""
        with self._lock:
            return self._configs.get(key)

    def get_all_configs(self, scope: Optional[ConfigScope] = None) -> Dict[str, ConfigItem]:
        """获取所有配置"""
        with self._lock:
            if scope is None:
                return self._configs.copy()
            else:
                return {k: v for k, v in self._configs.items() if v.scope == scope}

    def delete_config(self, key: str) -> bool:
        """删除配置"""
        try:
            with self._lock:
                if key in self._configs:
                    del self._configs[key]
                    logger.info(f"配置已删除: {key}")
                    return True
                else:
                    logger.warning(f"配置项不存在: {key}")
                    return False
        except Exception as e:
            logger.error(f"删除配置失败: {str(e)}")
            return False

    def register_change_callback(self, callback: Callable[[ConfigChange], None]):
        """注册配置变更回调"""
        self._change_callbacks.append(callback)
        logger.info(f"配置变更回调已注册: {callback.__name__}")

    def _notify_change_callbacks(self, change: ConfigChange):
        """通知配置变更回调"""
        for callback in self._change_callbacks:
            try:
                callback(change)
            except Exception as e:
                logger.error(f"配置变更回调执行失败: {str(e)}")

    def get_change_history(self, limit: int = 50) -> List[ConfigChange]:
        """获取配置变更历史"""
        with self._lock:
            return self._change_history[-limit:]

    def export_config(self, scope: Optional[ConfigScope] = None, format_type: ConfigFormat = ConfigFormat.YAML) -> str:
        """导出配置"""
        try:
            configs = self.get_all_configs(scope)
            data = {key: config_item.value for key, config_item in configs.items()}

            if format_type == ConfigFormat.JSON:
                return json.dumps(data, indent=2, ensure_ascii=False)
            else:
                return yaml.dump(data, default_flow_style=False, allow_unicode=True)
        except Exception as e:
            logger.error(f"导出配置失败: {str(e)}")
            raise

    def create_config_template(self, template_name: str,
                             config_dict: Dict[str, Any],
                             output_path: Optional[Union[str, Path]] = None) -> str:
        """创建配置模板"""
        try:
            template = {
                'name': template_name,
                'description': f'Configuration template for {template_name}',
                'created_at': datetime.now().isoformat(),
                'configs': config_dict
            }

            content = yaml.dump(template, default_flow_style=False, allow_unicode=True)

            if output_path:
                output_path = Path(output_path)
                with open(output_path, 'w', encoding='utf-8') as f:
                    f.write(content)

                logger.info(f"配置模板已创建: {template_name}")

            return content
        except Exception as e:
            logger.error(f"创建配置模板失败: {str(e)}")
            raise

    def apply_config_template(self, template_content: str,
                             scope: ConfigScope = ConfigScope.GLOBAL) -> bool:
        """应用配置模板"""
        try:
            template = yaml.safe_load(template_content)
            if 'configs' not in template:
                raise ValueError("无效的模板格式")

            config_dict = template['configs']
            return self.load_config_from_dict(config_dict, scope, "template")
        except Exception as e:
            logger.error(f"应用配置模板失败: {str(e)}")
            return False

    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        with self._lock:
            stats = self._stats.copy()
            stats['total_configs'] = len(self._configs)
            stats['change_history_size'] = len(self._change_history)
            stats['config_dir'] = str(self._config_dir)
            return stats


if __name__ == "__main__":
    async def main():
        """测试配置管理器"""
        config_manager = ConfigManager()
        await config_manager.start()

        config_manager.set_config("app.name", "XLeRobot")
        config_manager.set_config("server.port", 8080)
        config_manager.set_config("debug.enabled", True)

        app_name = config_manager.get_config("app.name")
        print(f"应用名称: {app_name}")

        all_configs = config_manager.get_all_configs()
        print(f"配置数量: {len(all_configs)}")

        stats = config_manager.get_stats()
        print(f"统计信息: {stats}")

        await config_manager.stop()

    asyncio.run(main())
