#!/usr/bin/env python3.10
"""
XleRobot 配置管理器
BMad-Method v6 Brownfield Level 4 标准

功能特性:
- 环境分离配置管理
- 配置继承和覆盖
- 敏感信息加密管理
- 配置版本控制
- 配置验证和校验
- 动态配置更新
"""

import os
import yaml
import json
import logging
import hashlib
import asyncio
from typing import Dict, Any, Optional, List, Union
from pathlib import Path
from cryptography.fernet import Fernet
from dataclasses import dataclass, field
from enum import Enum

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class Environment(Enum):
    """环境枚举"""
    DEVELOPMENT = "development"
    TESTING = "testing"
    STAGING = "staging"
    PRODUCTION = "production"

class ConfigType(Enum):
    """配置类型枚举"""
    BASE = "base"
    ENVIRONMENT = "environment"
    SERVICE = "service"
    RUNTIME = "runtime"
    SECRETS = "secrets"

@dataclass
class ConfigMetadata:
    """配置元数据"""
    version: str
    environment: Environment
    last_modified: float
    checksum: str
    encrypted_fields: List[str] = field(default_factory=list)
    validation_rules: Dict[str, Any] = field(default_factory=dict)

class ConfigValidationError(Exception):
    """配置验证错误"""
    pass

class ConfigEncryptionError(Exception):
    """配置加密错误"""
    pass

class XleRobotConfigManager:
    """XleRobot配置管理器"""

    def __init__(self, config_root: str = "/home/sunrise/xlerobot/deployment/config"):
        """
        初始化配置管理器

        Args:
            config_root: 配置根目录
        """
        self.config_root = Path(config_root)
        self.base_config_dir = self.config_root / "base"
        self.environments_dir = self.config_root / "environments"
        self.services_dir = self.config_root / "services"
        self.runtime_dir = self.config_root / "runtime"
        self.secrets_dir = self.config_root / "secrets"

        # 加密密钥
        self.encryption_key = self._load_encryption_key()
        self.cipher = Fernet(self.encryption_key)

        # 配置缓存
        self._config_cache: Dict[str, Any] = {}
        self._metadata_cache: Dict[str, ConfigMetadata] = {}

        # 初始化目录结构
        self._ensure_directory_structure()

        logger.info("✅ XleRobot配置管理器初始化完成")

    def _ensure_directory_structure(self):
        """确保配置目录结构存在"""
        directories = [
            self.base_config_dir,
            self.environments_dir,
            self.services_dir,
            self.runtime_dir,
            self.secrets_dir,
            self.secrets_dir / "encrypted",
            self.secrets_dir / "environments",
            self.secrets_dir / "certificates"
        ]

        for directory in directories:
            directory.mkdir(parents=True, exist_ok=True)

    def _load_encryption_key(self) -> bytes:
        """加载加密密钥"""
        key_file = self.secrets_dir / ".encryption_key"

        if key_file.exists():
            with open(key_file, 'rb') as f:
                return f.read()
        else:
            # 生成新密钥
            key = Fernet.generate_key()
            with open(key_file, 'wb') as f:
                f.write(key)
            # 设置文件权限
            os.chmod(key_file, 0o600)
            return key

    def _encrypt_value(self, value: str) -> str:
        """加密敏感值"""
        try:
            encrypted_value = self.cipher.encrypt(value.encode())
            return encrypted_value.decode()
        except Exception as e:
            raise ConfigEncryptionError(f"加密失败: {str(e)}")

    def _decrypt_value(self, encrypted_value: str) -> str:
        """解密敏感值"""
        try:
            decrypted_value = self.cipher.decrypt(encrypted_value.encode())
            return decrypted_value.decode()
        except Exception as e:
            raise ConfigEncryptionError(f"解密失败: {str(e)}")

    def _calculate_checksum(self, data: Dict[str, Any]) -> str:
        """计算配置数据校验和"""
        data_str = json.dumps(data, sort_keys=True, ensure_ascii=False)
        return hashlib.sha256(data_str.encode()).hexdigest()

    def _load_yaml_file(self, file_path: Path) -> Dict[str, Any]:
        """加载YAML文件"""
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            logger.warning(f"配置文件不存在: {file_path}")
            return {}
        except yaml.YAMLError as e:
            raise ConfigValidationError(f"YAML文件格式错误 {file_path}: {str(e)}")

    def _save_yaml_file(self, file_path: Path, data: Dict[str, Any]) -> None:
        """保存YAML文件"""
        try:
            with open(file_path, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, default_flow_style=False, allow_unicode=True, indent=2)
        except Exception as e:
            raise ConfigValidationError(f"保存配置文件失败 {file_path}: {str(e)}")

    def _merge_configs(self, base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
        """深度合并配置"""
        result = base.copy()

        for key, value in override.items():
            if key in result and isinstance(result[key], dict) and isinstance(value, dict):
                result[key] = self._merge_configs(result[key], value)
            else:
                result[key] = value

        return result

    def _validate_config(self, config: Dict[str, Any], rules: Dict[str, Any]) -> None:
        """验证配置"""
        for field_path, rule in rules.items():
            field_value = self._get_nested_value(config, field_path)

            # 检查必需字段
            if rule.get('required', False) and field_value is None:
                raise ConfigValidationError(f"必需字段缺失: {field_path}")

            # 检查字段类型
            if 'type' in rule and field_value is not None:
                expected_type = rule['type']
                if not self._check_type(field_value, expected_type):
                    raise ConfigValidationError(f"字段类型错误 {field_path}: 期望 {expected_type}")

            # 检查值范围
            if 'constraints' in rule and field_value is not None:
                self._check_constraints(field_path, field_value, rule['constraints'])

    def _get_nested_value(self, data: Dict[str, Any], path: str) -> Any:
        """获取嵌套字典值"""
        keys = path.split('.')
        current = data

        for key in keys:
            if isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None

        return current

    def _set_nested_value(self, data: Dict[str, Any], path: str, value: Any) -> None:
        """设置嵌套字典值"""
        keys = path.split('.')
        current = data

        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]

        current[keys[-1]] = value

    def _check_type(self, value: Any, expected_type: str) -> bool:
        """检查值类型"""
        type_mapping = {
            'string': str,
            'integer': int,
            'float': float,
            'boolean': bool,
            'list': list,
            'dict': dict
        }

        expected_python_type = type_mapping.get(expected_type)
        if expected_python_type:
            return isinstance(value, expected_python_type)

        return True

    def _check_constraints(self, field_path: str, value: Any, constraints: Dict[str, Any]) -> None:
        """检查值约束"""
        if 'min' in constraints and value < constraints['min']:
            raise ConfigValidationError(f"值过小 {field_path}: {value} < {constraints['min']}")

        if 'max' in constraints and value > constraints['max']:
            raise ConfigValidationError(f"值过大 {field_path}: {value} > {constraints['max']}")

        if 'enum' in constraints and value not in constraints['enum']:
            raise ConfigValidationError(f"无效枚举值 {field_path}: {value} not in {constraints['enum']}")

        if 'pattern' in constraints:
            import re
            if not re.match(constraints['pattern'], str(value)):
                raise ConfigValidationError(f"模式匹配失败 {field_path}: {value}")

    async def load_config(self, environment: Environment, service: Optional[str] = None) -> Dict[str, Any]:
        """
        加载完整配置

        Args:
            environment: 目标环境
            service: 可选的服务名称

        Returns:
            合并后的完整配置
        """
        cache_key = f"{environment.value}_{service or 'all'}"

        # 检查缓存
        if cache_key in self._config_cache:
            logger.debug(f"从缓存加载配置: {cache_key}")
            return self._config_cache[cache_key]

        logger.info(f"加载配置 - 环境: {environment.value}, 服务: {service or 'all'}")

        # 1. 加载基础配置
        config = {}
        base_files = list(self.base_config_dir.glob("*.yaml"))
        for base_file in base_files:
            base_config = self._load_yaml_file(base_file)
            config = self._merge_configs(config, base_config)

        # 2. 加载环境配置
        env_file = self.environments_dir / f"{environment.value}.yaml"
        if env_file.exists():
            env_config = self._load_yaml_file(env_file)
            config = self._merge_configs(config, env_config)

        # 3. 加载服务配置
        if service:
            service_file = self.services_dir / f"{service}.yaml"
            if service_file.exists():
                service_config = self._load_yaml_file(service_file)
                config = self._merge_configs(config, service_config)

        # 4. 加载运行时配置
        runtime_files = list(self.runtime_dir.glob("*.yaml"))
        for runtime_file in runtime_files:
            runtime_config = self._load_yaml_file(runtime_file)
            config = self._merge_configs(config, runtime_config)

        # 5. 加载敏感信息
        config = await self._load_secrets(environment, config)

        # 6. 验证配置
        await self._validate_full_config(config, environment)

        # 缓存配置
        self._config_cache[cache_key] = config

        # 创建元数据
        metadata = ConfigMetadata(
            version=config.get('system', {}).get('version', '1.0.0'),
            environment=environment,
            last_modified=asyncio.get_event_loop().time(),
            checksum=self._calculate_checksum(config)
        )
        self._metadata_cache[cache_key] = metadata

        logger.info(f"✅ 配置加载完成: {cache_key}")
        return config

    async def _load_secrets(self, environment: Environment, config: Dict[str, Any]) -> Dict[str, Any]:
        """加载敏感信息"""
        secrets_file = self.secrets_dir / "environments" / f"{environment.value}.yaml"

        if not secrets_file.exists():
            logger.warning(f"敏感信息文件不存在: {secrets_file}")
            return config

        try:
            secrets_config = self._load_yaml_file(secrets_file)

            # 解密敏感字段
            for key, value in secrets_config.items():
                if isinstance(value, str) and value.startswith('encrypted:'):
                    encrypted_value = value[10:]  # 移除 'encrypted:' 前缀
                    decrypted_value = self._decrypt_value(encrypted_value)
                    self._set_nested_value(config, key, decrypted_value)
                else:
                    self._set_nested_value(config, key, value)

        except Exception as e:
            logger.error(f"加载敏感信息失败: {str(e)}")

        return config

    async def _validate_full_config(self, config: Dict[str, Any], environment: Environment) -> None:
        """验证完整配置"""
        # 基础验证规则
        base_rules = {
            'system.version': {'required': True, 'type': 'string'},
            'system.name': {'required': True, 'type': 'string'},
            'environment.name': {'required': True, 'type': 'string'},
            'services.*.port': {'required': True, 'type': 'integer', 'constraints': {'min': 1000, 'max': 65535}},
            'services.*.replicas': {'required': True, 'type': 'integer', 'constraints': {'min': 1, 'max': 10}},
            'monitoring.enabled': {'required': True, 'type': 'boolean'}
        }

        # 环境特定验证规则
        env_specific_rules = {}

        if environment == Environment.PRODUCTION:
            env_specific_rules.update({
                'services.*.replicas': {'constraints': {'min': 2}},
                'security.tls.enabled': {'required': True, 'type': 'boolean', 'constraints': {'enum': [True]}},
                'backup.enabled': {'required': True, 'type': 'boolean', 'constraints': {'enum': [True]}}
            })

        # 合并验证规则
        all_rules = {**base_rules, **env_specific_rules}

        # 执行验证
        self._validate_config(config, all_rules)

        logger.info("✅ 配置验证通过")

    async def save_secret(self, environment: Environment, key: str, value: str) -> None:
        """
        保存敏感信息

        Args:
            environment: 目标环境
            key: 敏感信息键
            value: 敏感信息值
        """
        secrets_file = self.secrets_dir / "environments" / f"{environment.value}.yaml"

        # 加载现有敏感信息
        secrets_config = {}
        if secrets_file.exists():
            secrets_config = self._load_yaml_file(secrets_file)

        # 加密新值
        encrypted_value = self._encrypt_value(value)
        secrets_config[key] = f"encrypted:{encrypted_value}"

        # 保存到文件
        self._save_yaml_file(secrets_file, secrets_config)

        # 设置文件权限
        os.chmod(secrets_file, 0o600)

        logger.info(f"✅ 敏感信息已保存: {key}")

    async def update_config(self, environment: Environment, updates: Dict[str, Any], service: Optional[str] = None) -> None:
        """
        更新配置

        Args:
            environment: 目标环境
            updates: 更新的配置项
            service: 可选的服务名称
        """
        logger.info(f"更新配置 - 环境: {environment.value}, 服务: {service}")

        # 加载当前配置
        current_config = await self.load_config(environment, service)

        # 应用更新
        for key, value in updates.items():
            self._set_nested_value(current_config, key, value)

        # 验证更新后的配置
        await self._validate_full_config(current_config, environment)

        # 确定保存文件路径
        if service:
            config_file = self.services_dir / f"{service}.yaml"
        else:
            config_file = self.environments_dir / f"{environment.value}.yaml"

        # 保存配置
        self._save_yaml_file(config_file, current_config)

        # 清除缓存
        cache_key = f"{environment.value}_{service or 'all'}"
        self._config_cache.pop(cache_key, None)
        self._metadata_cache.pop(cache_key, None)

        logger.info(f"✅ 配置更新完成")

    def get_config_metadata(self, environment: Environment, service: Optional[str] = None) -> Optional[ConfigMetadata]:
        """获取配置元数据"""
        cache_key = f"{environment.value}_{service or 'all'}"
        return self._metadata_cache.get(cache_key)

    async def rollback_config(self, environment: Environment, target_version: str, service: Optional[str] = None) -> bool:
        """
        回滚配置到指定版本

        Args:
            environment: 目标环境
            target_version: 目标版本
            service: 可选的服务名称

        Returns:
            回滚是否成功
        """
        logger.info(f"回滚配置 - 环境: {environment.value}, 版本: {target_version}, 服务: {service}")

        try:
            # 这里应该实现版本控制和回滚逻辑
            # 简化实现：从备份目录恢复配置
            backup_dir = self.config_root / "backups" / environment.value / target_version

            if not backup_dir.exists():
                logger.error(f"备份版本不存在: {backup_dir}")
                return False

            # 恢复配置文件
            if service:
                backup_file = backup_dir / f"{service}.yaml"
                target_file = self.services_dir / f"{service}.yaml"
            else:
                backup_file = backup_dir / f"{environment.value}.yaml"
                target_file = self.environments_dir / f"{environment.value}.yaml"

            if backup_file.exists():
                import shutil
                shutil.copy2(backup_file, target_file)

                # 清除缓存
                cache_key = f"{environment.value}_{service or 'all'}"
                self._config_cache.pop(cache_key, None)
                self._metadata_cache.pop(cache_key, None)

                logger.info("✅ 配置回滚完成")
                return True
            else:
                logger.error(f"备份文件不存在: {backup_file}")
                return False

        except Exception as e:
            logger.error(f"配置回滚失败: {str(e)}")
            return False

    async def create_backup(self, environment: Environment, service: Optional[str] = None) -> str:
        """
        创建配置备份

        Args:
            environment: 目标环境
            service: 可选的服务名称

        Returns:
            备份版本号
        """
        import datetime
        import shutil

        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_version = f"v_{timestamp}"

        backup_dir = self.config_root / "backups" / environment.value / backup_version
        backup_dir.mkdir(parents=True, exist_ok=True)

        logger.info(f"创建配置备份 - 环境: {environment.value}, 版本: {backup_version}")

        try:
            # 备份环境配置
            env_file = self.environments_dir / f"{environment.value}.yaml"
            if env_file.exists():
                shutil.copy2(env_file, backup_dir / f"{environment.value}.yaml")

            # 备份服务配置
            if service:
                service_file = self.services_dir / f"{service}.yaml"
                if service_file.exists():
                    shutil.copy2(service_file, backup_dir / f"{service}.yaml")

            logger.info(f"✅ 配置备份完成: {backup_version}")
            return backup_version

        except Exception as e:
            logger.error(f"创建配置备份失败: {str(e)}")
            raise

    async def reload_config(self, environment: Environment, service: Optional[str] = None) -> None:
        """重新加载配置"""
        cache_key = f"{environment.value}_{service or 'all'}"

        # 清除缓存
        self._config_cache.pop(cache_key, None)
        self._metadata_cache.pop(cache_key, None)

        # 重新加载配置
        await self.load_config(environment, service)

        logger.info(f"✅ 配置重新加载完成: {cache_key}")

    def list_environments(self) -> List[Environment]:
        """列出所有可用环境"""
        environments = []
        for env_file in self.environments_dir.glob("*.yaml"):
            env_name = env_file.stem
            try:
                environments.append(Environment(env_name))
            except ValueError:
                logger.warning(f"未知环境: {env_name}")

        return environments

    def list_services(self) -> List[str]:
        """列出所有可用服务"""
        services = []
        for service_file in self.services_dir.glob("*.yaml"):
            services.append(service_file.stem)

        return services

# 配置管理器单例
_config_manager_instance: Optional[XleRobotConfigManager] = None

def get_config_manager(config_root: Optional[str] = None) -> XleRobotConfigManager:
    """获取配置管理器实例"""
    global _config_manager_instance

    if _config_manager_instance is None:
        _config_manager_instance = XleRobotConfigManager(config_root)

    return _config_manager_instance

# 便捷函数
async def load_config(environment: Union[str, Environment], service: Optional[str] = None) -> Dict[str, Any]:
    """便捷函数：加载配置"""
    if isinstance(environment, str):
        environment = Environment(environment)

    config_manager = get_config_manager()
    return await config_manager.load_config(environment, service)

async def save_secret(environment: Union[str, Environment], key: str, value: str) -> None:
    """便捷函数：保存敏感信息"""
    if isinstance(environment, str):
        environment = Environment(environment)

    config_manager = get_config_manager()
    await config_manager.save_secret(environment, key, value)

if __name__ == "__main__":
    async def main():
        """测试配置管理器"""
        config_manager = XleRobotConfigManager()

        # 测试加载配置
        config = await config_manager.load_config(Environment.DEVELOPMENT)
        print("开发环境配置加载成功")

        # 测试保存敏感信息
        await config_manager.save_secret(Environment.DEVELOPMENT, "test_secret", "test_value")
        print("敏感信息保存成功")

        # 测试配置验证
        config = await config_manager.load_config(Environment.PRODUCTION)
        print("生产环境配置验证成功")

    asyncio.run(main())