#!/usr/bin/env python3.10
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API

"""
XLeRobot环境变量管理脚本
用于安全管理多环境配置文件
"""

import os
import sys
import shutil
import argparse
from pathlib import Path
from datetime import datetime

class EnvManager:
    """环境变量管理器"""

    def __init__(self, project_root=None):
        self.project_root = Path(project_root) if project_root else Path(__file__).parent.parent
        self.env_files = {
            'template': self.project_root / '.env.example',
            'development': self.project_root / '.env.development',
            'testing': self.project_root / '.env.testing',
            'production': self.project_root / '.env.production',
            'local': self.project_root / '.env.local'
        }

    def backup_env_file(self, env_path):
        """备份环境文件"""
        if not env_path.exists():
            print(f"⚠️  环境文件不存在: {env_path}")
            return False

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = env_path.with_suffix(f'.backup.{timestamp}')

        try:
            shutil.copy2(env_path, backup_path)
            print(f"✅ 环境文件已备份: {backup_path}")
            return True
        except Exception as e:
            print(f"❌ 备份失败: {e}")
            return False

    def create_env_from_template(self, env_type='development'):
        """从模板创建环境配置文件"""
        template_path = self.env_files['template']
        target_path = self.env_files.get(env_type)

        if not template_path.exists():
            print(f"❌ 模板文件不存在: {template_path}")
            return False

        if not target_path:
            print(f"❌ 无效的环境类型: {env_type}")
            return False

        # 备份现有文件
        if target_path.exists():
            self.backup_env_file(target_path)

        try:
            shutil.copy2(template_path, target_path)
            print(f"✅ 已创建{env_type}环境配置: {target_path}")
            print(f"📝 请编辑文件并填入真实的API密钥")
            return True
        except Exception as e:
            print(f"❌ 创建失败: {e}")
            return False

    def validate_env_file(self, env_path):
        """验证环境配置文件"""
        if not env_path.exists():
            print(f"❌ 环境文件不存在: {env_path}")
            return False

        required_keys = [
            'ALIBABA_CLOUD_ACCESS_KEY_ID',
            'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
            'ALIYUN_NLS_APPKEY',
            'QWEN_API_KEY'
        ]

        try:
            with open(env_path, 'r', encoding='utf-8') as f:
                content = f.read()

            missing_keys = []
            placeholder_keys = []

            for key in required_keys:
                if f'{key}=' not in content:
                    missing_keys.append(key)
                elif any(placeholder in content.split(f'{key}=')[1].split('\n')[0]
                        for placeholder in ['your_', 'placeholder', 'xxx', 'here']):
                    placeholder_keys.append(key)

            if missing_keys:
                print(f"❌ 缺少必需的配置项: {', '.join(missing_keys)}")
                return False

            if placeholder_keys:
                print(f"⚠️  以下配置项仍为占位符: {', '.join(placeholder_keys)}")
                return False

            print(f"✅ 环境配置验证通过: {env_path}")
            return True

        except Exception as e:
            print(f"❌ 验证失败: {e}")
            return False

    def load_env(self, env_type='local'):
        """加载指定环境配置"""
        env_path = self.env_files.get(env_type)

        if not env_path or not env_path.exists():
            print(f"❌ 环境文件不存在: {env_path}")
            return False

        try:
            with open(env_path, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#') and '=' in line:
                        key, value = line.split('=', 1)
                        os.environ[key] = value

            print(f"✅ 已加载{env_type}环境配置")
            return True

        except Exception as e:
            print(f"❌ 加载失败: {e}")
            return False

    def list_env_files(self):
        """列出所有环境配置文件"""
        print("📋 环境配置文件状态:")
        print("-" * 50)

        for env_type, path in self.env_files.items():
            status = "✅ 存在" if path.exists() else "❌ 不存在"
            print(f"{env_type:12} : {status:8} - {path}")

        # 检查备份文件
        backup_files = list(self.project_root.glob('.env.backup.*'))
        if backup_files:
            print(f"\n📦 备份文件 ({len(backup_files)}个):")
            for backup in sorted(backup_files)[-5:]:  # 显示最新5个
                print(f"    {backup.name}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='XLeRobot环境变量管理工具')
    parser.add_argument('action', choices=[
        'create', 'validate', 'load', 'list', 'backup'
    ], help='执行的操作')
    parser.add_argument('--env', default='development',
                       help='环境类型 (development/testing/production/local)')
    parser.add_argument('--project-root', help='项目根目录路径')

    args = parser.parse_args()

    manager = EnvManager(args.project_root)

    if args.action == 'list':
        manager.list_env_files()

    elif args.action == 'create':
        manager.create_env_from_template(args.env)

    elif args.action == 'validate':
        env_path = manager.env_files.get(args.env)
        if env_path:
            manager.validate_env_file(env_path)

    elif args.action == 'load':
        manager.load_env(args.env)

    elif args.action == 'backup':
        for env_type, path in manager.env_files.items():
            if path.exists() and env_type != 'template':
                manager.backup_env_file(path)

if __name__ == '__main__':
    main()