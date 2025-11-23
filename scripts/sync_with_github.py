#!/usr/bin/env python3.10
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API

"""
XLeRobot GitHub同步脚本
用于安全地与远程仓库同步，保护本地敏感信息
"""

import os
import sys
import subprocess
import argparse
from pathlib import Path
from datetime import datetime

class GitHubSyncManager:
    """GitHub同步管理器"""

    def __init__(self, project_root=None):
        self.project_root = Path(project_root) if project_root else Path(__file__).parent.parent
        self.git_dir = self.project_root / '.git'

        # 检查是否在Git仓库中
        if not self.git_dir.exists():
            raise RuntimeError(f"❌ 当前目录不是Git仓库: {self.project_root}")

    def run_git_command(self, command, check=True):
        """执行Git命令"""
        try:
            result = subprocess.run(
                ['git'] + command,
                cwd=self.project_root,
                capture_output=True,
                text=True,
                check=check
            )
            return result.stdout.strip(), result.stderr.strip()
        except subprocess.CalledProcessError as e:
            if check:
                raise RuntimeError(f"Git命令执行失败: {e.stderr}")
            return e.stdout.strip(), e.stderr.strip()

    def check_security_status(self):
        """检查安全状态"""
        print("🔒 检查安全状态...")

        # 检查敏感文件是否被正确忽略
        sensitive_files = ['.env', '.env.local', '*.key', '*.secret']
        ignored_files = []

        with open(self.project_root / '.gitignore', 'r', encoding='utf-8') as f:
            gitignore_content = f.read()

        for pattern in sensitive_files:
            if pattern in gitignore_content:
                ignored_files.append(pattern)

        if len(ignored_files) == len(sensitive_files):
            print("✅ 所有敏感文件类型都已添加到.gitignore")
        else:
            print(f"⚠️  部分敏感文件类型可能未被忽略: {set(sensitive_files) - set(ignored_files)}")

        # 检查是否有敏感文件被意外添加
        stdout, _ = self.run_git_command(['status', '--porcelain'])
        for line in stdout.split('\n'):
            if line and any(sensitive in line for sensitive in ['.env', 'key', 'secret']):
                print(f"⚠️  发现可能的敏感文件: {line}")

    def backup_before_sync(self):
        """同步前备份"""
        print("💾 创建同步前备份...")

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_branch = f"backup_before_sync_{timestamp}"

        # 备份当前分支状态
        try:
            # 保存未提交的修改
            stdout, _ = self.run_git_command(['stash', 'push', '-m', f"自动备份_{timestamp}"])
            if "Saved working directory" in stdout:
                print(f"✅ 未提交修改已保存到stash")

            # 创建备份分支（如果有提交历史）
            stdout, _ = self.run_git_command(['rev-parse', '--verify', 'HEAD'], check=False)
            if stdout:
                self.run_git_command(['branch', backup_branch])
                print(f"✅ 创建备份分支: {backup_branch}")

        except Exception as e:
            print(f"⚠️  备份过程遇到问题: {e}")

    def fetch_remote_changes(self):
        """获取远程更改"""
        print("📥 获取远程仓库更改...")

        try:
            stdout, stderr = self.run_git_command(['fetch', 'origin'])
            print("✅ 远程更改获取完成")
            return True
        except Exception as e:
            print(f"❌ 获取远程更改失败: {e}")
            return False

    def show_changes_summary(self):
        """显示更改摘要"""
        print("\n📊 远程更改摘要:")

        try:
            # 检查是否有新提交
            stdout, _ = self.run_git_command(['log', '--oneline', 'HEAD..origin/main'])
            if stdout:
                print("远程新增提交:")
                for line in stdout.split('\n')[:10]:  # 显示前10个
                    print(f"  {line}")
            else:
                print("✅ 没有新的远程提交")

            # 检查是否有分支差异
            stdout, _ = self.run_git_command(['branch', '-a'])
            print(f"\n当前分支状态:\n{stdout}")

        except Exception as e:
            print(f"⚠️  无法获取更改摘要: {e}")

    def sync_strategy_interactive(self):
        """交互式同步策略选择"""
        print("\n🎯 选择同步策略:")
        print("1. 安全合并 (merge) - 保留本地更改，合并远程更改")
        print("2. 重置到远程 (reset) - 丢弃本地更改，使用远程版本")
        print("3. 仅查看差异 (diff) - 查看但不执行同步")
        print("4. 取消同步 (cancel)")

        choice = input("\n请选择 (1-4): ").strip()

        return {
            '1': 'merge',
            '2': 'reset',
            '3': 'diff',
            '4': 'cancel'
        }.get(choice, 'cancel')

    def execute_sync(self, strategy):
        """执行同步策略"""
        if strategy == 'cancel':
            print("❌ 同步已取消")
            return False

        if strategy == 'diff':
            self.show_diff()
            return True

        if strategy == 'merge':
            return self.merge_changes()

        if strategy == 'reset':
            return self.reset_to_remote()

        return False

    def show_diff(self):
        """显示差异"""
        print("\n📋 本地与远程差异:")

        try:
            stdout, _ = self.run_git_command(['diff', '--stat', 'HEAD...origin/main'])
            if stdout:
                print(f"文件更改统计:\n{stdout}")
            else:
                print("没有文件差异")

            # 显示详细差异（前50行）
            stdout, _ = self.run_git_command(['diff', 'HEAD...origin/main'])
            if stdout:
                lines = stdout.split('\n')
                print(f"\n详细差异 (前50行):")
                for line in lines[:50]:
                    print(line)
                if len(lines) > 50:
                    print(f"... (还有 {len(lines) - 50} 行)")

        except Exception as e:
            print(f"⚠️  无法显示差异: {e}")

    def merge_changes(self):
        """合并更改"""
        print("\n🔀 执行安全合并...")

        try:
            # 确保在main分支
            self.run_git_command(['checkout', 'main'])

            # 拉取并合并
            stdout, _ = self.run_git_command(['pull', 'origin', 'main'])
            print("✅ 合并完成")
            return True

        except Exception as e:
            print(f"❌ 合并失败: {e}")
            print("💡 建议手动解决冲突或使用重置策略")
            return False

    def reset_to_remote(self):
        """重置到远程版本"""
        print("\n🔄 重置到远程版本...")

        try:
            # 确保在main分支
            self.run_git_command(['checkout', 'main'])

            # 硬重置到远程
            self.run_git_command(['reset', '--hard', 'origin/main'])
            print("✅ 已重置到远程版本")

            # 恢复本地环境文件
            local_env = self.project_root / '.env.local'
            if local_env.exists():
                print("✅ 本地环境配置文件已保留")

            return True

        except Exception as e:
            print(f"❌ 重置失败: {e}")
            return False

    def post_sync_check(self):
        """同步后检查"""
        print("\n🔍 同步后检查:")

        try:
            # 检查环境文件状态
            env_files = ['.env', '.env.local', '.env.example']
            for env_file in env_files:
                env_path = self.project_root / env_file
                if env_path.exists():
                    print(f"✅ {env_file} 存在")
                else:
                    print(f"❌ {env_file} 不存在")

            # 检查Git状态
            stdout, _ = self.run_git_command(['status', '--porcelain'])
            if stdout:
                print(f"⚠️  存在未提交的修改:\n{stdout}")
            else:
                print("✅ 工作目录干净")

            # 检查远程连接
            stdout, _ = self.run_git_command(['remote', '-v'])
            print(f"✅ 远程仓库配置:\n{stdout}")

        except Exception as e:
            print(f"⚠️  同步后检查失败: {e}")

def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='XLeRobot GitHub同步工具')
    parser.add_argument('--project-root', help='项目根目录路径')
    parser.add_argument('--strategy', choices=['merge', 'reset', 'diff'],
                       help='同步策略 (不指定则交互选择)')

    args = parser.parse_args()

    try:
        sync_manager = GitHubSyncManager(args.project_root)

        print(f"🚀 XLeRobot GitHub同步工具")
        print(f"📁 项目路径: {sync_manager.project_root}")
        print(f"⏰ 同步时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print("-" * 50)

        # 1. 安全状态检查
        sync_manager.check_security_status()

        # 2. 同步前备份
        sync_manager.backup_before_sync()

        # 3. 获取远程更改
        if not sync_manager.fetch_remote_changes():
            print("❌ 无法获取远程更改，同步终止")
            return 1

        # 4. 显示更改摘要
        sync_manager.show_changes_summary()

        # 5. 选择同步策略
        if args.strategy:
            strategy = args.strategy
        else:
            strategy = sync_manager.sync_strategy_interactive()

        # 6. 执行同步
        if not sync_manager.execute_sync(strategy):
            print("❌ 同步失败或被取消")
            return 1

        # 7. 同步后检查
        sync_manager.post_sync_check()

        print("\n🎉 GitHub同步完成!")
        return 0

    except Exception as e:
        print(f"❌ 同步过程发生错误: {e}")
        return 1

if __name__ == '__main__':
    sys.exit(main())