#!/usr/bin/env python3.10
"""
ASR音频访问快速修复补丁
========================

直接替换ASR系统中的ThreadSafeAudioRecorder为工作版本。

作者: Claude Code Agent
日期: 2025-11-18
"""

import os
import sys
import time
import logging

# 设置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def apply_audio_fix():
    """应用音频修复"""
    print("🔧 开始应用ASR音频访问修复...")

    # 备份原始文件
    asr_system_path = "/home/sunrise/xlerobot/src/modules/asr/asr_system.py"
    backup_path = f"{asr_system_path}.backup_{int(time.time())}"

    try:
        # 读取原始文件
        with open(asr_system_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 创建备份
        with open(backup_path, 'w', encoding='utf-8') as f:
            f.write(content)
        logger.info(f"✅ 已创建备份: {backup_path}")

        # 应用修复1：添加SimpleALSARecorder导入
        if "from .simple_alsa_recorder import SimpleALSARecorder" not in content:
            # 在ThreadSafeAudioRecorder导入后添加
            import_line = "from .thread_safe_audio_recorder import ThreadSafeAudioRecorder"
            new_import = f"""{import_line}
from .simple_alsa_recorder import SimpleALSARecorder
"""

            content = content.replace(import_line, new_import)
            logger.info("✅ 添加SimpleALSARecorder导入")

        # 应用修复2：修改ThreadSafeAudioRecorder为别名
        if "ThreadSafeAudioRecorder = None" in content:
            # 找到这行并修改
            content = content.replace("self.audio_recorder = None",
                                        "# self.audio_recorder = None  # 由ALSA录音器替代")
            logger.info("✅ 修改了None赋值")

        # 应用修复3：在初始化中使用ALSA录音器
        if "self.audio_recorder = ThreadSafeAudioRecorder()" in content:
            # 替换为ALSA录音器
            content = content.replace("self.audio_recorder = ThreadSafeAudioRecorder()",
                                        "# 使用ALSA录音器替代ThreadSafeAudioRecorder\n                        # self.audio_recorder = ThreadSafeAudioRecorder()")
            logger.info("✅ 修改了ThreadSafeAudioRecorder初始化")

        # 添加ALSA录音器初始化
        if "# 使用ALSA录音器替代ThreadSafeAudioRecorder" in content:
            alsa_init_code = """                        # 使用ALSA录音器替代ThreadSafeAudioRecorder
                        try:
                            self.audio_recorder = SimpleALSARecorder()
                            logger.info("✅ 使用ALSA录音器初始化成功")
                        except Exception as e:
                            logger.error(f"❌ ALSA录音器初始化失败: {e}")
                            self.audio_recorder = None"""

            content = content.replace("# 使用ALSA录音器替代ThreadSafeAudioRecorder", alsa_init_code)
            logger.info("✅ 添加ALSA录音器初始化代码")

        # 应用修复4：在回退路径中也使用ALSA录音器
        if "self.audio_recorder = ThreadSafeAudioRecorder()" in content:
            # 替换所有剩余的ThreadSafeAudioRecorder
            content = content.replace("self.audio_recorder = ThreadSafeAudioRecorder()",
                                        "self.audio_recorder = SimpleALSARecorder()")
            logger.info("✅ 替换所有ThreadSafeAudioRecorder实例化")

        # 写入修复后的文件
        with open(asr_system_path, 'w', encoding='utf-8') as f:
            f.write(content)

        logger.info("✅ ASR音频访问修复应用完成")
        return True

    except Exception as e:
        logger.error(f"❌ 修复应用失败: {e}")
        return False

def test_fix():
    """测试修复效果"""
    print("🧪 测试修复效果...")

    try:
        # 加载修复后的ASR系统
        sys.path.insert(0, '/home/sunrise/xlerobot/src')
        from modules.asr.asr_system import ASRSystem

        # 创建ASR系统实例
        asr_system = ASRSystem()

        # 初始化系统
        print("🔄 初始化ASR系统...")
        success = asr_system.initialize()

        if success:
            print("✅ ASR系统初始化成功")

            # 检查录音器
            if asr_system.audio_recorder:
                print(f"✅ 录音器已初始化: {type(asr_system.audio_recorder).__name__}")

                # 测试录音器
                print("🎤 测试录音器...")
                test_success = asr_system.audio_recorder.test_recording()

                if test_success:
                    print("🎉 录音器测试成功！")
                    print("✅ ASR音频访问问题已修复！")
                    return True
                else:
                    print("❌ 录音器测试失败")
                    return False
            else:
                print("❌ 录音器未初始化")
                return False
        else:
            print("❌ ASR系统初始化失败")
            return False

    except Exception as e:
        logger.error(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """主函数"""
    import time
    import argparse

    parser = argparse.ArgumentParser(description='ASR音频访问修复工具')
    parser.add_argument('--test', action='store_true', help='应用修复后进行测试')
    parser.add_argument('--backup', action='store_true', help='仅创建备份，不修改原文件')

    args = parser.parse_args()

    print("🚀 XLeRobot ASR音频访问修复工具")
    print("=" * 50)

    if args.backup:
        print("📋 仅创建备份模式，不修改原文件")
        # 仅创建备份
        asr_system_path = "/home/sunrise/xlerobot/src/modules/asr/asr_system.py"
        backup_path = f"{asr_system_path}.backup_{int(time.time())}"

        try:
            with open(asr_system_path, 'r', encoding='utf-8') as f:
                content = f.read()
            with open(backup_path, 'w', encoding='utf-8') as f:
                f.write(content)
            print(f"✅ 备份已创建: {backup_path}")
            return True
        except Exception as e:
            print(f"❌ 备份创建失败: {e}")
            return False

    # 应用修复
    if apply_audio_fix():
        print("✅ 修复应用成功")

        if args.test:
            print("\n🧪 开始测试修复效果...")
            if test_fix():
                print("\n🎉 修复验证成功！")
                print("📋 ASR系统现在可以正常录制音频了")
                print("🎯 \"傻强\"语音助手应该能够正常工作了")
                return True
            else:
                print("\n❌ 修复验证失败")
                return False
        else:
            print("📋 修复应用完成，建议运行 --test 参数进行验证")
            return True
    else:
        print("❌ 修复应用失败")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)