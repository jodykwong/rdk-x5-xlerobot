#!/usr/bin/env python3.10
"""
快速音频修复方案
==============

直接替换ThreadSafeAudioRecorder为ALSA录音器。

作者: Claude Code Agent
日期: 2025-11-18
"""

import os
import sys

def apply_quick_fix():
    """应用快速修复"""
    print("🔧 开始快速音频修复...")

    # 备份原始文件
    tsar_path = "/home/sunrise/xlerobot/src/modules/asr/thread_safe_audio_recorder.py"
    backup_path = f"{tsar_path}.backup_{int(time.time())}"

    try:
        # 读取ThreadSafeAudioRecorder文件
        with open(tsar_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # 创建备份
        with open(backup_path, 'w', encoding='utf-8') as f:
            f.write(content)
        print(f"✅ 已创建备份: {backup_path}")

        # 在文件末尾添加ALSA别名
        alsa_alias = '''
# 向后兼容的别名 - 替换为ALSA录音器
class ThreadSafeAudioRecorder:
    def __init__(self, *args, **kwargs):
        print("⚠️ ThreadSafeAudioRecorder正在被ALSA录音器替代")
        from .simple_alsa_recorder import SimpleALSARecorder
        self._recorder = SimpleALSARecorder(*args, **kwargs)
        self.start_recording = self._recorder.start_recording
        self.stop_recording = self._recorder.stop_recording
        self.get_state = self._recorder.get_state
        self.get_completion_event = self._recorder.get_completion_event
        self.get_stats = self._recorder.get_stats
        self.test_recording = self._recorder.test_recording
        self.force_stop = self._recorder.force_stop
        self.get_audio_config = self._recorder.get_audio_config
        self.__del__ = self._recorder.__del__
        self.__class__.__name__ = 'ThreadSafeAudioRecorder'

# 创建实例函数
def create_thread_safe_recorder():
    from .simple_alsa_recorder import SimpleALSARecorder
    return SimpleALSARecorder()

# 确保向后兼容
if 'ThreadSafeAudioRecorder' not in globals():
    globals()['ThreadSafeAudioRecorder'] = ThreadSafeRecorder
'''

        # 在文件末尾添加别名代码
        content += alsa_alias

        # 写入修改后的文件
        with open(tsar_path, 'w', encoding='utf-8') as f:
            f.write(content)

        print("✅ ThreadSafeAudioRecorder已被ALSA录音器替代")
        return True

    except Exception as e:
        print(f"❌ 快速修复失败: {e}")
        return False

def main():
    """主函数"""
    print("🚀 快速音频修复方案")
    print("=" * 40)

    if apply_quick_fix():
        print("✅ 快速修复应用成功")
        print("📋 现在ThreadSafeAudioRecorder实际使用ALSA录音器")
        print("🎯 ASR系统应该能够正常录音了")
        return True
    else:
        print("❌ 快速修复失败")
        return False

if __name__ == "__main__":
    import time
    success = main()
    sys.exit(0 if success else 1)