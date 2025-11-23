"""
音色功能测试套件
==============

测试所有音色相关功能：多音色支持、参数控制、混合渐变、质量评估等。

作者: Dev Agent
"""

import unittest
import numpy as np
import time
from typing import Dict, Any
import logging
import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

from modules.tts.voices.voice_manager import VoiceManager, VoiceProfile, VoiceType
from modules.tts.voices.voice_controller import VoiceController, VoiceParameters, BlendMode
from modules.tts.voices.voice_cache import VoiceCache
from modules.tts.voices.voice_quality import VoiceQualityEvaluator


class TestVoiceManager(unittest.TestCase):
    """音色管理器测试"""

    def setUp(self):
        """测试前准备"""
        self.voice_manager = VoiceManager()

    def test_list_voices(self):
        """测试列出音色"""
        voices = self.voice_manager.list_voices()
        self.assertGreater(len(voices), 0)
        print(f"✓ 已加载 {len(voices)} 个音色")

    def test_get_voice(self):
        """测试获取音色"""
        voice = self.voice_manager.get_voice("voice_female_001")
        self.assertIsNotNone(voice)
        self.assertEqual(voice.voice_id, "voice_female_001")
        print(f"✓ 获取音色成功: {voice.name}")

    def test_switch_voice(self):
        """测试音色切换"""
        # 切换到年轻女性音色
        success = self.voice_manager.switch_voice("voice_female_001")
        self.assertTrue(success)

        current = self.voice_manager.get_current_voice()
        self.assertIsNotNone(current)
        self.assertEqual(current.voice_id, "voice_female_001")
        print(f"✓ 音色切换成功: {current.name}")

    def test_clone_voice(self):
        """测试音色克隆"""
        success = self.voice_manager.clone_voice(
            "voice_female_001",
            "voice_clone_test",
            "测试克隆音色"
        )
        self.assertTrue(success)

        cloned_voice = self.voice_manager.get_voice("voice_clone_test")
        self.assertIsNotNone(cloned_voice)
        print(f"✓ 音色克隆成功: {cloned_voice.name}")

    def test_backup_restore(self):
        """测试备份和恢复"""
        # 创建备份
        backup_path = "/tmp/voice_backup_test.json"
        success = self.voice_manager.backup_voices(backup_path)
        self.assertTrue(success)
        self.assertTrue(os.path.exists(backup_path))

        # 修改音色配置
        original_count = len(self.voice_manager.voices)

        # 恢复备份
        success = self.voice_manager.restore_voices(backup_path)
        self.assertTrue(success)

        # 验证恢复结果
        restored_count = len(self.voice_manager.voices)
        self.assertEqual(original_count, restored_count)
        print(f"✓ 备份恢复测试成功")

        # 清理
        os.remove(backup_path)

    def test_voice_statistics(self):
        """测试音色统计"""
        stats = self.voice_manager.get_voice_statistics()
        self.assertIn('total_voices', stats)
        self.assertIn('voice_types', stats)
        print(f"✓ 音色统计: {stats}")


class TestVoiceController(unittest.TestCase):
    """音色控制器测试"""

    def setUp(self):
        """测试前准备"""
        self.controller = VoiceController()

    def test_set_parameters(self):
        """测试设置参数"""
        params = VoiceParameters(
            brightness=0.8,
            warmth=0.7,
            clarity=0.9
        )
        success = self.controller.set_parameters(params)
        self.assertTrue(success)

        current = self.controller.get_current_parameters()
        self.assertAlmostEqual(current.brightness, 0.8, places=2)
        print(f"✓ 参数设置成功: brightness={current.brightness}")

    def test_transition_parameters(self):
        """测试参数过渡"""
        start_params = VoiceParameters(brightness=0.5)
        target_params = VoiceParameters(brightness=0.9)

        success = self.controller.set_parameters(target_params, transition_duration=0.1)
        self.assertTrue(success)

        time.sleep(0.15)
        self.controller.update_transitions()

        current = self.controller.get_current_parameters()
        self.assertAlmostEqual(current.brightness, 0.9, places=2)
        print(f"✓ 参数过渡成功")

    def test_blend_voices(self):
        """测试音色混合"""
        voice1_params = VoiceParameters(brightness=0.5, warmth=0.5)
        voice2_params = VoiceParameters(brightness=0.9, warmth=0.9)

        blended = self.controller.blend_voices(
            voice1_params,
            voice2_params,
            BlendMode.INTERPOLATE,
            0.5
        )

        self.assertAlmostEqual(blended.brightness, 0.7, places=2)
        self.assertAlmostEqual(blended.warmth, 0.7, places=2)
        print(f"✓ 音色混合成功")

    def test_modulate_parameters(self):
        """测试音调调制"""
        modulated = self.controller.modulate_parameters(
            modulation_type="vibrato",
            intensity=0.5,
            frequency=5.0
        )

        self.assertIn('pitch', modulated)
        print(f"✓ 音调调制成功")


class TestVoiceCache(unittest.TestCase):
    """音色缓存测试"""

    def setUp(self):
        """测试前准备"""
        self.cache = VoiceCache(cache_dir="/tmp/tts_voice_cache_test", max_size=10)

    def test_cache_put_get(self):
        """测试缓存存取"""
        voice_id = "test_voice"
        model_data = {'speaker_id': 0, 'model_loaded': True}

        self.cache.put(voice_id, model_data)

        cached_data = self.cache.get(voice_id)
        self.assertIsNotNone(cached_data)
        self.assertEqual(cached_data['model_data']['speaker_id'], 0)
        print(f"✓ 缓存存取成功")

    def test_cache_eviction(self):
        """测试缓存驱逐"""
        # 填满缓存
        for i in range(12):  # 超过max_size
            self.cache.put(f"voice_{i}", {'data': i})

        # 检查缓存大小
        self.assertLessEqual(len(self.cache.memory_cache), 10)
        print(f"✓ 缓存驱逐正常")

    def test_cache_stats(self):
        """测试缓存统计"""
        stats = self.cache.get_cache_stats()
        self.assertIn('hits', stats)
        self.assertIn('misses', stats)
        print(f"✓ 缓存统计: {stats}")


class TestVoiceQualityEvaluator(unittest.TestCase):
    """音色质量评估测试"""

    def setUp(self):
        """测试前准备"""
        self.evaluator = VoiceQualityEvaluator()

    def test_evaluate_voice_quality(self):
        """测试音色质量评估"""
        # 生成测试音频
        sample_rate = 22050
        duration = 2.0
        t = np.linspace(0, duration, int(sample_rate * duration))
        audio = 0.5 * np.sin(2 * np.pi * 440 * t)  # 440Hz正弦波

        metrics = self.evaluator.evaluate_voice_quality(audio, sample_rate, "test_voice")

        self.assertIn('mos_score', metrics.__dict__)
        self.assertGreater(metrics.mos_score, 0)
        self.assertLess(metrics.mos_score, 6)
        print(f"✓ 质量评估完成: MOS={metrics.mos_score:.2f}")


class TestIntegratedTTSVoices(unittest.TestCase):
    """集成测试：TTS多音色功能"""

    def setUp(self):
        """测试前准备"""
        # 创建模拟TTS引擎配置
        self.config = {
            'voices': {},
            'cache_dir': '/tmp/tts_voice_cache_integration',
            'max_cache_size': 100
        }

        # 由于无法直接导入TTSEngine（循环依赖），我们测试各个组件
        self.voice_manager = VoiceManager()
        self.controller = VoiceController()

    def test_voice_switching_performance(self):
        """测试音色切换性能"""
        voice_ids = list(self.voice_manager.voices.keys())

        switch_times = []
        for voice_id in voice_ids:
            start_time = time.time()
            self.voice_manager.switch_voice(voice_id)
            switch_time = (time.time() - start_time) * 1000
            switch_times.append(switch_time)

        avg_switch_time = np.mean(switch_times)
        print(f"✓ 音色切换性能测试: 平均 {avg_switch_time:.2f}ms")

        # 验证切换时间 < 100ms
        self.assertLess(avg_switch_time, 100)

    def test_voice_parameter_control(self):
        """测试音色参数控制"""
        # 设置不同的音色参数
        test_cases = [
            VoiceParameters(brightness=0.9, warmth=0.5, clarity=0.9),
            VoiceParameters(brightness=0.3, warmth=0.8, clarity=0.7),
            VoiceParameters(brightness=0.5, warmth=0.5, clarity=0.5),
        ]

        for params in test_cases:
            success = self.controller.set_parameters(params)
            self.assertTrue(success)

            current = self.controller.get_current_parameters()
            self.assertAlmostEqual(current.brightness, params.brightness, places=2)

        print(f"✓ 音色参数控制测试通过")

    def test_voice_quality_assessment(self):
        """测试音色质量评估"""
        evaluator = VoiceQualityEvaluator()

        # 生成测试音频
        sample_rate = 22050
        duration = 1.0
        t = np.linspace(0, duration, int(sample_rate * duration))
        audio = 0.5 * np.sin(2 * np.pi * 440 * t) + 0.1 * np.random.randn(len(t))

        # 测试所有音色
        for voice_id in self.voice_manager.voices.keys():
            metrics = evaluator.evaluate_voice_quality(audio, sample_rate, voice_id)
            self.assertGreaterEqual(metrics.mos_score, 1.0)
            self.assertLessEqual(metrics.mos_score, 5.0)

        print(f"✓ 音色质量评估测试通过: {len(self.voice_manager.voices)} 个音色")


def run_voice_tests():
    """运行所有音色测试"""
    # 设置日志
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

    # 创建测试套件
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # 添加测试用例
    suite.addTests(loader.loadTestsFromTestCase(TestVoiceManager))
    suite.addTests(loader.loadTestsFromTestCase(TestVoiceController))
    suite.addTests(loader.loadTestsFromTestCase(TestVoiceCache))
    suite.addTests(loader.loadTestsFromTestCase(TestVoiceQualityEvaluator))
    suite.addTests(loader.loadTestsFromTestCase(TestIntegratedTTSVoices))

    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # 总结测试结果
    print("\n" + "="*80)
    print("测试结果总结:")
    print(f"  运行测试: {result.testsRun}")
    print(f"  成功: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"  失败: {len(result.failures)}")
    print(f"  错误: {len(result.errors)}")
    print(f"  成功率: {(result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100:.1f}%")
    print("="*80)

    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_voice_tests()
    sys.exit(0 if success else 1)
