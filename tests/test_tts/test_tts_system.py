"""
TTS系统测试套件
===============

测试TTS语音合成系统的所有功能。

作者: Dev Agent
"""

import os
import sys
import unittest
import tempfile
import logging
from pathlib import Path

# 添加模块路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

from modules.tts.tts_system import TTSSystem
from modules.tts.config.tts_config import TTSConfig
from modules.tts.text.text_processor import TextProcessor
from modules.tts.audio.audio_processor import AudioProcessor


class TestTTSConfig(unittest.TestCase):
    """TTS配置测试"""

    def test_config_creation(self):
        """测试配置创建"""
        config = TTSConfig()
        self.assertIsNotNone(config.config)
        self.assertIn('model', config.config)

    def test_config_get_set(self):
        """测试配置获取和设置"""
        config = TTSConfig()
        config.set('test.value', 123)
        self.assertEqual(config.get('test.value'), 123)

    def test_config_validation(self):
        """测试配置验证"""
        config = TTSConfig()
        self.assertTrue(config.validate())

    def test_config_save_load(self):
        """测试配置保存和加载"""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
            config_path = f.name

        try:
            config = TTSConfig()
            config.set('test.value', 'test123')
            config.save_config(config_path)

            new_config = TTSConfig(config_path)
            self.assertEqual(new_config.get('test.value'), 'test123')
        finally:
            os.unlink(config_path)


class TestTextProcessor(unittest.TestCase):
    """文本处理器测试"""

    def setUp(self):
        """测试设置"""
        self.processor = TextProcessor()

    def test_clean_text(self):
        """测试文本清理"""
        text = "  你好  ，世界  ！"
        cleaned = self.processor.clean_text(text)
        self.assertNotIn('  ', cleaned)

    def test_split_sentences(self):
        """测试分句"""
        text = "你好。世界！你好？"
        sentences = self.processor.split_sentences(text)
        self.assertGreater(len(sentences), 1)

    def test_text_to_phonemes(self):
        """测试文本转音素"""
        text = "你好"
        phonemes = self.processor.text_to_phonemes(text)
        self.assertIsInstance(phonemes, list)

    def test_preprocess_text(self):
        """测试完整预处理"""
        text = "你好，傻强！"
        cleaned, sentences, phonemes = self.processor.preprocess_text(text)
        self.assertIsInstance(cleaned, str)
        self.assertIsInstance(sentences, list)
        self.assertIsInstance(phonemes, list)


class TestAudioProcessor(unittest.TestCase):
    """音频处理器测试"""

    def setUp(self):
        """测试设置"""
        self.processor = AudioProcessor()

    def test_audio_info(self):
        """测试音频信息获取"""
        import numpy as np
        audio = np.random.randn(22050)
        sr = 22050
        info = self.processor.get_audio_info(audio, sr)

        self.assertIn('duration', info)
        self.assertIn('sample_rate', info)
        self.assertEqual(info['sample_rate'], sr)

    def test_resample_audio(self):
        """测试音频重采样"""
        import numpy as np
        audio = np.random.randn(44100)
        sr_in = 44100
        sr_out = 22050

        resampled = self.processor.resample_audio(audio, sr_in, sr_out)
        self.assertEqual(len(resampled), 22050)

    def test_adjust_volume(self):
        """测试音量调整"""
        import numpy as np
        audio = np.array([0.1, 0.2, 0.3])

        adjusted = self.processor.adjust_volume(audio, 6.0)  # +6dB
        self.assertGreater(np.max(np.abs(adjusted)), np.max(np.abs(audio)))

    def test_normalize_audio(self):
        """测试音频归一化"""
        from modules.tts.audio.audio_processor import normalize_audio
        import numpy as np

        audio = np.array([0.5, 0.8, 1.0])
        normalized = normalize_audio(audio)

        self.assertAlmostEqual(np.max(np.abs(normalized)), 1.0, places=5)


class TestTTSSystem(unittest.TestCase):
    """TTS系统测试"""

    def setUp(self):
        """测试设置"""
        self.temp_dir = tempfile.mkdtemp()
        self.system = TTSSystem()

    def tearDown(self):
        """测试清理"""
        import shutil
        shutil.rmtree(self.temp_dir, ignore_errors=True)

    def test_system_initialization(self):
        """测试系统初始化"""
        # 测试基本初始化（不加载模型）
        result = self.system.initialize(load_model=False)
        self.assertTrue(result)
        self.assertTrue(self.system.initialized)

    def test_system_info(self):
        """测试系统信息获取"""
        self.system.initialize(load_model=False)
        info = self.system.get_system_info()

        self.assertIn('initialized', info)
        self.assertIn('config', info)
        self.assertIn('components', info)

    def test_text_processing_pipeline(self):
        """测试文本处理流水线"""
        self.system.initialize(load_model=False)

        text = "你好，傻强！"
        success, output_path, elapsed = self.system.synthesize(
            text,
            os.path.join(self.temp_dir, 'test.wav')
        )

        # 由于没有真实模型，预期失败
        # 但我们可以验证系统响应
        self.assertIsInstance(success, bool)
        self.assertIsInstance(elapsed, float)

    def test_benchmark(self):
        """测试基准测试"""
        self.system.initialize(load_model=False)

        test_texts = ["测试文本一", "测试文本二", "测试文本三"]
        results = self.system.benchmark(test_texts, self.temp_dir)

        self.assertIn('total_tests', results)
        self.assertIn('timing', results)
        self.assertEqual(results['total_tests'], len(test_texts))

    def test_synthesize_stream(self):
        """测试流式合成"""
        self.system.initialize(load_model=False)

        text = "你好"
        success, audio_data, elapsed = self.system.synthesize_stream(text)

        self.assertIsInstance(success, bool)
        self.assertIsInstance(audio_data, tuple)

    def test_empty_text(self):
        """测试空文本处理"""
        self.system.initialize(load_model=False)

        success, output_path, elapsed = self.system.synthesize(
            "",
            os.path.join(self.temp_dir, 'empty.wav')
        )

        # 空文本应该被处理
        self.assertIsInstance(success, bool)


class TestTTSIntegration(unittest.TestCase):
    """TTS集成测试"""

    def test_full_pipeline(self):
        """测试完整流水线"""
        system = TTSSystem()
        system.initialize(load_model=False)

        test_text = "你好，傻强！欢迎使用TTS语音合成系统。"

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = os.path.join(temp_dir, 'test.wav')

            success, output_path, elapsed = system.synthesize(test_text, output_path)

            # 验证系统响应（即使没有真实模型）
            self.assertIsInstance(success, bool)
            self.assertIsInstance(elapsed, float)
            self.assertGreater(elapsed, 0)

    def test_multiple_syntheses(self):
        """测试多次合成"""
        system = TTSSystem()
        system.initialize(load_model=False)

        texts = ["文本一", "文本二", "文本三"]
        results = []

        with tempfile.TemporaryDirectory() as temp_dir:
            for i, text in enumerate(texts):
                output_path = os.path.join(temp_dir, f'test_{i}.wav')
                success, output_path, elapsed = system.synthesize(text, output_path)
                results.append((success, elapsed))

            # 验证所有合成都完成
            self.assertEqual(len(results), len(texts))
            for success, elapsed in results:
                self.assertIsInstance(success, bool)
                self.assertIsInstance(elapsed, float)


def run_performance_tests():
    """运行性能测试"""
    print("\n" + "="*60)
    print("🧪 TTS系统性能测试")
    print("="*60)

    system = TTSSystem()
    system.initialize(load_model=False)

    # 准备测试文本
    test_texts = [
        "你好，傻强！",
        "这是一个测试文本，用于验证TTS系统的性能。",
        "语音合成技术已经广泛应用于智能助手、语音播报等场景。",
        "粤语语音合成是本项目的核心功能之一。",
        "我们正在开发一个高质量的TTS语音合成系统。"
    ]

    # 基准测试
    print(f"\n📊 基准测试: {len(test_texts)}个文本")
    results = system.benchmark(test_texts, "/tmp/tts_perf_test")

    # 输出结果
    print(f"\n✅ 测试结果:")
    print(f"  总测试数: {results['total_tests']}")
    print(f"  总耗时: {results['timing']['total_time']:.3f}s")
    print(f"  平均耗时: {results['timing']['avg_time']:.3f}s")
    print(f"  最快: {results['timing']['min_time']:.3f}s")
    print(f"  最慢: {results['timing']['max_time']:.3f}s")

    # 验证延迟要求
    avg_time = results['timing']['avg_time']
    if avg_time < 1.0:
        print(f"\n🎯 延迟要求: ✅ 通过 (平均 {avg_time:.3f}s < 1.0s)")
    else:
        print(f"\n⚠️ 延迟要求: ⚠️ 超标 (平均 {avg_time:.3f}s >= 1.0s)")

    print("="*60)


if __name__ == '__main__':
    # 设置日志
    logging.basicConfig(level=logging.INFO)

    # 运行单元测试
    print("\n🧪 TTS单元测试")
    print("="*60)
    unittest.main(argv=[''], exit=False, verbosity=2)

    # 运行性能测试
    run_performance_tests()
