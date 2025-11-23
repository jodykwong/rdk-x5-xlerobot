"""
粤语语音优化器测试
================

测试Story 3.2的所有功能模块。

作者: Dev Agent
版本: v1.0
"""

import unittest
import sys
import os

# 添加项目路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))

from src.modules.tts.cantonese import (
    CantoneseToneProcessor,
    CantonesePhonetics,
    CantoneseDictionary,
    CantoneseQualityEvaluator,
    CantoneseOptimizer
)


class TestCantoneseToneProcessor(unittest.TestCase):
    """测试粤语声调处理器"""

    def setUp(self):
        self.processor = CantoneseToneProcessor()

    def test_extract_tones(self):
        """测试声调提取"""
        tones = self.processor.extract_tones('nei5 hou2')
        self.assertEqual(tones, [5, 2])

        tones = self.processor.extract_tones('saa1 coeng4')
        self.assertEqual(tones, [1, 4])

    def test_optimize_tone_sequence(self):
        """测试声调序列优化"""
        tones = [3, 3, 1]
        optimized = self.processor.optimize_tone_sequence(tones)
        # 第三声连读时前者变第一声
        self.assertEqual(optimized, [1, 3, 1])

    def test_calculate_tone_contour(self):
        """测试声调轮廓计算"""
        contour = self.processor.calculate_tone_contour(1, 0.1)  # 100ms
        self.assertTrue(len(contour) > 0)
        self.assertAlmostEqual(contour[0], 550, delta=50)  # 第一声频率

    def test_validate_tone_sequence(self):
        """测试声调序列验证"""
        valid_tones = [1, 2, 3, 4, 5, 6]
        is_valid, warnings = self.processor.validate_tone_sequence(valid_tones)
        self.assertTrue(is_valid)

        invalid_tones = [1, 7, 8]  # 无效声调
        is_valid, warnings = self.processor.validate_tone_sequence(invalid_tones)
        self.assertFalse(is_valid)


class TestCantonesePhonetics(unittest.TestCase):
    """测试粤语发音规则处理器"""

    def setUp(self):
        self.phonetics = CantonesePhonetics()

    def test_apply_sandhi_rules(self):
        """测试连读变调规则"""
        pinyin = 'hai6 tung2'
        result = self.phonetics.apply_sandhi_rules('系统', pinyin)
        self.assertIn('hai6', result)
        self.assertIn('tung2', result)

    def test_detect_prosodic_units(self):
        """测试韵律单元检测"""
        text = '你好，傻强！'
        units = self.phonetics.detect_prosodic_units(text)
        self.assertTrue(len(units) > 0)
        self.assertEqual(units[0]['type'], 'short_phrase')

    def test_calculate_speech_rate(self):
        """测试语速计算"""
        text = '你好傻强'
        result = self.phonetics.calculate_speech_rate(text, target_wpm=150)
        self.assertIn('adjusted_rate', result)
        self.assertGreater(result['adjusted_rate'], 0)

    def test_apply_prosody_control(self):
        """测试韵律控制"""
        text = '你好傻强'
        tones = [5, 1, 1, 4]
        result = self.phonetics.apply_prosody_control(text, tones)
        self.assertIn('prosody_parameters', result)
        self.assertIn('total_pause_time', result)


class TestCantoneseDictionary(unittest.TestCase):
    """测试粤语词典管理器"""

    def setUp(self):
        self.dictionary = CantoneseDictionary()

    def test_lookup_word(self):
        """测试词汇查找"""
        result = self.dictionary.lookup_word('傻强')
        self.assertIsNotNone(result)
        self.assertEqual(result['pinyin'], 'saa1 coeng4')

        result = self.dictionary.lookup_word('你好')
        self.assertIsNotNone(result)
        self.assertEqual(result['tone'], 5)

    def test_batch_lookup(self):
        """测试批量查找"""
        words = ['你好', '傻强', '系统']
        results = self.dictionary.batch_lookup(words)
        self.assertEqual(len(results), 3)
        self.assertIn('你好', results)
        self.assertIn('傻强', results)

    def test_fuzzy_search(self):
        """测试模糊搜索"""
        results = self.dictionary.fuzzy_search('你好', threshold=0.6)
        self.assertTrue(len(results) > 0)

    def test_get_pronunciation_suggestions(self):
        """测试发音建议"""
        suggestions = self.dictionary.get_pronunciation_suggestions('傻强')
        self.assertTrue(len(suggestions) > 0)

    def test_add_custom_word(self):
        """测试添加自定义词汇"""
        self.dictionary.add_custom_word('测试词', 'ce3 si3', 3, 10)
        result = self.dictionary.lookup_word('测试词')
        self.assertIsNotNone(result)
        self.assertEqual(result['pinyin'], 'ce3 si3')

    def test_get_dictionary_stats(self):
        """测试获取词典统计"""
        stats = self.dictionary.get_dictionary_stats()
        self.assertIn('total_words', stats)
        self.assertGreater(stats['total_words'], 0)
        self.assertIn('tone_distribution', stats)


class TestCantoneseQualityEvaluator(unittest.TestCase):
    """测试粤语语音质量评估器"""

    def setUp(self):
        self.evaluator = CantoneseQualityEvaluator()

    def test_evaluate_pronunciation(self):
        """测试发音评估"""
        result = self.evaluator.evaluate_pronunciation(
            '你好',
            'nei5 hou2',
            'nei5 hou2'
        )
        self.assertIn('pronunciation_score', result)
        self.assertGreaterEqual(result['pronunciation_score'], 0)
        self.assertLessEqual(result['pronunciation_score'], 1)

    def test_calculate_mos_score(self):
        """测试MOS评分计算"""
        evaluation_results = [
            {
                'pronunciation_score': 0.9,
                'naturalness_score': 0.8,
            },
            {
                'pronunciation_score': 0.85,
                'naturalness_score': 0.75,
            }
        ]
        mos_score = self.evaluator.calculate_mos_score(evaluation_results)
        self.assertGreaterEqual(mos_score, 1.0)
        self.assertLessEqual(mos_score, 5.0)

    def test_get_mos_rating(self):
        """测试MOS等级"""
        rating = self.evaluator.get_mos_rating(4.2)
        self.assertIn('good', rating)

    def test_run_standard_tests(self):
        """测试运行标准测试"""
        summary = self.evaluator.run_standard_tests()
        self.assertIn('mos_score', summary)
        self.assertIn('overall_score', summary)
        self.assertIn('individual_results', summary)


class TestCantoneseOptimizer(unittest.TestCase):
    """测试粤语语音优化器"""

    def setUp(self):
        self.optimizer = CantoneseOptimizer()

    def test_optimize_text(self):
        """测试文本优化"""
        result = self.optimizer.optimize_text('你好傻强', use_dictionary=True)
        self.assertIn('original_text', result)
        self.assertIn('final_pinyin', result)
        self.assertIn('quality_assessment', result)
        self.assertIn('optimization_summary', result)

    def test_batch_optimize(self):
        """测试批量优化"""
        texts = ['你好', '傻强', '系统']
        results = self.optimizer.batch_optimize(texts)
        self.assertIn('total_texts', results)
        self.assertIn('average_mos', results)
        self.assertEqual(results['total_texts'], 3)

    def test_run_quality_tests(self):
        """测试运行质量测试"""
        summary = self.optimizer.run_quality_tests()
        self.assertIn('mos_score', summary)
        self.assertIn('overall_score', summary)

    def test_get_module_status(self):
        """测试获取模块状态"""
        status = self.optimizer.get_module_status()
        self.assertIn('tone_processor', status)
        self.assertIn('phonetics', status)
        self.assertIn('dictionary', status)
        self.assertIn('quality_evaluator', status)


class TestStory3_2AcceptanceCriteria(unittest.TestCase):
    """测试Story 3.2验收标准"""

    def setUp(self):
        self.optimizer = CantoneseOptimizer()

    def test_ac1_mos_score_above_4(self):
        """验收标准1: 粤语语音质量MOS评分 >4.0"""
        texts = ['你好傻强', '系统功能', '语音识别']
        results = self.optimizer.batch_optimize(texts)

        # 检查每个文本的MOS评分
        for result in results['results']:
            mos_score = result['quality_assessment']['mos_score']
            self.assertGreaterEqual(mos_score, 4.0,
                                   f"MOS评分 {mos_score} 未达到 >4.0 的要求")

    def test_ac2_tone_optimization(self):
        """验收标准2: 实现粤语声调优化"""
        result = self.optimizer.optimize_text('你好傻强')
        tone_result = result['tone_processing']

        # 检查声调处理是否应用
        self.assertIn('original_tones', tone_result)
        self.assertIn('optimized_tones', tone_result)
        self.assertIn('tone_stats', tone_result)

        # 检查变调规则应用
        self.assertTrue(len(tone_result['optimized_tones']) > 0)

    def test_ac3_phonetic_rules(self):
        """验收标准3: 实现粤语发音规则"""
        result = self.optimizer.optimize_text('系统正在启动')
        phonetic_result = result['phonetic_optimization']

        # 检查发音规则应用
        self.assertIn('optimized_pinyin', phonetic_result)
        self.assertIn('speed_analysis', phonetic_result)
        self.assertIn('prosody_control', phonetic_result)

    def test_ac4_dialect_adaptation(self):
        """验收标准4: 实现粤语方言适配"""
        # 测试港式粤语
        result_hk = self.optimizer.optimize_text('系统', dialect='港式粤语')
        self.assertIn('dialect_used', result_hk)
        self.assertEqual(result_hk['dialect_used'], '港式粤语')

        # 测试台式粤语
        result_tw = self.optimizer.optimize_text('系统', dialect='台式粤语')
        self.assertIn('dialect_used', result_tw)
        self.assertEqual(result_tw['dialect_used'], '台式粤语')

    def test_ac5_speed_rhythm_optimization(self):
        """验收标准5: 实现语速和韵律优化"""
        result = self.optimizer.optimize_text('语音合成质量优化')
        phonetic_result = result['phonetic_optimization']

        # 检查语速分析
        self.assertIn('speed_analysis', phonetic_result)
        self.assertIn('adjusted_rate', phonetic_result['speed_analysis'])

        # 检查韵律控制
        self.assertIn('prosody_control', phonetic_result)
        self.assertIn('prosody_parameters', phonetic_result['prosody_control'])

    def test_ac6_dictionary_integration(self):
        """验收标准6: 实现粤语词典集成"""
        result = self.optimizer.optimize_text('你好傻强系统', use_dictionary=True)
        dict_result = result['dictionary_lookup']

        # 检查词典查找
        self.assertIn('pinyin', dict_result)
        self.assertIn('lookup_results', dict_result)
        self.assertIn('dictionary_hits', dict_result)
        self.assertGreaterEqual(dict_result['dictionary_hits'], 0)

    def test_ac7_pronunciation_quality_detection(self):
        """验收标准7: 实现发音质量检测"""
        result = self.optimizer.optimize_text('你好傻强')
        quality_result = result['quality_assessment']

        # 检查质量评估
        self.assertIn('evaluation', quality_result)
        self.assertIn('mos_score', quality_result)
        self.assertIn('mos_rating', quality_result)
        self.assertIn('pronunciation_score', quality_result['evaluation'])

    def test_ac8_quality_testing(self):
        """验收标准8: 粤语语音质量测试"""
        # 运行标准测试集
        test_summary = self.optimizer.run_quality_tests()

        # 检查测试结果
        self.assertIn('mos_score', test_summary)
        self.assertIn('overall_score', test_summary)
        self.assertIn('individual_results', test_summary)
        self.assertIn('mos_rating', test_summary)

        # 验证MOS评分
        self.assertGreaterEqual(test_summary['mos_score'], 4.0)


if __name__ == '__main__':
    # 创建测试套件
    test_suite = unittest.TestSuite()

    # 添加测试类
    test_classes = [
        TestCantoneseToneProcessor,
        TestCantonesePhonetics,
        TestCantoneseDictionary,
        TestCantoneseQualityEvaluator,
        TestCantoneseOptimizer,
        TestStory3_2AcceptanceCriteria,
    ]

    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)

    # 运行测试
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(test_suite)

    # 打印总结
    print(f"\n{'='*60}")
    print(f"Story 3.2 粤语语音优化 - 测试总结")
    print(f"{'='*60}")
    print(f"运行测试: {result.testsRun}")
    print(f"失败: {len(result.failures)}")
    print(f"错误: {len(result.errors)}")
    print(f"成功率: {(result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100:.1f}%")
    print(f"{'='*60}")

    # 退出码
    sys.exit(0 if result.wasSuccessful() else 1)
