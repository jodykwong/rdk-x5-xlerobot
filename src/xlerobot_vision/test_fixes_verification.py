#!/usr/bin/env python3.10
"""
Story 1.6 修复验证测试脚本
验证Senior Developer Review中发现的高优先级问题修复效果

修复内容:
1. 移除硬编码API密钥，使用环境变量
2. 扩展粤语术语优化器
"""

import os
import sys
import unittest
from unittest.mock import patch, MagicMock

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

from xlerobot_vision.qwen_vl_client import (
    QwenVLPlusClient, QwenVLConfig, CantoneseVisualOptimizer,
    XleRobotVisionError, ImageProcessor
)


class TestSecurityFixes(unittest.TestCase):
    """测试安全修复：API密钥管理"""

    def test_api_key_from_environment(self):
        """测试从环境变量读取API密钥"""
        # 设置测试环境变量
        test_api_key = "test-api-key-12345"
        with patch.dict(os.environ, {'DASHSCOPE_API_KEY': test_api_key}):
            config = QwenVLConfig()
            client = QwenVLPlusClient(config)
            self.assertEqual(client.config.api_key, test_api_key)

    def test_missing_api_key_raises_error(self):
        """测试缺少API密钥时抛出异常"""
        # 确保环境变量未设置
        with patch.dict(os.environ, {}, clear=True):
            with self.assertRaises(XleRobotVisionError) as context:
                QwenVLPlusClient()
            self.assertEqual(context.exception.error_code, "MISSING_API_KEY")

    def test_hardcoded_api_key_removed(self):
        """验证硬编码API密钥已移除"""
        config = QwenVLConfig()
        self.assertEqual(config.api_key, "")  # 空字符串表示已移除硬编码


class TestCantoneseOptimizerEnhancement(unittest.TestCase):
    """测试粤语优化器增强"""

    def setUp(self):
        self.optimizer = CantoneseVisualOptimizer()

    def test_basic_term_replacement(self):
        """测试基础术语替换"""
        test_cases = [
            ("桌子上有苹果", "枱上有蘋果"),
            ("电视在椅子旁边", "電視機在櫈旁边"),
            ("这是红色的门", "呢個係紅色嘅門"),
            ("看那个蓝色的电视", "睇嗰個藍色嘅電視機")
        ]

        for input_text, expected in test_cases:
            with self.subTest(input_text=input_text):
                result = self.optimizer.optimize_response(input_text)
                self.assertEqual(result, expected)

    def test_extended_vocabulary(self):
        """测试扩展词汇库"""
        test_cases = [
            ("洗衣机在沙发旁边", "洗衣機在梳化旁边"),
            ("猫和狗在窗户外面", "貓同狗在窗戶外面"),
            ("坐公交车去地铁", "坐巴士去地鐵"),
            ("冰箱里有鸡蛋和牛奶", "雪櫃裡面有雞蛋同牛奶")
        ]

        for input_text, expected in test_cases:
            with self.subTest(input_text=input_text):
                result = self.optimizer.optimize_response(input_text)
                self.assertEqual(result, expected)

    def test_grammar_optimization(self):
        """测试语法优化"""
        test_cases = [
            ("我吃了", "我食咗"),
            ("看着电视", "睇住電視機"),
            ("去过那里", "去過嗰度"),
            ("不好", "唔好"),
            ("没有问题", "冇問題"),
            ("好吗？", "好嘛？"),
            ("红的", "紅嘅")
        ]

        for input_text, expected in test_cases:
            with self.subTest(input_text=input_text):
                result = self.optimizer.optimize_response(input_text)
                self.assertEqual(result, expected)

    def test_cantonese_prompt_enhancement(self):
        """测试粤语提示词增强"""
        prompt = "这是什么？"
        result = self.optimizer.add_cantonese_prompt(prompt)

        # 验证增强的提示词包含粤语要求
        self.assertIn("純正廣東話", result)
        self.assertIn("地道嘅粵語詞彙", result)
        self.assertIn("粵語語法習慣", result)

    def test_optimization_coverage(self):
        """测试优化覆盖率 - 验证AC-002改进"""
        test_text = """
        这是一个红色的桌子，上面有苹果和香蕉。电视里播放着动画片，
        猫和狗在沙发旁边玩。洗衣机在阳台，冰箱里有鸡蛋和牛奶。
        我已经吃了饭，现在看着窗外。好吗？
        """

        result = self.optimizer.optimize_response(test_text)

        # 计算粤语词汇覆盖率
        total_terms = len(self.optimizer.visual_terms)
        found_terms = sum(1 for term in self.optimizer.visual_terms.keys()
                         if term in test_text)

        # 验证结果包含粤语词汇
        self.assertIn("呢個", result)  # 这个 -> 呢個
        self.assertIn("枱", result)    # 桌子 -> 枱
        self.assertIn("蘋果", result)  # 苹果 -> 蘋果
        self.assertIn("電視機", result) # 电视 -> 電視機
        self.assertIn("貓", result)    # 猫 -> 貓
        self.assertIn("梳化", result)  # 沙发 -> 梳化
        self.assertIn("食咗", result)  # 吃了 -> 食咗
        self.assertIn("睇住", result)  # 看着 -> 睇住

    def test_vocabulary_size_improvement(self):
        """验证词汇库大小改进"""
        # 原来的词汇库约50个词，现在应该大幅增加
        vocab_size = len(self.optimizer.visual_terms)
        self.assertGreater(vocab_size, 100, "粤语词汇库应该超过100个词")

        # 分类统计
        categories = {
            '家居用品': ['桌子', '椅子', '电视', '冰箱', '洗衣机'],
            '食物': ['苹果', '香蕉', '米饭', '面条', '鸡蛋'],
            '动物': ['猫', '狗', '鸟', '鱼', '兔子'],
            '交通工具': ['汽车', '自行车', '公交车', '地铁'],
            '常用词汇': ['什么', '这个', '看', '说', '吃']
        }

        for category, terms in categories.items():
            found_terms = [term for term in terms if term in self.optimizer.visual_terms]
            self.assertGreater(len(found_terms), 0,
                             f"{category}类别应该包含相关词汇")


class TestIntegration(unittest.TestCase):
    """集成测试"""

    def test_client_with_environment_variables(self):
        """测试客户端与环境变量集成"""
        test_api_key = "integration-test-key"

        with patch.dict(os.environ, {'DASHSCOPE_API_KEY': test_api_key}):
            # 模拟API响应
            with patch('requests.Session.post') as mock_post:
                mock_response = MagicMock()
                mock_response.json.return_value = {
                    'choices': [{'message': {'content': '测试响应'}}]
                }
                mock_response.raise_for_status.return_value = None
                mock_post.return_value = mock_response

                client = QwenVLPlusClient()

                # 验证API密钥已正确设置
                self.assertEqual(client.config.api_key, test_api_key)

                # 验证请求头包含正确的API密钥
                self.assertIn('Authorization', client.session.headers)
                self.assertEqual(client.session.headers['Authorization'], f'Bearer {test_api_key}')
                self.assertEqual(client.session.headers['Content-Type'], 'application/json')


def main():
    """运行所有测试"""
    print("🧪 Story 1.6 修复验证测试")
    print("=" * 60)
    print("测试项目:")
    print("1. API密钥安全修复")
    print("2. 粤语优化器扩展")
    print("3. 集成测试")
    print("=" * 60)

    # 设置测试环境变量
    os.environ['DASHSCOPE_API_KEY'] = 'test-key-for-verification'

    # 运行测试
    unittest.main(verbosity=2, exit=False)

    print("\n✅ 所有测试完成")


if __name__ == '__main__':
    main()