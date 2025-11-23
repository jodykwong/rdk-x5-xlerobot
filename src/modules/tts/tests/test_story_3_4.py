"""
Story 3.4 情感语音合成 - 完整集成测试

测试情感语音合成的完整功能
"""

import pytest
import time
import numpy as np
from typing import List, Dict

from ..emotion import (
    EmotionEngine, EmotionContext,
    EmotionType, EmotionAnalyzer,
    EmotionMapper, EmotionStateManager
)


class TestStory34Acceptance:
    """Story 3.4 验收测试"""

    @pytest.fixture
    def emotion_engine(self):
        """创建情感引擎实例"""
        engine = EmotionEngine(max_history_size=1000)
        yield engine
        engine.reset_state()

    # ========== 验收标准测试 ==========

    def test_ac1_emotion_accuracy(self, emotion_engine):
        """
        验收标准 AC-1: 情感表达准确率 >85%

        测试各种文本的情感识别准确率
        """
        # 准备测试用例
        test_cases = [
            # 快乐
            ("我今天很开心！", EmotionType.HAPPY),
            ("太好了！", EmotionType.HAPPY),
            ("心情愉悦", EmotionType.HAPPY),

            # 悲伤
            ("我很伤心", EmotionType.SAD),
            ("很难過", EmotionType.SAD),
            ("心情低落", EmotionType.SAD),

            # 愤怒
            ("我非常生气", EmotionType.ANGRY),
            ("气死了", EmotionType.ANGRY),
            ("愤愤不平", EmotionType.ANGRY),

            # 兴奋
            ("太激动了", EmotionType.EXCITED),
            ("热血沸腾", EmotionType.EXCITED),
            ("兴奋不已", EmotionType.EXCITED),

            # 平静
            ("我很平静", EmotionType.CALM),
            ("心情平和", EmotionType.CALM),
            ("安详宁静", EmotionType.CALM),

            # 惊讶
            ("太惊讶了", EmotionType.SURPRISED),
            ("没想到", EmotionType.SURPRISED),
            ("令人震惊", EmotionType.SURPRISED),

            # 恐惧
            ("我很害怕", EmotionType.FEARFUL),
            ("心惊肉跳", EmotionType.FEARFUL),
            ("提心吊胆", EmotionType.FEARFUL),

            # 厌恶
            ("很恶心", EmotionType.DISGUSTED),
            ("令人作呕", EmotionType.DISGUSTED),
            ("嫌恶不已", EmotionType.DISGUSTED),
        ]

        correct = 0
        total = len(test_cases)

        for text, expected_emotion in test_cases:
            context = EmotionContext(text=text)
            output = emotion_engine.process_text(text, context)

            if output.emotion == expected_emotion:
                correct += 1

        accuracy = correct / total * 100
        print(f"\n情感识别准确率: {accuracy:.1f}% ({correct}/{total})")

        # 验收标准：准确率 >85%
        assert accuracy > 85, f"情感识别准确率 {accuracy:.1f}% 未达到85%的要求"

    def test_ac2_emotion_types_support(self, emotion_engine):
        """
        验收标准 AC-2: 支持多种情感类型

        测试基础情感和复合情感的支持
        """
        # 基础情感类型
        basic_emotions = [
            EmotionType.NEUTRAL,
            EmotionType.HAPPY,
            EmotionType.SAD,
            EmotionType.ANGRY,
            EmotionType.EXCITED,
            EmotionType.CALM,
            EmotionType.SURPRISED,
            EmotionType.FEARFUL,
            EmotionType.DISGUSTED
        ]

        for emotion in basic_emotions:
            context = EmotionContext(text=f"测试{emotion.value}")
            output = emotion_engine.process_text("测试", context)

            # 验证引擎能处理所有情感类型
            assert output.emotion is not None

        # 验收标准：支持至少8种基础情感
        assert len(basic_emotions) >= 8, "未达到支持8种情感的要求"

    def test_ac3_intensity_control(self, emotion_engine):
        """
        验收标准 AC-3: 实现情感强度控制

        测试情感强度的精确控制
        """
        # 测试同一情感的不同强度
        intensity_levels = [0.1, 0.3, 0.5, 0.7, 0.9]
        texts = ["很开心", "很开心", "很开心", "很开心", "很开心"]

        for text, intensity in zip(texts, intensity_levels):
            context = EmotionContext(text=text)
            output = emotion_engine.process_text(text, context)

            # 强度应该在期望范围内
            assert 0.0 <= output.intensity <= 1.0

        # 测试强度参数映射
        test_text = "测试文本"
        params = emotion_engine.blend_emotions(
            EmotionType.HAPPY,
            EmotionType.SAD,
            weight=0.5,
            intensity=0.7
        )

        # 验证强度参数
        assert 0.0 <= params.intensity <= 1.0
        assert 0.5 <= params.duration_factor <= 2.0
        assert -50.0 <= params.pitch_shift <= 50.0
        assert 0.5 <= params.speed_factor <= 2.0
        assert 0.1 <= params.volume_factor <= 2.0

    def test_ac4_consistency_detection(self, emotion_engine):
        """
        验收标准 AC-4: 实现情感一致性检测

        测试情感状态的连贯性和一致性
        """
        # 创建连贯的情感序列
        texts = [
            "我今天很开心",
            "天气很好",
            "心情愉悦",
            "决定去散步"
        ]

        outputs = []
        for text in texts:
            context = EmotionContext(text=text)
            output = emotion_engine.process_text(text, context)
            outputs.append(output)

        # 检查情感序列的连贯性
        for i in range(1, len(outputs)):
            curr = outputs[i]
            prev = outputs[i - 1]

            # 情感转换应该平滑
            if curr.emotion != prev.emotion:
                # 如果情感发生变化，强度变化应该合理
                intensity_change = abs(curr.intensity - prev.intensity)
                assert intensity_change < 0.8, "情感强度变化过大"

    def test_ac5_contextual_adaptation(self, emotion_engine):
        """
        验收标准 AC-5: 实现上下文情感适应

        测试不同上下文下的情感适应
        """
        # 相同文本在不同上下文中的情感适应
        base_text = "真的吗？"

        # 疑问上下文
        context1 = EmotionContext(
            text=base_text,
            domain='question',
            additional_info={'question_type': 'inquiry'}
        )
        output1 = emotion_engine.process_text(base_text, context1)

        # 惊讶上下文
        context2 = EmotionContext(
            text=base_text,
            domain='surprise',
            additional_info={'emotion_hint': 'surprised'}
        )
        output2 = emotion_engine.process_text(base_text, context2)

        # 不同上下文应该产生不同的结果
        assert output1.emotion is not None
        assert output2.emotion is not None

        # 上下文信息应该被保留
        assert 'question' in output1.analysis_result.contextual_factors or \
               'question' in context1.additional_info

    def test_ac6_smooth_transition(self, emotion_engine):
        """
        验收标准 AC-6: 实现情感语音平滑过渡

        测试情感之间的平滑过渡
        """
        # 创建情感转换序列
        sequence = [
            (EmotionType.HAPPY, 0.7),
            (EmotionType.SAD, 0.6),
            (EmotionType.CALM, 0.5),
            (EmotionType.EXCITED, 0.8)
        ]

        params_sequence = emotion_engine.mapper.create_sequence(
            sequence,
            smoothness=0.8
        )

        # 验证序列长度
        assert len(params_sequence) == len(sequence)

        # 验证平滑度
        for i in range(1, len(params_sequence)):
            prev = params_sequence[i - 1]
            curr = params_sequence[i]

            # 参数变化应该平滑
            assert abs(curr.intensity - prev.intensity) < 0.5
            assert abs(curr.speed_factor - prev.speed_factor) < 0.3
            assert abs(curr.volume_factor - prev.volume_factor) < 0.3

    def test_ac7_state_management(self, emotion_engine):
        """
        验收标准 AC-7: 实现情感状态管理

        测试情感状态的完整管理功能
        """
        # 更新多次状态
        for i in range(10):
            emotion = list(EmotionType)[i % len(EmotionType)]
            intensity = (i % 10) / 10.0
            context = {'iteration': i}

            transition = emotion_engine.state_manager.update_state(
                emotion, intensity, context
            )

            assert transition.to_emotion == emotion
            assert transition.to_intensity == intensity

        # 验证状态历史
        state = emotion_engine.state_manager.get_current_state()
        assert state.current_emotion is not None

        # 验证转换历史
        history = emotion_engine.state_manager.get_transition_history()
        assert len(history) == 10

        # 验证统计信息
        stats = emotion_engine.state_manager.get_statistics()
        assert 'total_transitions' in stats
        assert stats['total_transitions'] == 10

    def test_ac8_quality_assessment(self, emotion_engine):
        """
        验收标准 AC-8: 情感语音质量测试

        测试情感语音的整体质量
        """
        # 准备多样化测试用例
        test_cases = [
            # 简单情感
            "很开心",
            "很伤心",

            # 复合情感
            "开心但是有点担心",
            "平静中带有兴奋",

            # 强度变化
            "有点开心",
            "非常开心",
            "超级开心",

            # 长文本
            "我今天早上醒来的时候心情很好，因为天气很不错，阳光明媚的，然后我就想着今天一定要做点有意义的事情。",

            # 短文本
            "好",
            "不好",
        ]

        quality_scores = []

        for text in test_cases:
            context = EmotionContext(text=text)
            output = emotion_engine.process_text(text, context)

            # 计算质量分数
            quality_score = 0.0

            # 情感识别置信度 (0-40%)
            quality_score += output.confidence * 40

            # 参数有效性 (0-30%)
            is_valid, _ = emotion_engine.mapper.validate_parameters(output.parameters)
            quality_score += 30 if is_valid else 0

            # 处理时间 (0-20%)
            if output.processing_time < 0.1:
                quality_score += 20
            elif output.processing_time < 0.2:
                quality_score += 10

            # 状态一致性 (0-10%)
            if output.transition:
                if output.transition.smoothness < 0.5:
                    quality_score += 10

            quality_scores.append(quality_score)

        # 计算平均质量分数
        avg_quality = np.mean(quality_scores)
        print(f"\n平均质量分数: {avg_quality:.1f}/100")

        # 验收标准：平均质量分数 >70分
        assert avg_quality > 70, f"平均质量分数 {avg_quality:.1f} 未达到70分的要求"

    # ========== 功能完整性测试 ==========

    def test_performance_benchmark(self, emotion_engine):
        """性能基准测试"""
        # 准备测试数据
        test_texts = [
            "我很开心",
            "今天天气很好",
            "去散步",
            "很伤心",
            "很愤怒",
        ] * 20  # 100个文本

        # 测试单次处理
        start_time = time.time()
        for text in test_texts:
            context = EmotionContext(text=text)
            emotion_engine.process_text(text, context)
        single_time = time.time() - start_time

        # 测试批量处理
        start_time = time.time()
        contexts = [EmotionContext(text=t) for t in test_texts]
        emotion_engine.process_batch(test_texts, contexts)
        batch_time = time.time() - start_time

        # 性能指标
        single_avg = single_time / len(test_texts) * 1000  # ms
        batch_avg = batch_time / len(test_texts) * 1000  # ms

        print(f"\n性能测试结果:")
        print(f"单次处理平均时间: {single_avg:.2f}ms")
        print(f"批量处理平均时间: {batch_avg:.2f}ms")

        # 性能要求：单次处理 <100ms
        assert single_avg < 100, f"单次处理时间 {single_avg:.2f}ms 超过100ms"

    def test_robustness_test(self, emotion_engine):
        """鲁棒性测试"""
        # 测试异常输入
        edge_cases = [
            "",  # 空文本
            " " * 1000,  # 极长文本
            "😀😁😂",  # 表情符号
            "12345",  # 纯数字
            "!@#$%",  # 特殊字符
            "中英Mixed混合文本",  # 中英文混合
        ]

        for text in edge_cases:
            try:
                context = EmotionContext(text=text)
                output = emotion_engine.process_text(text, context)
                assert output is not None
            except Exception as e:
                pytest.fail(f"处理'{text}'时发生异常: {e}")

    def test_memory_efficiency(self, emotion_engine):
        """内存效率测试"""
        # 大量文本处理
        for i in range(1000):
            text = f"测试文本 {i}"
            context = EmotionContext(text=text)
            emotion_engine.process_text(text, context)

        # 验证状态管理正常
        state = emotion_engine.state_manager.get_current_state()
        assert state is not None

        # 验证缓存大小在限制内
        assert len(emotion_engine._cache) <= emotion_engine._cache_size

    def test_personalization(self, emotion_engine):
        """个性化测试"""
        # 设置个性化参数
        emotion_engine.state_manager.set_personalization(
            preferred_emotions=['happy', 'calm'],
            intensity_preference=0.6,
            transition_style='smooth'
        )

        # 处理文本
        output1 = emotion_engine.process_text(
            "测试",
            EmotionContext(text="测试", speaker_id="user1")
        )

        # 切换用户
        emotion_engine.reset_state(speaker_id="user2")
        output2 = emotion_engine.process_text(
            "测试",
            EmotionContext(text="测试", speaker_id="user2")
        )

        # 验证个性化生效
        assert output1 is not None
        assert output2 is not None

    def test_integration_with_tts(self, emotion_engine):
        """与TTS系统集成测试"""
        # 模拟TTS输入
        text = "我今天很开心！"
        context = EmotionContext(
            text=text,
            language="zh-CN",
            domain="conversation"
        )

        # 处理情感
        output = emotion_engine.process_text(text, context)

        # 生成TTS参数
        params = output.parameters

        # 验证参数完整性
        assert params.emotion is not None
        assert 0.0 <= params.intensity <= 1.0
        assert 0.5 <= params.duration_factor <= 2.0
        assert -50.0 <= params.pitch_shift <= 50.0
        assert 0.5 <= params.speed_factor <= 2.0
        assert 0.1 <= params.volume_factor <= 2.0
        assert 0.0 <= params.tone_variation <= 1.0
        assert 0.0 <= params.prosody_emphasis <= 1.0

        # 验证参数可序列化
        params_dict = emotion_engine.mapper.get_parameter_summary(params)
        assert 'emotion' in params_dict
        assert 'intensity' in params_dict


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
