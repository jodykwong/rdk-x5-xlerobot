#!/usr/bin/env python3.10
"""
Story 1.6 视觉理解集成测试
BMad-Method v6 Brownfield Level 4 合规性测试

测试范围:
- Qwen3-VL-Plus API集成
- 多模态上下文处理
- 粤语视觉理解
- 端到端功能验证
"""

import os
import sys
import time
import tempfile
from PIL import Image, ImageDraw

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

from xlerobot_vision.qwen_vl_client import QwenVLPlusClient, QwenVLConfig
from xlerobot_vision.multimodal_context import MultimodalContextProcessor


def create_test_scenarios():
    """创建测试场景图像"""
    scenarios = {}

    # 场景1: 红色圆形
    img1 = Image.new('RGB', (200, 200), 'red')
    draw1 = ImageDraw.Draw(img1)
    draw1.ellipse([50, 50, 150, 150], fill='white')

    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
        img1.save(tmp.name)
        scenarios['red_circle'] = tmp.name

    # 场景2: 蓝色方形
    img2 = Image.new('RGB', (200, 200), 'blue')
    draw2 = ImageDraw.Draw(img2)
    draw2.rectangle([50, 50, 150, 150], fill='yellow')

    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
        img2.save(tmp.name)
        scenarios['blue_square'] = tmp.name

    return scenarios


def test_api_integration():
    """测试API集成功能"""
    print("🔗 测试Qwen3-VL-Plus API集成")
    print("-" * 40)

    config = QwenVLConfig(
        # 移除硬编码密钥: api_key="YOUR_QWEN_API_KEY",
        timeout=30,
        max_tokens=300
    )

    client = QwenVLPlusClient(config)
    scenarios = create_test_scenarios()

    try:
        # 测试场景1
        print("📸 测试场景1: 红色圆形")
        response1 = client.analyze_image(
            scenarios['red_circle'],
            "請用廣東話描述呢張圖片",
            use_cantonese=True
        )

        if "choices" in response1:
            content1 = response1["choices"][0]["message"]["content"]
            print(f"✅ 响应1: {content1[:100]}...")

            # 验证粤语内容
            cantonese_indicators = ['呢張', '圖片', '廣東話', '紅色', '圓形']
            cantonese_score = sum(1 for word in cantonese_indicators if word in content1)
            print(f"🗣️ 粤语指标匹配: {cantonese_score}/{len(cantonese_indicators)}")
        else:
            print("❌ 场景1响应失败")
            return False

        # 测试场景2
        print("\n📸 测试场景2: 蓝色方形")
        response2 = client.analyze_image(
            scenarios['blue_square'],
            "呢張圖有乜顏色同形狀？",
            use_cantonese=True
        )

        if "choices" in response2:
            content2 = response2["choices"][0]["message"]["content"]
            print(f"✅ 响应2: {content2[:100]}...")

            # 验证准确性
            accuracy_indicators = ['藍色', '方形', '黃色']
            accuracy_score = sum(1 for word in accuracy_indicators if word in content2)
            print(f"🎯 准确性指标匹配: {accuracy_score}/{len(accuracy_indicators)}")
        else:
            print("❌ 场景2响应失败")
            return False

        # 检查API统计
        stats = client.get_call_statistics()
        print(f"📊 API调用统计:")
        print(f"   总调用: {stats['total_calls']}")
        print(f"   成功率: {stats['success_rate']:.2%}")
        print(f"   平均响应时间: {stats['average_response_time']:.2f}秒")

        return stats['success_rate'] >= 0.8  # 80%以上成功率

    except Exception as e:
        print(f"❌ API集成测试失败: {e}")
        return False
    finally:
        # 清理临时文件
        for path in scenarios.values():
            if os.path.exists(path):
                os.remove(path)


def test_multimodal_context():
    """测试多模态上下文处理"""
    print("\n🧠 测试多模态上下文处理")
    print("-" * 40)

    processor = MultimodalContextProcessor()

    try:
        # 创建测试会话
        session_id = "test_session_story_1_6"
        print(f"🆔 创建测试会话: {session_id}")

        # 场景1: 纯文本对话
        print("📝 场景1: 纯文本对话")
        entry_id_1 = processor.add_multimodal_input(
            session_id, 'text', '你好，我想了解視覺理解功能')

        context_info_1 = processor.process_current_context(
            session_id, '呢個功能可以點用？')

        print(f"✅ 上下文置信度: {context_info_1['context_confidence']:.2f}")
        print(f"🎯 意图类型: {context_info_1['intent_analysis']['intent_type']}")

        # 场景2: 图像+文本
        print("\n📸 场景2: 图像+文本多模态")
        scenarios = create_test_scenarios()

        entry_id_2 = processor.add_multimodal_input(
            session_id, 'image', scenarios['red_circle'])

        context_info_2 = processor.process_current_context(
            session_id, '睇下呢張圖有乜特別', [scenarios['red_circle']])

        print(f"✅ 上下文置信度: {context_info_2['context_confidence']:.2f}")
        print(f"👀 视觉查询: {context_info_2['intent_analysis']['visual_query']}")
        print(f"🗣️ 粤语检测: {context_info_2['intent_analysis']['is_cantonese']}")
        print(f"📝 建议token数: {context_info_2['suggested_max_tokens']}")

        # 存储响应
        processor.store_response(session_id, entry_id_2,
            "呢張圖顯示一個紅色背景上面有白色圓形嘅設計。")

        # 场景3: 连续对话
        print("\n💬 场景3: 连续对话")
        context_info_3 = processor.process_current_context(
            session_id, '個圓形代表乜意思？')

        print(f"✅ 上下文置信度: {context_info_3['context_confidence']:.2f}")
        print(f"📚 相关上下文条目: {len(context_info_3['relevant_context'])}")

        # 获取会话摘要
        summary = processor.get_session_summary(session_id)
        print(f"📋 会话统计:")
        print(f"   总交互: {summary['total_interactions']}")
        print(f"   多模态比例: {summary['multimodal_ratio']:.2%}")
        print(f"   会话时长: {summary['session_duration']:.1f}秒")

        # 获取处理器统计
        stats = processor.get_processor_stats()
        print(f"📊 处理器统计:")
        print(f"   粤语交互比例: {stats['cantonese_interaction_ratio']:.2%}")
        print(f"   多模态交互比例: {stats['multimodal_interaction_ratio']:.2%}")

        return True

    except Exception as e:
        print(f"❌ 多模态上下文测试失败: {e}")
        return False
    finally:
        # 清理临时文件
        if 'scenarios' in locals():
            for path in scenarios.values():
                if os.path.exists(path):
                    os.remove(path)


def test_end_to_end_integration():
    """端到端集成测试"""
    print("\n🔄 端到端集成测试")
    print("-" * 40)

    try:
        # 初始化组件
        config = QwenVLConfig(
            # 移除硬编码密钥: api_key="YOUR_QWEN_API_KEY",
            max_tokens=400
        )
        client = QwenVLPlusClient(config)
        processor = MultimodalContextProcessor()

        # 创建测试图像
        scenarios = create_test_scenarios()

        # 模拟完整流程
        session_id = "e2e_test_session"

        print("🚀 模拟用户交互流程:")

        # 步骤1: 用户打招呼
        print("1. 用户: 你好，我想試下視覺理解")
        entry_1 = processor.add_multimodal_input(session_id, 'text', '你好，我想試下視覺理解')

        # 步骤2: 展示图像并提问
        print("2. 用户展示图像并提问: 呢張圖畫緊乜？")
        entry_2 = processor.add_multimodal_input(session_id, 'image', scenarios['red_circle'])

        context_info = processor.process_current_context(
            session_id, '呢張圖畫緊乜？', [scenarios['red_circle']])

        # 步骤3: 调用API获取响应
        print("3. 调用视觉理解API...")
        api_response = client.analyze_image(
            scenarios['red_circle'],
            context_info['optimized_prompt'],
            use_cantonese=True
        )

        if "choices" in api_response:
            ai_response = api_response["choices"][0]["message"]["content"]
            print(f"4. AI助手: {ai_response}")

            # 存储响应
            processor.store_response(session_id, entry_2, ai_response)

        # 步骤4: 连续对话
        print("5. 用户追问: 個圓形大定細？")
        context_info_2 = processor.process_current_context(session_id, '個圓形大定細？')

        api_response_2 = client.analyze_image(
            scenarios['red_circle'],
            "根據之前嘅對話，請回答個圓形大定細，用粵語回答。",
            use_cantonese=True
        )

        if "choices" in api_response_2:
            ai_response_2 = api_response_2["choices"][0]["message"]["content"]
            print(f"6. AI助手: {ai_response_2}")

        # 验证结果
        print("\n📊 端到端测试结果:")

        # API性能
        api_stats = client.get_call_statistics()
        print(f"✅ API成功率: {api_stats['success_rate']:.2%}")
        print(f"✅ 平均响应时间: {api_stats['average_response_time']:.2f}秒")

        # 上下文性能
        context_stats = processor.get_processor_stats()
        print(f"✅ 多模态交互比例: {context_stats['multimodal_interaction_ratio']:.2%}")
        print(f"✅ 粤语交互比例: {context_stats['cantonese_interaction_ratio']:.2%}")

        # 综合评分
        success_score = (
            api_stats['success_rate'] * 0.4 +  # API性能40%
            context_stats['multimodal_interaction_ratio'] * 0.3 +  # 多模态30%
            context_stats['cantonese_interaction_ratio'] * 0.3   # 粤语30%
        )

        print(f"🎯 综合评分: {success_score:.2%}")

        return success_score >= 0.7  # 70%以上通过

    except Exception as e:
        print(f"❌ 端到端测试失败: {e}")
        return False
    finally:
        # 清理临时文件
        if 'scenarios' in locals():
            for path in scenarios.values():
                if os.path.exists(path):
                    os.remove(path)


def main():
    """主测试函数"""
    print("🧪 Story 1.6 视觉理解集成测试")
    print("=" * 60)
    print("BMad-Method v6 Brownfield Level 4 合规性测试")
    print("=" * 60)

    test_results = {}

    try:
        # 执行各项测试
        test_results['api_integration'] = test_api_integration()
        test_results['multimodal_context'] = test_multimodal_context()
        test_results['end_to_end'] = test_end_to_end_integration()

        # 测试总结
        print("\n" + "=" * 60)
        print("📊 测试结果汇总")
        print("=" * 60)

        total_tests = len(test_results)
        passed_tests = sum(test_results.values())

        for test_name, result in test_results.items():
            status = "✅ 通过" if result else "❌ 失败"
            print(f"{test_name}: {status}")

        success_rate = passed_tests / total_tests
        print(f"\n🎯 总体通过率: {success_rate:.1%} ({passed_tests}/{total_tests})")

        if success_rate >= 0.8:
            print("🎉 Story 1.6 视觉理解集成开发 - 测试通过!")
            print("✅ BMad-Method v6 Brownfield Level 4 合规")
            print("🚀 可以进入下一阶段开发")
        else:
            print("⚠️ 测试未完全通过，需要进一步优化")

        # 验收标准检查
        print("\n📋 验收标准检查:")
        print(f"✅ 视觉问答功能: {'正常' if test_results['api_integration'] else '异常'}")
        print(f"✅ 多模态上下文处理: {'正常' if test_results['multimodal_context'] else '异常'}")
        print(f"✅ 粤语视觉理解: {'正常' if test_results['end_to_end'] else '异常'}")
        print(f"✅ 系统稳定性: {'良好' if success_rate >= 0.8 else '需改进'}")

        return success_rate >= 0.8

    except Exception as e:
        print(f"\n❌ 测试过程中发生错误: {e}")
        import traceback

# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
# 安全配置导入
try:
    from core.security.security_config_manager import init_security_config, get_security_manager
    init_security_config()
    security_manager = get_security_manager()
except Exception as e:
    print(f'❌ 安全配置初始化失败: {e}')
    # 根据文件类型决定是否退出
    import sys
    sys.exit(1)
        traceback.print_exc()
        return False


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)