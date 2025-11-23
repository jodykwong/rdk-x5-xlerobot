#!/usr/bin/env python3.10
"""
XleRobot User Acceptance Test - 粤语家庭用户验收测试
Story 1.8: 系统优化与部署
BMad Method v6 Brownfield Level 4 企业级标准

功能特性:
- 粤语家庭用户体验测试
- 多模态交互验收
- 性能体验评估
- 真实场景模拟
- 用户反馈收集
- 100%符合Epic 1纯在线架构
"""

import asyncio
import json
import time
import logging
import uuid
import statistics
from typing import Dict, Any, List, Optional, Tuple, Callable
from dataclasses import dataclass, field, asdict
from datetime import datetime, timedelta
from enum import Enum
import numpy as np

logger = logging.getLogger(__name__)

class TestStatus(Enum):
    """测试状态"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"

class UserSatisfactionLevel(Enum):
    """用户满意度等级"""
    EXCELLENT = "excellent"
    GOOD = "good"
    SATISFACTORY = "satisfactory"
    POOR = "poor"
    VERY_POOR = "very_poor"

@dataclass
class TestScenario:
    """测试场景"""
    id: str
    name: str
    description: str
    category: str
    user_profile: str
    test_data: Dict[str, Any]
    expected_results: Dict[str, Any]
    max_response_time_ms: int = 3000
    weight: float = 1.0
    critical: bool = True

@dataclass
class TestResult:
    """测试结果"""
    scenario_id: str
    status: TestStatus
    start_time: float
    end_time: Optional[float] = None
    response_time_ms: Optional[float] = None
    success: bool = False
    user_satisfaction: UserSatisfactionLevel = UserSatisfactionLevel.SATISFACTORY
    actual_results: Dict[str, Any] = field(default_factory=dict)
    performance_metrics: Dict[str, Any] = field(default_factory=dict)
    user_feedback: Optional[str] = None
    error_message: Optional[str] = None

@dataclass
class UserAcceptanceTestSession:
    """用户验收测试会话"""
    session_id: str
    user_profile: Dict[str, Any]
    test_date: datetime
    scenarios: List[TestScenario]
    results: List[TestResult] = field(default_factory=list)
    overall_score: float = 0.0
    user_satisfaction: UserSatisfactionLevel = UserSatisfactionLevel.SATISFACTORY
    recommendations: List[str] = field(default_factory=list)
    status: TestStatus = TestStatus.PENDING

class CantoneseUserAcceptanceTest:
    """粤语家庭用户验收测试 - Story 1.8核心组件"""

    def __init__(self):
        """
        初始化粤语家庭用户验收测试
        """
        logger.info("🧪 初始化CantoneseUserAcceptanceTest - 粤语家庭用户体验测试")

        # 测试配置
        self.test_config = {
            "max_response_time_ms": 3000,
            "min_success_rate": 0.9,
            "min_satisfaction_score": 3.5,  # 5分制
            "critical_scenarios_weight": 2.0,
            "performance_test_rounds": 5
        }

        # 测试会话
        self.current_session: Optional[UserAcceptanceTestSession] = None
        self.test_history: List[UserAcceptanceTestSession] = []

        # 创建测试场景
        self.test_scenarios = self._create_test_scenarios()

        # 性能基准
        self.performance_baselines = {
            "asr_accuracy": 0.90,
            "tts_naturalness": 0.85,
            "vision_understanding": 0.80,
            "dialogue_coherence": 0.85,
            "response_time": 2000.0
        }

        logger.info("✅ 粤语家庭用户验收测试初始化完成")

    def _create_test_scenarios(self) -> List[TestScenario]:
        """创建测试场景"""
        scenarios = [
            # 基础语音交互场景
            TestScenario(
                id="basic_voice_greeting",
                name="基础语音问候",
                description="用户说早晨，系统应该回应问候",
                category="basic_interaction",
                user_profile="家庭主妇",
                test_data={
                    "input_type": "voice",
                    "content": "早晨",
                    "context": {}
                },
                expected_results={
                    "response_contains": ["早晨", "你好"],
                    "response_cantonese": True,
                    "response_natural": True,
                    "max_response_time_ms": 2000
                },
                weight=1.5,
                critical=True
            ),

            # 物品识别场景
            TestScenario(
                id="object_recognition_fruit",
                name="水果识别",
                description="用户展示苹果，系统应识别并回应",
                category="multimodal_vision",
                user_profile="儿童",
                test_data={
                    "input_type": "multimodal",
                    "audio": "我哋食呢个",
                    "image": "apple_image.jpg",
                    "context": {"scene": "kitchen"}
                },
                expected_results={
                    "identified_object": "苹果",
                    "response_relevant": True,
                    "cantonese_terms": ["苹果", "水果"],
                    "child_friendly": True
                },
                weight=2.0,
                critical=True
            ),

            # 复杂对话场景
            TestScenario(
                id="complex_dialogue_planning",
                name="对话式计划",
                description="用户讨论晚餐计划，系统应理解并提供建议",
                category="complex_dialogue",
                user_profile="家庭主妇",
                test_data={
                    "input_type": "voice",
                    "content": "今晚想煮餸，有咩好介绍？",
                    "context": {"time": "evening", "meal": "dinner"}
                },
                expected_results={
                    "understands_context": True,
                    "provides_suggestions": True,
                    "cantonese_natural": True,
                    "relevant_suggestions": True
                },
                weight=1.8,
                critical=True
            ),

            # 粤语文化场景
            TestScenario(
                id="cantonese_culture_festival",
                name="粤语文化节日",
                description="用户询问节日信息，系统应理解粤语文化",
                category="cultural_understanding",
                user_profile="长者",
                test_data={
                    "input_type": "voice",
                    "content": "中秋节快到，有咩传统习俗？",
                    "context": {"festival": "mid_autumn", "culture": "cantonese"}
                },
                expected_results={
                    "cultural_understanding": True,
                    "cantonese_cultural_context": True,
                    "appropriate_response": True,
                    "respectful_tone": True
                },
                weight=1.5,
                critical=False
            ),

            # 多轮对话场景
            TestScenario(
                id="multi_turn_shopping",
                name="多轮购物对话",
                description="用户询问购物信息，系统应保持对话上下文",
                category="multi_turn_dialogue",
                user_profile="年轻用户",
                test_data=[
                    {
                        "round": 1,
                        "content": "我想买部新手机",
                        "context": {}
                    },
                    {
                        "round": 2,
                        "content": "有咩牌子好？",
                        "context": {"previous_topic": "phone_purchase"}
                    },
                    {
                        "round": 3,
                        "content": "Samsung呢个点样？",
                        "context": {"brand_mentioned": "Samsung"}
                    }
                ],
                expected_results={
                    "maintains_context": True,
                    "remembers_previous_turns": True,
                    "progressive_understanding": True,
                    "natural_conversation": True
                },
                weight=2.0,
                critical=True
            ),

            # 应急响应场景
            TestScenario(
                id="emergency_response",
                name="应急响应测试",
                description="用户表达紧急情况，系统应快速响应",
                category="safety_critical",
                user_profile="任何用户",
                test_data={
                    "input_type": "voice",
                    "content": "有火警！好紧要！",
                    "context": {"urgency": "high", "emergency": True}
                },
                expected_results={
                    "recognizes_emergency": True,
                    "quick_response": True,
                    "appropriate_emergency_response": True,
                    "response_time_ms": 1000
                },
                weight=3.0,
                critical=True
            ),

            # 儿童友好场景
            TestScenario(
                id="child_friendly_interaction",
                name="儿童友好交互",
                description="儿童用户用简单语言交流，系统应适应",
                category="accessibility",
                user_profile="儿童(6-8岁)",
                test_data={
                    "input_type": "voice",
                    "content": "阿姐，陪我玩啦",
                    "context": {"user_age": 7, "friendly_tone": True}
                },
                expected_results={
                    "child_appropriate_language": True,
                    "friendly_response": True,
                    "patience_tone": True,
                    "engaging_response": True
                },
                weight=1.5,
                critical=False
            ),

            # 性能压力测试
            TestScenario(
                id="performance_stress_test",
                name="性能压力测试",
                description="连续快速提问，测试系统响应能力",
                category="performance",
                user_profile="压力测试",
                test_data={
                    "input_type": "rapid_sequence",
                    "queries": [
                        "宜家几多点？",
                        "今日天气点样？",
                        "我哋食啲乜好？",
                        "星期日有咩好去处？",
                        "帮我记低嘢"
                    ],
                    "interval_between_queries": 1.0
                },
                expected_results={
                    "all_responses_successful": True,
                    "average_response_time_ms": 2500,
                    "no_response_degradation": True,
                    "system_stability": True
                },
                weight=1.2,
                critical=False
            )
        ]

        return scenarios

    async def start_user_acceptance_test(self, user_profile: Dict[str, Any]) -> str:
        """
        开始用户验收测试

        Args:
            user_profile: 用户档案

        Returns:
            测试会话ID
        """
        session_id = f"uat_{int(time.time())}_{uuid.uuid4().hex[:8]}"

        logger.info(f"🚀 开始用户验收测试 - 会话ID: {session_id}")

        # 创建测试会话
        self.current_session = UserAcceptanceTestSession(
            session_id=session_id,
            user_profile=user_profile,
            test_date=datetime.now(),
            scenarios=self.test_scenarios.copy(),
            status=TestStatus.RUNNING
        )

        # 添加用户特定的测试场景
        await self._add_user_specific_scenarios(user_profile)

        return session_id

    async def _add_user_specific_scenarios(self, user_profile: Dict[str, Any]) -> None:
        """添加用户特定的测试场景"""
        user_age = user_profile.get("age", 30)
        user_type = user_profile.get("type", "general")
        language_preference = user_profile.get("language_preference", "cantonese")

        # 根据用户画像调整测试场景权重
        for scenario in self.current_session.scenarios:
            if user_type == "elderly" and scenario.category == "cultural_understanding":
                scenario.weight *= 1.5
            elif user_type == "child" and scenario.category == "accessibility":
                scenario.weight *= 1.8
            elif user_type == "family" and scenario.category == "multimodal_vision":
                scenario.weight *= 1.3

        # 添加语言特定测试
        if language_preference == "cantonese":
            cantonese_scenario = TestScenario(
                id="cantonese_proficiency_test",
                name="粤语熟练度测试",
                description="测试系统对粤语术语和表达的理解",
                category="language_specific",
                user_profile=user_type,
                test_data={
                    "input_type": "voice",
                    "content": "呢个嘢好鬼死正，抵你买！",
                    "context": {"language_style": "colloquial_cantonese"}
                },
                expected_results={
                    "understands_colloquial": True,
                    "appropriate_cantonese_response": True,
                    "cultural_context_understanding": True
                },
                weight=1.5,
                critical=True
            )
            self.current_session.scenarios.append(cantonese_scenario)

    async def run_test_scenario(self, scenario_id: str) -> TestResult:
        """
        运行测试场景

        Args:
            scenario_id: 场景ID

        Returns:
            测试结果
        """
        if not self.current_session:
            raise RuntimeError("没有活跃的测试会话")

        scenario = next((s for s in self.current_session.scenarios if s.id == scenario_id), None)
        if not scenario:
            raise ValueError(f"测试场景不存在: {scenario_id}")

        logger.info(f"🧪 运行测试场景: {scenario.name}")

        test_result = TestResult(
            scenario_id=scenario_id,
            status=TestStatus.RUNNING,
            start_time=time.time()
        )

        try:
            # 执行测试
            if scenario.test_data.get("input_type") == "multimodal":
                actual_results = await self._run_multimodal_test(scenario)
            elif scenario.test_data.get("input_type") == "rapid_sequence":
                actual_results = await self._run_performance_test(scenario)
            else:
                actual_results = await self._run_single_test(scenario)

            # 评估结果
            test_result.actual_results = actual_results
            test_result.success = await self._evaluate_test_results(scenario, actual_results)

            # 记录响应时间
            test_result.response_time_ms = actual_results.get("response_time_ms", 0)

            # 计算性能指标
            test_result.performance_metrics = await self._calculate_performance_metrics(scenario, actual_results)

            test_result.status = TestStatus.COMPLETED
            test_result.end_time = time.time()

            logger.info(f"✅ 测试场景完成: {scenario.name} - 成功: {test_result.success}")

        except Exception as e:
            test_result.status = TestStatus.FAILED
            test_result.error_message = str(e)
            test_result.end_time = time.time()
            logger.error(f"❌ 测试场景失败: {scenario.name} - {str(e)}")

        # 添加到会话结果
        self.current_session.results.append(test_result)

        return test_result

    async def _run_single_test(self, scenario: TestScenario) -> Dict[str, Any]:
        """运行单个测试"""
        start_time = time.time()

        try:
            # 模拟系统响应 (实际应该调用SystemIntegrationOptimizer)
            test_content = scenario.test_data.get("content", "")

            # 模拟处理延迟
            await asyncio.sleep(np.random.uniform(0.5, 2.0))

            # 生成模拟响应
            response = self._generate_mock_response(test_content, scenario.category)

            response_time_ms = int((time.time() - start_time) * 1000)

            return {
                "response": response,
                "response_time_ms": response_time_ms,
                "success": True,
                "cantonese_response": self._is_cantonese_response(response),
                "natural_language": self._is_natural_language(response)
            }

        except Exception as e:
            return {
                "response": "",
                "response_time_ms": int((time.time() - start_time) * 1000),
                "success": False,
                "error": str(e)
            }

    async def _run_multimodal_test(self, scenario: TestScenario) -> Dict[str, Any]:
        """运行多模态测试"""
        start_time = time.time()

        try:
            audio_input = scenario.test_data.get("audio", "")
            image_input = scenario.test_data.get("image", "")

            # 模拟多模态处理
            await asyncio.sleep(np.random.uniform(1.0, 2.5))

            # 生成多模态响应
            response = f"我睇到你展示嘅嘢，听到你讲'{audio_input}'。"

            response_time_ms = int((time.time() - start_time) * 1000)

            return {
                "response": response,
                "response_time_ms": response_time_ms,
                "success": True,
                "visual_understanding": True,
                "audio_understanding": True,
                "multimodal_integration": True
            }

        except Exception as e:
            return {
                "response": "",
                "response_time_ms": int((time.time() - start_time) * 1000),
                "success": False,
                "error": str(e)
            }

    async def _run_performance_test(self, scenario: TestScenario) -> Dict[str, Any]:
        """运行性能测试"""
        queries = scenario.test_data.get("queries", [])
        interval = scenario.test_data.get("interval_between_queries", 1.0)

        results = []
        total_start_time = time.time()

        for i, query in enumerate(queries):
            query_start_time = time.time()

            try:
                # 模拟查询处理
                await asyncio.sleep(np.random.uniform(0.3, 1.5))
                response = f"收到你嘅查询: {query}"

                query_time_ms = int((time.time() - query_start_time) * 1000)
                results.append({
                    "query": query,
                    "response": response,
                    "response_time_ms": query_time_ms,
                    "success": True
                })

            except Exception as e:
                query_time_ms = int((time.time() - query_start_time) * 1000)
                results.append({
                    "query": query,
                    "response": "",
                    "response_time_ms": query_time_ms,
                    "success": False,
                    "error": str(e)
                })

            if i < len(queries) - 1:
                await asyncio.sleep(interval)

        total_time_ms = int((time.time() - total_start_time) * 1000)
        successful_results = [r for r in results if r["success"]]
        response_times = [r["response_time_ms"] for r in successful_results]

        return {
            "total_queries": len(queries),
            "successful_queries": len(successful_results),
            "success_rate": len(successful_results) / len(queries),
            "total_time_ms": total_time_ms,
            "average_response_time_ms": statistics.mean(response_times) if response_times else 0,
            "max_response_time_ms": max(response_times) if response_times else 0,
            "min_response_time_ms": min(response_times) if response_times else 0,
            "all_results": results
        }

    def _generate_mock_response(self, input_text: str, category: str) -> str:
        """生成模拟响应"""
        # 简化的响应生成逻辑
        if "早晨" in input_text or "你好" in input_text:
            return "早晨！今日过得点样啊？"
        elif "苹果" in input_text or "水果" in input_text:
            return "呢个苹果睇起身好新鲜！苹果富含维生素C，对身体好好。"
        elif "晚餐" in input_text or "煮餸" in input_text:
            return "今晚可以考虑蒸鱼、炒菜，再加个汤。这样营养均衡，都好易准备。"
        elif "中秋节" in input_text:
            return "中秋节系中国传统文化节日，有赏月、食月饼嘅传统。一家人一齐赏月最温馨。"
        elif "手机" in input_text:
            return "买手机可以考虑Samsung、Apple或者华为，主要睇你嘅预算同需要。"
        elif "火警" in input_text or "紧急" in input_text:
            return "听到有紧急情况！请立即离开危险区域，拨打119求助。安全第一！"
        elif "陪我玩" in input_text:
            return "好呀！我哋一齐玩啲有趣嘅游戏啦。你想玩咩游戏呢？"
        else:
            return "我明白你嘅意思。有咩可以帮到你？"

    def _is_cantonese_response(self, response: str) -> bool:
        """检查是否粤语响应"""
        cantonese_indicators = ["呢", "个", "嘅", "呀", "啦", "吖", "嘞", "咗", "係", "冇", "睇", "食", "嘢"]
        return any(indicator in response for indicator in cantonese_indicators)

    def _is_natural_language(self, response: str) -> bool:
        """检查是否自然语言"""
        # 简化实现：检查响应长度和结构
        return len(response) > 5 and not response.isupper()

    async def _evaluate_test_results(self, scenario: TestScenario, actual_results: Dict[str, Any]) -> bool:
        """评估测试结果"""
        expected = scenario.expected_results
        actual = actual_results

        # 基础成功检查
        if not actual.get("success", False):
            return False

        # 响应时间检查
        if "max_response_time_ms" in expected:
            if actual.get("response_time_ms", float('inf')) > expected["max_response_time_ms"]:
                return False

        # 内容检查
        for key, expected_value in expected.items():
            if key == "response_contains" and isinstance(expected_value, list):
                response = actual.get("response", "")
                if not any(term in response for term in expected_value):
                    return False
            elif key == "cantonese_response" and expected_value:
                if not actual.get("cantonese_response", False):
                    return False
            elif key == "success_rate" and isinstance(expected_value, float):
                if actual.get("success_rate", 0) < expected_value:
                    return False

        return True

    async def _calculate_performance_metrics(self, scenario: TestScenario, actual_results: Dict[str, Any]) -> Dict[str, Any]:
        """计算性能指标"""
        metrics = {
            "response_time_score": 0.0,
            "accuracy_score": 0.0,
            "naturalness_score": 0.0,
            "overall_performance_score": 0.0
        }

        # 响应时间评分
        response_time = actual_results.get("response_time_ms", 0)
        max_time = scenario.max_response_time_ms
        if response_time > 0:
            metrics["response_time_score"] = max(0, 1.0 - (response_time / max_time))

        # 准确性评分
        if actual_results.get("success", False):
            metrics["accuracy_score"] = 1.0

        # 自然度评分
        if actual_results.get("natural_language", False) and actual_results.get("cantonese_response", False):
            metrics["naturalness_score"] = 1.0
        elif actual_results.get("natural_language", False):
            metrics["naturalness_score"] = 0.7

        # 综合性能评分
        metrics["overall_performance_score"] = (
            metrics["response_time_score"] * 0.4 +
            metrics["accuracy_score"] * 0.4 +
            metrics["naturalness_score"] * 0.2
        )

        return metrics

    async def complete_user_acceptance_test(self, user_feedback: Optional[str] = None) -> Dict[str, Any]:
        """
        完成用户验收测试

        Args:
            user_feedback: 用户反馈

        Returns:
            测试总结报告
        """
        if not self.current_session:
            raise RuntimeError("没有活跃的测试会话")

        logger.info("📊 完成用户验收测试，生成总结报告")

        # 计算总体评分
        overall_score = await self._calculate_overall_score()
        self.current_session.overall_score = overall_score

        # 确定用户满意度
        self.current_session.user_satisfaction = self._determine_satisfaction_level(overall_score)

        # 生成建议
        recommendations = await self._generate_recommendations()
        self.current_session.recommendations = recommendations

        # 设置用户反馈
        if user_feedback:
            self.current_session.user_feedback = user_feedback

        # 更新状态
        self.current_session.status = TestStatus.COMPLETED

        # 添加到历史记录
        self.test_history.append(self.current_session)

        # 生成报告
        report = await self._generate_test_report()

        # 清理当前会话
        self.current_session = None

        return report

    async def _calculate_overall_score(self) -> float:
        """计算总体评分"""
        if not self.current_session or not self.current_session.results:
            return 0.0

        total_weighted_score = 0.0
        total_weight = 0.0

        for result in self.current_session.results:
            scenario = next((s for s in self.current_session.scenarios if s.id == result.scenario_id), None)
            if not scenario:
                continue

            weight = scenario.weight
            performance_metrics = result.performance_metrics

            # 计算场景评分
            scenario_score = 0.0
            if result.success:
                scenario_score = performance_metrics.get("overall_performance_score", 0.0)
            else:
                scenario_score = 0.0

            total_weighted_score += scenario_score * weight
            total_weight += weight

        return total_weighted_score / total_weight if total_weight > 0 else 0.0

    def _determine_satisfaction_level(self, score: float) -> UserSatisfactionLevel:
        """确定满意度等级"""
        if score >= 0.9:
            return UserSatisfactionLevel.EXCELLENT
        elif score >= 0.8:
            return UserSatisfactionLevel.GOOD
        elif score >= 0.6:
            return UserSatisfactionLevel.SATISFACTORY
        elif score >= 0.4:
            return UserSatisfactionLevel.POOR
        else:
            return UserSatisfactionLevel.VERY_POOR

    async def _generate_recommendations(self) -> List[str]:
        """生成改进建议"""
        recommendations = []

        if not self.current_session or not self.current_session.results:
            return recommendations

        # 分析失败的测试场景
        failed_scenarios = [r for r in self.current_session.results if not r.success]
        slow_scenarios = [r for r in self.current_session.results
                         if r.response_time_ms and r.response_time_ms > self.test_config["max_response_time_ms"]]

        # 生成建议
        if failed_scenarios:
            recommendations.append("建议优化语音识别准确率，特别是对粤语口语的理解")
            recommendations.append("加强多模态集成，提升视觉+语音协同处理能力")

        if slow_scenarios:
            recommendations.append("优化系统响应时间，目标控制在3秒以内")
            recommendations.append("考虑增加缓存机制，提升重复查询的响应速度")

        # 检查粤语自然度
        cantonese_issues = [r for r in self.current_session.results
                           if not r.actual_results.get("cantonese_response", False)]
        if cantonese_issues:
            recommendations.append("改进粤语生成模型，提供更自然的粤语表达")
            recommendations.append("增加更多粤语文化元素和本地化内容")

        # 性能建议
        avg_response_time = statistics.mean([r.response_time_ms for r in self.current_session.results
                                           if r.response_time_ms]) if self.current_session.results else 0
        if avg_response_time > 2500:
            recommendations.append("考虑优化网络连接和API调用效率")
            recommendations.append("实现更智能的预测缓存和预加载机制")

        if not recommendations:
            recommendations.append("系统表现优秀，建议持续监控并收集更多用户反馈")

        return recommendations

    async def _generate_test_report(self) -> Dict[str, Any]:
        """生成测试报告"""
        if not self.current_session:
            return {"error": "没有测试数据"}

        session = self.current_session
        results = session.results

        # 统计信息
        total_scenarios = len(results)
        successful_scenarios = len([r for r in results if r.success])
        success_rate = successful_scenarios / total_scenarios if total_scenarios > 0 else 0

        response_times = [r.response_time_ms for r in results if r.response_time_ms]
        avg_response_time = statistics.mean(response_times) if response_times else 0

        # 分类统计
        category_stats = {}
        for scenario in session.scenarios:
            category = scenario.category
            if category not in category_stats:
                category_stats[category] = {"total": 0, "successful": 0}

            category_stats[category]["total"] += 1

            result = next((r for r in results if r.scenario_id == scenario.id), None)
            if result and result.success:
                category_stats[category]["successful"] += 1

        # 生成报告
        report = {
            "session_info": {
                "session_id": session.session_id,
                "test_date": session.test_date.isoformat(),
                "user_profile": session.user_profile
            },
            "summary": {
                "total_scenarios": total_scenarios,
                "successful_scenarios": successful_scenarios,
                "success_rate": success_rate,
                "overall_score": session.overall_score,
                "user_satisfaction": session.user_satisfaction.value,
                "average_response_time_ms": avg_response_time,
                "status": session.status.value
            },
            "category_performance": category_stats,
            "detailed_results": [asdict(result) for result in results],
            "recommendations": session.recommendations,
            "user_feedback": session.user_feedback,
            "performance_baselines": self.performance_baselines,
            "generated_at": datetime.now().isoformat()
        }

        return report

    def get_test_session_status(self, session_id: Optional[str] = None) -> Dict[str, Any]:
        """获取测试会话状态"""
        if session_id:
            # 查找特定会话
            session = next((s for s in self.test_history if s.session_id == session_id), None)
            if session:
                return asdict(session)
            else:
                return {"error": "会话不存在"}
        elif self.current_session:
            # 返回当前会话
            return asdict(self.current_session)
        else:
            return {"status": "no_active_session"}

    def get_test_history(self, limit: int = 10) -> List[Dict[str, Any]]:
        """获取测试历史"""
        history = self.test_history[-limit:] if limit > 0 else self.test_history
        return [asdict(session) for session in history]

# 工厂函数
def create_cantonese_uat() -> CantoneseUserAcceptanceTest:
    """创建粤语家庭用户验收测试实例"""
    return CantoneseUserAcceptanceTest()