#!/usr/bin/env python3.10
"""
多模态上下文处理器 - 音视频上下文融合管理
Story 1.6: 视觉理解集成开发 - Day 12-14

功能特性:
- 多模态上下文数据结构管理
- 音视频输入融合算法
- 对话状态管理
- 上下文连贯性保证
- 粤语多模态对话优化
- Brownfield Level 4企业级标准
"""

import time
import json
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass, field
from datetime import datetime, timedelta
import uuid


@dataclass
class MultimodalInput:
    """多模态输入数据结构"""
    input_id: str
    timestamp: float
    input_type: str  # 'text', 'audio', 'image'
    content: Any
    metadata: Dict[str, Any] = field(default_factory=dict)
    session_id: str = ""


@dataclass
class ContextEntry:
    """上下文条目"""
    entry_id: str
    session_id: str
    inputs: List[MultimodalInput]
    response: Optional[str] = None
    context_type: str = "multimodal"  # 'text_only', 'visual', 'multimodal'
    created_at: float = field(default_factory=time.time)
    relevance_score: float = 1.0
    tags: List[str] = field(default_factory=list)


@dataclass
class SessionContext:
    """会话上下文"""
    session_id: str
    created_at: float = field(default_factory=time.time)
    last_active: float = field(default_factory=time.time)
    entries: List[ContextEntry] = field(default_factory=list)
    user_profile: Dict[str, Any] = field(default_factory=dict)
    context_state: Dict[str, Any] = field(default_factory=dict)
    total_interactions: int = 0


class CantoneseContextOptimizer:
    """粤语上下文优化器"""

    def __init__(self):
        # 粤语对话模式
        self.conversation_patterns = {
            'greeting': ['你好', '早晨', '您好', 'Hi', 'Hello'],
            'farewell': ['拜拜', '再見', '再见', 'Goodbye'],
            'question': ['乜嘢', '什么', '點解', '为什么', '邊個', '哪个'],
            'visual_query': ['睇下', '睇睇', '睇', '看下', '看看', '看'],
            'gratitude': ['唔該', '多謝', '谢谢', 'Thank you'],
            'acknowledgment': ['好', '得', 'OK', '明白', '知道']
        }

        # 粤语视觉相关词汇
        self.visual_terms = {
            '颜色': ['紅色', '藍色', '綠色', '黃色', '黑色', '白色'],
            '形状': ['圓形', '方形', '三角形', '長方形'],
            '位置': ['上面', '下面', '左邊', '右邊', '中間', '旁邊'],
            '数量': ['一個', '兩個', '三個', '好多', '少少'],
            '大小': ['大', '細', '中等', '好大', '好細']
        }

    def analyze_text_intent(self, text: str) -> Dict[str, Any]:
        """分析文本意图"""
        text_lower = text.lower()
        intent_analysis = {
            'intent_type': 'general',
            'is_cantonese': False,
            'visual_query': False,
            'confidence': 0.0
        }

        # 检查是否包含粤语词汇
        cantonese_detected = False
        for pattern_type, patterns in self.conversation_patterns.items():
            for pattern in patterns:
                if pattern in text_lower:
                    cantonese_detected = True
                    intent_analysis['intent_type'] = pattern_type
                    intent_analysis['confidence'] = 0.8
                    break

        # 检查视觉查询
        visual_keywords = ['睇', '看', '圖', '圖片', '相', '照片']
        if any(keyword in text_lower for keyword in visual_keywords):
            intent_analysis['visual_query'] = True
            intent_analysis['confidence'] = max(intent_analysis['confidence'], 0.7)

        intent_analysis['is_cantonese'] = cantonese_detected
        return intent_analysis

    def optimize_context_prompt(self, context: List[ContextEntry], current_input: str) -> str:
        """优化上下文提示词"""
        # 基础提示词
        base_prompt = "請根據以下多模態對話上下文，用粵語回答問題：\n\n"

        # 添加历史上下文
        if context:
            base_prompt += "對話歷史：\n"
            for i, entry in enumerate(context[-3:]):  # 只保留最近3条
                if entry.inputs:
                    for input_item in entry.inputs:
                        if input_item.input_type == 'text':
                            base_prompt += f"用戶：{input_item.content}\n"
                    if entry.response:
                        base_prompt += f"助手：{entry.response}\n"
                base_prompt += "\n"

        # 添加当前问题
        base_prompt += f"當前問題：{current_input}\n\n"
        base_prompt += "請基於上下文和圖像信息，用自然嘅粵語回答。"

        return base_prompt


class ContextFusionEngine:
    """上下文融合引擎"""

    def __init__(self):
        self.fusion_weights = {
            'text': 0.4,
            'audio': 0.3,
            'image': 0.3
        }
        self.temporal_decay = 0.9  # 时间衰减因子

    def calculate_relevance_score(self, entry: ContextEntry, current_time: float) -> float:
        """计算上下文条目的相关性分数"""
        # 时间衰减
        time_diff = current_time - entry.created_at
        temporal_score = self.temporal_decay ** (time_diff / 60.0)  # 每分钟衰减

        # 输入类型权重
        input_weights = 0.0
        for input_item in entry.inputs:
            weight = self.fusion_weights.get(input_item.input_type, 0.1)
            input_weights += weight

        if entry.inputs:
            input_weights /= len(entry.inputs)

        # 综合评分
        relevance_score = temporal_score * input_weights * entry.relevance_score
        return min(max(relevance_score, 0.0), 1.0)

    def fuse_multimodal_inputs(self, inputs: List[MultimodalInput]) -> Dict[str, Any]:
        """融合多模态输入"""
        fusion_result = {
            'fused_text': '',
            'has_visual': False,
            'has_audio': False,
            'dominant_modality': 'text',
            'confidence': 0.0
        }

        if not inputs:
            return fusion_result

        # 分类输入
        text_inputs = [inp for inp in inputs if inp.input_type == 'text']
        audio_inputs = [inp for inp in inputs if inp.input_type == 'audio']
        image_inputs = [inp for inp in inputs if inp.input_type == 'image']

        fusion_result['has_visual'] = len(image_inputs) > 0
        fusion_result['has_audio'] = len(audio_inputs) > 0

        # 融合文本内容
        if text_inputs:
            fusion_result['fused_text'] = ' '.join([inp.content for inp in text_inputs])

        # 确定主导模态
        modality_scores = {
            'text': len(text_inputs) * self.fusion_weights['text'],
            'audio': len(audio_inputs) * self.fusion_weights['audio'],
            'visual': len(image_inputs) * self.fusion_weights['image']
        }

        fusion_result['dominant_modality'] = max(modality_scores, key=modality_scores.get)
        fusion_result['confidence'] = max(modality_scores.values()) / max(len(inputs), 1)

        return fusion_result


class MultimodalContextProcessor:
    """多模态上下文处理器主类"""

    def __init__(self, max_context_entries: int = 10, session_timeout: int = 1800):
        self.max_context_entries = max_context_entries
        self.session_timeout = session_timeout  # 30分钟
        self.sessions: Dict[str, SessionContext] = {}
        self.fusion_engine = ContextFusionEngine()
        self.cantonese_optimizer = CantoneseContextOptimizer()

        # 统计信息
        self.stats = {
            'total_sessions': 0,
            'total_interactions': 0,
            'multimodal_interactions': 0,
            'cantonese_interactions': 0
        }

    def get_or_create_session(self, session_id: str = None) -> SessionContext:
        """获取或创建会话"""
        if session_id is None:
            session_id = str(uuid.uuid4())

        if session_id not in self.sessions:
            self.sessions[session_id] = SessionContext(session_id=session_id)
            self.stats['total_sessions'] += 1

        session = self.sessions[session_id]
        session.last_active = time.time()
        return session

    def add_multimodal_input(self, session_id: str, input_type: str, content: Any,
                           metadata: Dict[str, Any] = None) -> str:
        """添加多模态输入"""
        session = self.get_or_create_session(session_id)

        # 创建输入对象
        multimodal_input = MultimodalInput(
            input_id=str(uuid.uuid4()),
            timestamp=time.time(),
            input_type=input_type,
            content=content,
            metadata=metadata or {},
            session_id=session_id
        )

        # 创建新的上下文条目
        context_entry = ContextEntry(
            entry_id=str(uuid.uuid4()),
            session_id=session_id,
            inputs=[multimodal_input]
        )

        # 确定上下文类型
        if input_type == 'image' or any(inp.input_type == 'image' for inp in session.entries[-3:]):
            context_entry.context_type = 'visual'
        elif input_type == 'audio':
            context_entry.context_type = 'multimodal'
        else:
            context_entry.context_type = 'text_only'

        # 添加到会话
        session.entries.append(context_entry)
        session.total_interactions += 1
        self.stats['total_interactions'] += 1

        # 更新统计
        if context_entry.context_type in ['visual', 'multimodal']:
            self.stats['multimodal_interactions'] += 1

        # 清理旧上下文
        self._cleanup_old_entries(session)

        return context_entry.entry_id

    def process_current_context(self, session_id: str, current_input: str,
                              images: List[str] = None) -> Dict[str, Any]:
        """处理当前上下文"""
        session = self.get_or_create_session(session_id)

        # 分析当前输入意图
        intent_analysis = self.cantonese_optimizer.analyze_text_intent(current_input)

        # 获取相关上下文
        current_time = time.time()
        relevant_context = self._get_relevant_context(session, current_time)

        # 融合多模态输入
        fusion_result = self.fusion_engine.fuse_multimodal_inputs(
            [inp for entry in relevant_context for inp in entry.inputs] +
            ([MultimodalInput("", current_time, 'text', current_input)] if current_input else [])
        )

        # 优化提示词
        if intent_analysis['is_cantonese']:
            self.stats['cantonese_interactions'] += 1
            optimized_prompt = self.cantonese_optimizer.optimize_context_prompt(
                relevant_context, current_input)
        else:
            optimized_prompt = current_input

        # 构建上下文信息
        context_info = {
            'session_id': session_id,
            'current_input': current_input,
            'images': images or [],
            'relevant_context': relevant_context,
            'fusion_result': fusion_result,
            'intent_analysis': intent_analysis,
            'optimized_prompt': optimized_prompt,
            'context_confidence': self._calculate_context_confidence(relevant_context, fusion_result),
            'suggested_max_tokens': self._suggest_max_tokens(intent_analysis, fusion_result)
        }

        return context_info

    def store_response(self, session_id: str, entry_id: str, response: str):
        """存储响应"""
        if session_id in self.sessions:
            session = self.sessions[session_id]
            for entry in session.entries:
                if entry.entry_id == entry_id:
                    entry.response = response
                    break

    def _get_relevant_context(self, session: SessionContext, current_time: float) -> List[ContextEntry]:
        """获取相关上下文"""
        # 计算相关性分数并排序
        scored_entries = []
        for entry in session.entries:
            relevance_score = self.fusion_engine.calculate_relevance_score(entry, current_time)
            if relevance_score > 0.1:  # 过滤低相关性条目
                scored_entries.append((entry, relevance_score))

        # 按相关性排序，取前N条
        scored_entries.sort(key=lambda x: x[1], reverse=True)
        return [entry for entry, _ in scored_entries[:self.max_context_entries]]

    def _calculate_context_confidence(self, context: List[ContextEntry],
                                   fusion_result: Dict[str, Any]) -> float:
        """计算上下文置信度"""
        if not context:
            return 0.0

        # 基于上下文条目数量和融合结果计算置信度
        context_score = min(len(context) / 5.0, 1.0) * 0.6  # 上下文丰富度
        fusion_score = fusion_result.get('confidence', 0.0) * 0.4  # 融合置信度

        return context_score + fusion_score

    def _suggest_max_tokens(self, intent_analysis: Dict[str, Any],
                          fusion_result: Dict[str, Any]) -> int:
        """建议最大token数"""
        base_tokens = 200

        # 根据意图类型调整
        if intent_analysis['visual_query']:
            base_tokens += 100  # 视觉查询需要更多token

        # 根据模态复杂度调整
        if fusion_result['has_visual']:
            base_tokens += 150  # 有图像时增加

        if fusion_result['dominant_modality'] == 'multimodal':
            base_tokens += 100  # 多模态时增加

        return min(base_tokens, 800)  # 最大不超过800

    def _cleanup_old_entries(self, session: SessionContext):
        """清理旧上下文条目"""
        current_time = time.time()
        cutoff_time = current_time - self.session_timeout

        # 移除超时的条目
        session.entries = [
            entry for entry in session.entries
            if entry.created_at > cutoff_time
        ]

        # 如果仍然太多，保留最近的
        if len(session.entries) > self.max_context_entries:
            session.entries = session.entries[-self.max_context_entries:]

    def get_session_summary(self, session_id: str) -> Dict[str, Any]:
        """获取会话摘要"""
        if session_id not in self.sessions:
            return {'error': 'Session not found'}

        session = self.sessions[session_id]
        return {
            'session_id': session_id,
            'created_at': session.created_at,
            'last_active': session.last_active,
            'total_interactions': session.total_interactions,
            'context_entries': len(session.entries),
            'session_duration': time.time() - session.created_at,
            'multimodal_ratio': (
                len([e for e in session.entries if e.context_type in ['visual', 'multimodal']]) /
                max(len(session.entries), 1)
            )
        }

    def get_processor_stats(self) -> Dict[str, Any]:
        """获取处理器统计信息"""
        active_sessions = len([
            s for s in self.sessions.values()
            if time.time() - s.last_active < self.session_timeout
        ])

        return {
            **self.stats,
            'active_sessions': active_sessions,
            'total_sessions_created': len(self.sessions),
            'average_session_length': (
                sum(s.total_interactions for s in self.sessions.values()) /
                max(len(self.sessions), 1)
            ),
            'multimodal_interaction_ratio': (
                self.stats['multimodal_interactions'] /
                max(self.stats['total_interactions'], 1)
            ),
            'cantonese_interaction_ratio': (
                self.stats['cantonese_interactions'] /
                max(self.stats['total_interactions'], 1)
            )
        }


def main():
    """测试函数"""
    print("🧠 多模态上下文处理器测试")
    print("=" * 50)

    try:
        # 创建处理器
        processor = MultimodalContextProcessor()
        print("✅ 处理器初始化成功")

        # 创建测试会话
        session_id = "test_session_001"
        print(f"🆔 创建会话: {session_id}")

        # 添加文本输入
        entry_id_1 = processor.add_multimodal_input(
            session_id, 'text', '你好，我想問關於呢張圖嘅問題')
        print("✅ 添加文本输入")

        # 添加图像输入（模拟）
        entry_id_2 = processor.add_multimodal_input(
            session_id, 'image', '/path/to/test_image.jpg',
            {'format': 'jpeg', 'size': '640x480'})
        print("✅ 添加图像输入")

        # 处理当前上下文
        context_info = processor.process_current_context(
            session_id, '呢張圖畫緊乜？', ['/path/to/test_image.jpg'])

        print(f"📊 上下文置信度: {context_info['context_confidence']:.2f}")
        print(f"🎯 意图类型: {context_info['intent_analysis']['intent_type']}")
        print(f"🗣️ 粤语检测: {context_info['intent_analysis']['is_cantonese']}")
        print(f"👀 视觉查询: {context_info['intent_analysis']['visual_query']}")
        print(f"📝 建议token数: {context_info['suggested_max_tokens']}")

        # 存储响应
        processor.store_response(session_id, entry_id_2, "呢張圖顯示一個紅色圓形")
        print("✅ 存储响应")

        # 获取会话摘要
        summary = processor.get_session_summary(session_id)
        print(f"📋 会话交互次数: {summary['total_interactions']}")
        print(f"📊 多模态比例: {summary['multimodal_ratio']:.2%}")

        # 获取处理器统计
        stats = processor.get_processor_stats()
        print(f"📈 总交互次数: {stats['total_interactions']}")
        print(f"🌐 多模态交互比例: {stats['multimodal_interaction_ratio']:.2%}")
        print(f"🗣️ 粤语交互比例: {stats['cantonese_interaction_ratio']:.2%}")

        print("\n✅ 多模态上下文处理器测试完成")

    except Exception as e:
        print(f"❌ 测试失败: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()