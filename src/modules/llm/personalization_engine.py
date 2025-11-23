#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
个性化引擎模块 - XLeBot LLM个性化处理
提供用户偏好学习、对话历史管理、个性化响应生成等功能
"""

import time
import json
import logging
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, asdict
from datetime import datetime
import os

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

@dataclass
class UserProfile:
    """用户档案数据类"""
    user_id: str
    name: Optional[str] = None
    language_preference: str = "cantonese"
    interaction_style: str = "friendly"
    response_length: str = "medium"  # short, medium, long
    topics_of_interest: List[str] = None
    interaction_count: int = 0
    last_interaction: Optional[str] = None
    created_at: str = None

    def __post_init__(self):
        if self.topics_of_interest is None:
            self.topics_of_interest = []
        if self.created_at is None:
            self.created_at = datetime.now().isoformat()

@dataclass
class DialogueContext:
    """对话上下文数据类"""
    session_id: str
    user_id: str
    timestamp: str
    user_input: str
    system_response: str
    intent: Optional[str] = None
    entities: Dict[str, Any] = None
    sentiment: Optional[str] = None
    context_tags: List[str] = None

    def __post_init__(self):
        if self.entities is None:
            self.entities = {}
        if self.context_tags is None:
            self.context_tags = []

class PersonalizationEngine:
    """个性化引擎主类"""

    def __init__(self, storage_path: str = "data/personalization"):
        self.storage_path = storage_path
        self.user_profiles: Dict[str, UserProfile] = {}
        self.dialogue_history: List[DialogueContext] = []
        self.context_window = 10  # 上下文窗口大小

        # 确保存储目录存在
        os.makedirs(storage_path, exist_ok=True)

        # 加载现有数据
        self._load_user_profiles()
        self._load_dialogue_history()

        logger.info("个性化引擎初始化完成")

    def _load_user_profiles(self) -> None:
        """加载用户档案"""
        try:
            profile_file = os.path.join(self.storage_path, "user_profiles.json")
            if os.path.exists(profile_file):
                with open(profile_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    for user_id, profile_data in data.items():
                        self.user_profiles[user_id] = UserProfile(**profile_data)
                logger.info(f"加载了 {len(self.user_profiles)} 个用户档案")
        except Exception as e:
            logger.warning(f"加载用户档案失败: {e}")

    def _save_user_profiles(self) -> None:
        """保存用户档案"""
        try:
            profile_file = os.path.join(self.storage_path, "user_profiles.json")
            data = {user_id: asdict(profile) for user_id, profile in self.user_profiles.items()}
            with open(profile_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            logger.error(f"保存用户档案失败: {e}")

    def _load_dialogue_history(self) -> None:
        """加载对话历史"""
        try:
            history_file = os.path.join(self.storage_path, "dialogue_history.json")
            if os.path.exists(history_file):
                with open(history_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    for item in data:
                        self.dialogue_history.append(DialogueContext(**item))
                logger.info(f"加载了 {len(self.dialogue_history)} 条对话历史")
        except Exception as e:
            logger.warning(f"加载对话历史失败: {e}")

    def _save_dialogue_history(self) -> None:
        """保存对话历史"""
        try:
            history_file = os.path.join(self.storage_path, "dialogue_history.json")
            data = [asdict(context) for context in self.dialogue_history]
            # 只保存最近的1000条对话
            data = data[-1000:]
            with open(history_file, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
        except Exception as e:
            logger.error(f"保存对话历史失败: {e}")

    def get_or_create_user(self, user_id: str, name: Optional[str] = None) -> UserProfile:
        """获取或创建用户档案"""
        if user_id not in self.user_profiles:
            self.user_profiles[user_id] = UserProfile(user_id=user_id, name=name)
            self._save_user_profiles()
            logger.info(f"创建新用户档案: {user_id}")

        return self.user_profiles[user_id]

    def update_user_profile(self, user_id: str, **kwargs) -> None:
        """更新用户档案"""
        if user_id in self.user_profiles:
            profile = self.user_profiles[user_id]
            for key, value in kwargs.items():
                if hasattr(profile, key):
                    setattr(profile, key, value)
            profile.last_interaction = datetime.now().isoformat()
            profile.interaction_count += 1
            self._save_user_profiles()
            logger.info(f"更新用户档案: {user_id}")

    def add_dialogue_context(self, session_id: str, user_id: str,
                           user_input: str, system_response: str,
                           intent: Optional[str] = None,
                           entities: Optional[Dict[str, Any]] = None) -> None:
        """添加对话上下文"""
        context = DialogueContext(
            session_id=session_id,
            user_id=user_id,
            timestamp=datetime.now().isoformat(),
            user_input=user_input,
            system_response=system_response,
            intent=intent,
            entities=entities or {}
        )

        self.dialogue_history.append(context)

        # 保持历史记录在合理范围内
        if len(self.dialogue_history) > 1000:
            self.dialogue_history = self.dialogue_history[-1000:]

        self._save_dialogue_history()

        # 更新用户互动次数
        self.update_user_profile(user_id)

    def get_recent_context(self, user_id: str, limit: int = 5) -> List[DialogueContext]:
        """获取用户最近的对话上下文"""
        user_contexts = [ctx for ctx in self.dialogue_history
                        if ctx.user_id == user_id]
        return user_contexts[-limit:]

    def get_user_preferences(self, user_id: str) -> Dict[str, Any]:
        """获取用户偏好设置"""
        if user_id not in self.user_profiles:
            return {}

        profile = self.user_profiles[user_id]
        return {
            "language_preference": profile.language_preference,
            "interaction_style": profile.interaction_style,
            "response_length": profile.response_length,
            "topics_of_interest": profile.topics_of_interest
        }

    def personalize_response(self, response: str, user_id: str,
                           intent: Optional[str] = None) -> str:
        """个性化响应内容"""
        if user_id not in self.user_profiles:
            return response

        profile = self.user_profiles[user_id]

        # 根据交互风格调整响应
        if profile.interaction_style == "formal":
            response = self._make_formal(response)
        elif profile.interaction_style == "friendly":
            response = self._make_friendly(response)
        elif profile.interaction_style == "professional":
            response = self._make_professional(response)

        # 根据响应长度偏好调整
        if profile.response_length == "short":
            response = self._shorten_response(response)
        elif profile.response_length == "long":
            response = self._expand_response(response)

        return response

    def _make_formal(self, response: str) -> str:
        """使响应更正式"""
        formal_prefixes = ["好的，", "明白了，", "是的，"]
        for prefix in formal_prefixes:
            if response.startswith(prefix):
                response = "好的" + response[2:] + "。"
                break
        return response

    def _make_friendly(self, response: str) -> str:
        """使响应更友好"""
        friendly_suffixes = ["呀！", "呢！", "哦！"]
        if not any(response.endswith(suffix) for suffix in friendly_suffixes):
            if len(response) > 5:
                response = response[:-1] + "呢！"
        return response

    def _make_professional(self, response: str) -> str:
        """使响应更专业"""
        # 移除过于随意的表达
        casual_words = ["哈哈", "呵呵", "哎呀"]
        for word in casual_words:
            response = response.replace(word, "")
        return response.strip()

    def _shorten_response(self, response: str) -> str:
        """缩短响应长度"""
        sentences = response.split("。")
        if len(sentences) > 2:
            response = "。".join(sentences[:2]) + "。"
        return response

    def _expand_response(self, response: str) -> str:
        """扩展响应长度"""
        if len(response) < 20:
            response += "希望这个回答对您有帮助。"
        return response

    def learn_from_feedback(self, user_id: str, session_id: str,
                          feedback_score: int, feedback_comment: Optional[str] = None) -> None:
        """从用户反馈中学习"""
        # 更新用户档案中的反馈历史
        if user_id in self.user_profiles:
            profile = self.user_profiles[user_id]

            # 根据反馈调整交互风格
            if feedback_score >= 4:  # 满意
                if profile.interaction_style == "formal":
                    # 如果用户对正式风格满意，保持
                    pass
            elif feedback_score <= 2:  # 不满意
                # 尝试调整交互风格
                if profile.interaction_style == "formal":
                    profile.interaction_style = "friendly"
                elif profile.interaction_style == "friendly":
                    profile.interaction_style = "professional"

            self._save_user_profiles()
            logger.info(f"根据用户反馈更新档案 {user_id}: {feedback_score}分")

    def get_user_statistics(self, user_id: str) -> Dict[str, Any]:
        """获取用户统计信息"""
        if user_id not in self.user_profiles:
            return {}

        profile = self.user_profiles[user_id]
        user_dialogues = [ctx for ctx in self.dialogue_history
                         if ctx.user_id == user_id]

        return {
            "interaction_count": profile.interaction_count,
            "last_interaction": profile.last_interaction,
            "total_dialogues": len(user_dialogues),
            "avg_response_length": sum(len(ctx.system_response) for ctx in user_dialogues) / max(len(user_dialogues), 1),
            "most_common_intents": self._get_most_common_intents(user_dialogues)
        }

    def _get_most_common_intents(self, dialogues: List[DialogueContext]) -> List[str]:
        """获取最常见的意图"""
        intents = [ctx.intent for ctx in dialogues if ctx.intent]
        from collections import Counter
        return [intent for intent, count in Counter(intents).most_common(5)]

    def get_personalization_stats(self) -> Dict[str, Any]:
        """获取个性化引擎统计信息"""
        return {
            'active_users': len(self.user_profiles),
            'total_personalizations': len(self.user_profiles),
            'total_dialogues': len(self.dialogue_history),
            'context_window': self.context_window
        }

# 全局个性化引擎实例
personalization_engine = PersonalizationEngine()

def get_personalization_engine() -> PersonalizationEngine:
    """获取个性化引擎实例"""
    return personalization_engine

# 简化版本接口（用于向后兼容）
class SimplePersonalizationEngine:
    """简化版个性化引擎"""

    def __init__(self):
        self.engine = get_personalization_engine()

    def get_user_profile(self, user_id: str) -> Dict[str, Any]:
        """获取用户档案"""
        if user_id in self.engine.user_profiles:
            return asdict(self.engine.user_profiles[user_id])
        return {"user_id": user_id, "interaction_count": 0}

    def update_preferences(self, user_id: str, preferences: Dict[str, Any]) -> None:
        """更新用户偏好"""
        self.engine.update_user_profile(user_id, **preferences)

    def personalize_text(self, text: str, user_id: str) -> str:
        """个性化文本"""
        return self.engine.personalize_response(text, user_id)

# 向后兼容的实例
simple_engine = SimplePersonalizationEngine()