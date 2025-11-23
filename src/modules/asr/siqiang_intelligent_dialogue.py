#!/usr/bin/env python3
"""
傻强智能对话管理模块
====================

为XleRobot语音助手傻强提供智能对话能力
支持粤语问候、情感表达、功能问答等多种场景

人设特点：
- 名字：傻强 (Si Qiang)
- 性格：友好、贴心、略带幽默感的粤语助手
- 语言：以粤语为主，自然流畅
- 能力：智能对话、功能服务、情感陪伴

作者: BMad Master
版本: 1.0 (智能对话版)
日期: 2025-11-14
"""

import re
import random
import logging
from datetime import datetime
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class DialogueResponse:
    """对话响应数据结构"""
    text: str
    emotion: str = "friendly"  # friendly, excited, calm, concerned
    confidence: float = 0.9
    category: str = "general"  # greeting, farewell, thanks, weather, time, help, chat

class SiQiangIntelligentDialogue:
    """傻强智能对话管理器"""

    def __init__(self):
        """初始化对话管理器"""
        self.conversation_count = 0
        self.user_name = "老细"  # 默认称呼用户为"老细"(老板)

        # 初始化对话库
        self._init_dialogue_patterns()
        logger.info("✅ 傻强智能对话管理器初始化完成")

    def _init_dialogue_patterns(self):
        """初始化对话模式库"""

        # 问候类对话
        self.问候模式 = {
            '早晨': [
                "早晨！老细，今日精神咁好！有咩可以帮到你？",
                "早晨啊！今日想点样安排？傻强随时准备！",
                "早晨老细！望你今日开心愉快！",
                "Good Morning！傻强听晒你嘅！"
            ],
            '你好': [
                "你好！我是傻强，你嘅贴心助手，有咩事？",
                "哈喽！傻强嚟啦！有咩可以帮手？",
                "你好呀！见到你真开心！",
                "Hi！傻强Ready好，请指示！"
            ],
            '早晨好/早上好': [
                "早晨好！新的一天开始啦，有咩计划？",
                "早上好！傻强祝你今日顺利！",
                "Good Morning！有咩需要傻强帮手？"
            ]
        }

        # 告别类对话
        self.告别模式 = {
            '拜拜/再见': [
                "拜拜！记得有需要随时叫傻强啊！",
                "再见！傻强会想念你嘅！",
                "Bye Bye！老细慢慢玩，下次见！",
                "拜拜！保重身体，有心嘢记得搵傻强！"
            ],
            '我走先': [
                "好嘅，老细慢走！傻强随时等你返来！",
                "行啦！路上小心，有空再倾！",
                "OK！下次见啦，多谢使用傻强！"
            ]
        }

        # 感谢类对话
        self.感谢模式 = {
            '多谢/唔该': [
                "唔使客气！傻强帮你係应该嘅！",
                "小事一桩！老细唔使咁客气！",
                "随时欢迎！傻强好乐意服务！",
                "彼此彼此！老细开心我就开心！"
            ],
            '感谢': [
                "唔使客气！能帮到你真系傻强嘅荣幸！",
                "谢谢老细嘅认可！傻强会继续努力！",
                "老细客气了！有咩需要再叫我！"
            ]
        }

        # 功能问答
        self.功能问答 = {
            '天气': [
                "今日天气唔错呢！你觉得点？",
                "睇咗天气，今日都几适合出街呱！",
                "天气方面，建议老细出门前多留意预报啦！",
                "今日天气预报话几好，记得注意温差啊！"
            ],
            '时间': [
                "时间过得真快，老细要好好把握时间哦！",
                "我睇下时间... 记得按时休息啊！",
                "时间就系金钱，老细要善用每一刻！",
                "记得准时吃饭休息，傻强关心你嘅健康！"
            ],
            '帮助': [
                "傻强可以帮你好多嘢！问我天气时间都得！",
                "有咩需要帮忙？傻强全力为你服务！",
                "随时欢迎提问！傻强有问必答！",
                "老细有咩困难？傻强一定尽力帮你！"
            ]
        }

        # 闲聊对话
        self.闲聊模式 = {
            '你係乜嘢/你系咩': [
                "我係傻强，你嘅智能粤语助手！",
                "你好！我叫傻强，专门为你服务嘅AI助手！",
                "我係傻强！一个友好、贴心嘅粤语语音助手！"
            ],
            '你好嘛/你好吗': [
                "我好啊！见到老细特别开心！",
                "我好得不得了！老细你呢？",
                "我精神饱满！随时准备帮老细！",
                "Good！能为你服务，傻强最开心！"
            ],
            '做咩啊/做什么': [
                "我等你叫唤啊，老细！有咩吩咐？",
                "咩都冇做，等你老细指示啦！",
                "呆呆哋等你嘅消息啊！有咩要我做？",
                "准备就绪，随时听候老细差遣！"
            ],
            '几时/何时': [
                "这个嘛，要看具体情况哦！",
                "几时都可以！老细决定时间！",
                "随时都可以，等老细方便嘅时候！",
                "时间灵活，以老细嘅时间为先！"
            ]
        }

        # 情感支持
        self.情感支持 = {
            '心情不好/唔开心': [
                "老细唔开心啊？傻强陪你倾倾偈！",
                "咩事令你唔开心？讲嚟听听啦！",
                "唔使愁，傻强陪你度过难关！",
                "开心啲啦！有傻强陪着你啊！"
            ],
            '好攰/好累': [
                "老细辛苦了！记得休息一下啊！",
                "唔好攰坏身体，要懂得放松！",
                "保重身体，傻强心疼你啊！",
                "找个时间好好休息，劳逸结合很重要！"
            ]
        }

    def generate_response(self, user_input: str) -> DialogueResponse:
        """
        生成智能对话响应

        Args:
            user_input: 用户输入文本

        Returns:
            DialogueResponse: 对话响应对象
        """
        try:
            self.conversation_count += 1

            # 预处理用户输入
            user_text = user_input.strip()
            if not user_text:
                return DialogueResponse(
                    text="老细，你冇说话哦，有咩想讲？",
                    emotion="friendly",
                    category="chat"
                )

            # 转换为小写进行匹配
            text_lower = user_text.lower()

            # 按优先级匹配对话模式
            response = self._match_greeting(text_lower)
            if response:
                return response

            response = self._match_farewell(text_lower)
            if response:
                return response

            response = self._match_thanks(text_lower)
            if response:
                return response

            response = self._match_functional_questions(text_lower)
            if response:
                return response

            response = self._match_emotional_support(text_lower)
            if response:
                return response

            response = self._match_chat(text_lower)
            if response:
                return response

            # 默认回应
            return self._default_response(user_text)

        except Exception as e:
            logger.error(f"❌ 生成对话响应失败: {e}")
            return DialogueResponse(
                text="抱歉，傻强暂时听不明白，可唔可以再讲一次？",
                emotion="friendly",
                confidence=0.5,
                category="error"
            )

    def _match_greeting(self, text: str) -> Optional[DialogueResponse]:
        """匹配问候语"""
        patterns = {
            '早晨': ['早晨', 'good morning', '早安'],
            '你好': ['你好', '哈喽', 'hello', 'hi', '嗨'],
            '晚上好': ['晚上好', 'good evening', '晚安']
        }

        for category, keywords in patterns.items():
            if any(keyword in text for keyword in keywords):
                if category in self.问候模式:
                    response_text = random.choice(self.问候模式[category])
                    return DialogueResponse(
                        text=response_text,
                        emotion="friendly",
                        confidence=0.95,
                        category="greeting"
                    )
        return None

    def _match_farewell(self, text: str) -> Optional[DialogueResponse]:
        """匹配告别语"""
        patterns = {
            '拜拜/再见': ['拜拜', '再见', 'bye', 'byybye', 'byebye'],
            '我走先': ['我走先', '我走了', '我先走了']
        }

        for category, keywords in patterns.items():
            if any(keyword in text for keyword in keywords):
                if category in self.告别模式:
                    response_text = random.choice(self.告别模式[category])
                    return DialogueResponse(
                        text=response_text,
                        emotion="friendly",
                        confidence=0.95,
                        category="farewell"
                    )
        return None

    def _match_thanks(self, text: str) -> Optional[DialogueResponse]:
        """匹配感谢语"""
        patterns = {
            '多谢/唔该': ['多谢', '唔该', 'thanks', 'thank you', '感激'],
            '感谢': ['感谢', '谢谢', 'thx']
        }

        for category, keywords in patterns.items():
            if any(keyword in text for keyword in keywords):
                if category in self.感谢模式:
                    response_text = random.choice(self.感谢模式[category])
                    return DialogueResponse(
                        text=response_text,
                        emotion="friendly",
                        confidence=0.95,
                        category="thanks"
                    )
        return None

    def _match_functional_questions(self, text: str) -> Optional[DialogueResponse]:
        """匹配功能问答"""
        patterns = {
            '天气': ['天气', '天氣', '下雨', '晴天', '温度'],
            '时间': ['时间', '几点', '几时', '现在', 'current time'],
            '帮助': ['帮助', '帮手', '可以做', '功能', 'help']
        }

        for category, keywords in patterns.items():
            if any(keyword in text for keyword in keywords):
                if category == '时间':
                    # 特殊处理时间询问
                    current_time = datetime.now().strftime("%H点%M分")
                    responses = [
                        f"而家係{current_time}，记得安排好时间哦！",
                        f"现在时间{current_time}，老细要珍惜时间啊！",
                        f"时间係{current_time}，咁快又过咗一段时间！"
                    ]
                    return DialogueResponse(
                        text=random.choice(responses),
                        emotion="calm",
                        confidence=0.98,
                        category="time"
                    )
                elif category in self.功能问答:
                    response_text = random.choice(self.功能问答[category])
                    return DialogueResponse(
                        text=response_text,
                        emotion="friendly",
                        confidence=0.90,
                        category=category
                    )
        return None

    def _match_emotional_support(self, text: str) -> Optional[DialogueResponse]:
        """匹配情感支持"""
        patterns = {
            '心情不好/唔开心': ['唔开心', '心情不好', '沮丧', '难受', 'sad'],
            '好攰/好累': ['好攰', '好累', '疲倦', '疲惫', 'tired']
        }

        for category, keywords in patterns.items():
            if any(keyword in text for keyword in keywords):
                if category in self.情感支持:
                    response_text = random.choice(self.情感支持[category])
                    return DialogueResponse(
                        text=response_text,
                        emotion="concerned",
                        confidence=0.85,
                        category="emotional"
                    )
        return None

    def _match_chat(self, text: str) -> Optional[DialogueResponse]:
        """匹配闲聊对话"""
        patterns = {
            '你係乜嘢/你系咩': ['你係乜嘢', '你系咩', '你是谁', 'what are you', 'who are you'],
            '你好嘛/你好吗': ['你好嘛', '你好吗', 'how are you'],
            '做咩啊/做什么': ['做咩啊', '做什么', 'what are you doing'],
            '几时/何时': ['几时', '何时', 'when']
        }

        for category, keywords in patterns.items():
            if any(keyword in text for keyword in keywords):
                if category in self.闲聊模式:
                    response_text = random.choice(self.闲聊模式[category])
                    return DialogueResponse(
                        text=response_text,
                        emotion="friendly",
                        confidence=0.90,
                        category="chat"
                    )
        return None

    def _default_response(self, user_text: str) -> DialogueResponse:
        """默认回应"""
        default_responses = [
            f"老细你讲嘅'{user_text}'，傻强都觉得几有趣啊！",
            "嗯嗯，傻强明白！有咩需要帮手吗？",
            "收到！老细有咩指示，傻强随时准备！",
            "OK！傻强听明白，继续努力为你服务！",
            "好嘅！有咩其他需要傻强帮手？"
        ]

        return DialogueResponse(
            text=random.choice(default_responses),
            emotion="friendly",
            confidence=0.70,
            category="general"
        )

    def get_welcome_message(self) -> str:
        """获取欢迎消息"""
        welcome_messages = [
            "你好！我是傻强，你嘅贴心粤语助手！有咩可以帮到你？",
            "老细好！傻强 Ready 好啦，随时听候差遣！",
            "Hi！欢迎来到傻强嘅世界，开心同你交流！",
            "傻强来啦！希望可以为你带来便利同快乐！"
        ]
        return random.choice(welcome_messages)

    def get_conversation_stats(self) -> Dict[str, any]:
        """获取对话统计信息"""
        return {
            "总对话次数": self.conversation_count,
            "用户称呼": self.user_name,
            "助手名字": "傻强",
            "语言风格": "粤语为主，自然亲切",
            "性格特点": "友好、贴心、幽默"
        }

def create_siqiang_dialogue_manager() -> SiQiangIntelligentDialogue:
    """创建傻强对话管理器实例"""
    return SiQiangIntelligentDialogue()

# 测试代码
if __name__ == "__main__":
    print("🧪 傻强智能对话管理器测试")

    try:
        # 创建对话管理器
        dialogue = SiQiangIntelligentDialogue()

        # 测试各种对话
        test_inputs = [
            "早晨",
            "你好啊",
            "多谢你啊",
            "现在几点啊？",
            "你係乜嘢？",
            "心情唔好",
            "今日天气点？",
            "拜拜"
        ]

        print("\n🗣️ 对话测试开始:")
        for user_input in test_inputs:
            response = dialogue.generate_response(user_input)
            print(f"👤 用户: {user_input}")
            print(f"🤖 傻强: {response.text} (情绪:{response.emotion}, 置信度:{response.confidence:.2f})")
            print("-" * 50)

        # 显示统计信息
        stats = dialogue.get_conversation_stats()
        print(f"\n📊 对话统计: {stats}")

        print("\n✅ 傻强智能对话管理器测试完成")

    except Exception as e:
        print(f"❌ 测试失败: {e}")