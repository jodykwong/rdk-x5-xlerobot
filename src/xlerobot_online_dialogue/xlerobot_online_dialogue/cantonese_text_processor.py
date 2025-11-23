"""
CantoneseTextProcessor - 粤语文本预处理
Story 1.7: 多模态在线对话API集成
严格遵循Epic 1纯在线架构 - 仅文本预处理，无本地对话逻辑
"""

import re
from typing import Dict, List
import logging

logger = logging.getLogger(__name__)

class CantoneseTextProcessor:
    """
    粤语文本预处理器
    严格遵循Epic 1纯在线架构 - 仅文本预处理，无本地对话逻辑
    用于在API调用前优化粤语文本输入
    """

    def __init__(self):
        """初始化粤语文本预处理器"""
        logger.info("🌏 初始化CantoneseTextProcessor - 纯在线架构")

        # 粤语标准术语映射
        self.standard_terms = {
            # 基础词汇
            "我": "我",
            "你": "你",
            "他": "佢",
            "她": "佢",
            "我们": "我哋",
            "你们": "你哋",
            "他们": "佢哋",

            # 常用表达
            "什么": "乜嘢",
            "怎么": "點",
            "为什么": "點解",
            "哪里": "邊度",
            "哪个": "邊個",
            "这个": "呢個",
            "那个": "嗰個",
            "这里": "呢度",
            "那里": "嗰度",

            # 动词
            "是": "係",
            "有": "有",
            "没有": "冇",
            "不是": "唔係",
            "会": "會",
            "可以": "可以",
            "想要": "想",
            "需要": "需要",
            "知道": "知",

            # 形容词
            "好": "好",
            "不好": "唔好",
            "对的": "啱嘅",
            "错的": "錯嘅",
            "漂亮": "靚",
            "好看": "好睇",
            "好吃": "好食",
            "厉害": "犀利",
            "厉害": "叻",

            # 助词
            "的": "嘅",
            "了": "喇",
            "吗": "嘛",
            "呢": "呢",
            "啊": "呀",
            "吧": "啦",

            # 礼貌用语
            "谢谢": "唔該",
            "请": "唔該",
            "对不起": "對唔住",
            "不好意思": "唔好意思",
            "再见": "拜拜",
            "你好": "你好",
            "早安": "早晨",
            "晚安": "夜晚好",

            # 时间表达
            "现在": "而家",
            "然后": "然後",
            "后来": "後嚟",
            "马上": "即刻",
            "等一下": "等一等",
            "一会儿": "一陣間",

            # 数量词
            "一个": "一個",
            "一些": "啲",
            "所有": "所有",
            "很多": "好多",
            "很少": "好少",
            "没有": "冇",

            # 疑问词
            "吗": "嘛",
            "呢": "呢",
            "吧": "呀",
            "呢": "呢",
            "吗": "嘛",
            "呀": "呀",
        }

        # 粤语特殊表达
        self.special_expressions = {
            # 问句模式
            "是吗": "係嘛",
            "对吗": "啱嘛",
            "你知道吗": "你知唔知",
            "明白吗": "明唔明",
            "可以吗": "可以呀",

            # 回应模式
            "好的": "好嘅",
            "是的": "係嘅",
            "当然": "當然",
            "没错": "冇錯",
            "是的呢": "係呀",
            "没问题": "冇問題",

            # 情感表达
            "很开心": "好開心",
            "很生气": "好嬲",
            "很失望": "好失望",
            "很惊讶": "好驚訝",
            "很担心": "好擔心",

            # 生活常用
            "吃饭": "食飯",
            "睡觉": "瞓覺",
            "洗澡": "沖涼",
            "上班": "返工",
            "下班": "放工",
            "回家": "返屋企",
            "出门": "出門",
            "逛街": "行街",
            "购物": "買嘢",
        }

        # 需要保留的专有名词（不转换）
        self.proper_nouns = {
            "阿里云", "腾讯", "华为", "小米",
            "XleRobot", "Robot", "AI", "API",
            "Python", "JavaScript", "Java", "C++",
            "Windows", "Linux", "Android", "iOS",
        }

        # 预处理统计
        self.preprocessing_stats = {
            "total_processed": 0,
            "terms_converted": 0,
            "expressions_converted": 0,
            "length_reduced": 0
        }

        logger.info("✅ 粤语预处理器初始化完成")

    def preprocess_text(self, text: str) -> str:
        """
        预处理粤语文本
        仅进行简单的术语转换，不进行任何本地对话逻辑

        Args:
            text: 原始文本

        Returns:
            str: 预处理后的文本
        """
        if not text:
            return text

        self.preprocessing_stats["total_processed"] += 1
        original_length = len(text)

        processed_text = text

        # 1. 标准术语转换
        processed_text = self._convert_standard_terms(processed_text)

        # 2. 特殊表达转换
        processed_text = self._convert_special_expressions(processed_text)

        # 3. 清理多余空格
        processed_text = self._clean_whitespace(processed_text)

        # 4. 长度检查（API限制）
        processed_text = self._check_length_limit(processed_text)

        # 更新统计信息
        self.preprocessing_stats["length_reduced"] += original_length - len(processed_text)

        logger.debug(f"📝 文本预处理: {text[:20]}... → {processed_text[:20]}...")

        return processed_text

    def _convert_standard_terms(self, text: str) -> str:
        """
        转换标准粤语术语
        """
        converted_text = text
        converted_count = 0

        for standard, cantonese in self.standard_terms.items():
            if standard in converted_text:
                converted_text = converted_text.replace(standard, cantonese)
                converted_count += 1

        self.preprocessing_stats["terms_converted"] += converted_count
        return converted_text

    def _convert_special_expressions(self, text: str) -> str:
        """
        转换特殊粤语表达
        """
        converted_text = text
        converted_count = 0

        for expression, cantonese in self.special_expressions.items():
            if expression in converted_text:
                converted_text = converted_text.replace(expression, cantonese)
                converted_count += 1

        self.preprocessing_stats["expressions_converted"] += converted_count
        return converted_text

    def _clean_whitespace(self, text: str) -> str:
        """
        清理多余空格和标点
        """
        # 移除多余空格
        text = re.sub(r'\s+', ' ', text.strip())

        # 清理多余标点
        text = re.sub(r'[。，！？]{2,}', lambda m: m.group(0)[0], text)

        return text

    def _check_length_limit(self, text: str) -> str:
        """
        检查文本长度限制（API要求）
        大多数API限制在2000字符以内
        """
        max_length = 2000

        if len(text) > max_length:
            truncated = text[:max_length - 3] + "..."
            logger.warning(f"⚠️ 文本过长，已截断: {len(text)} → {len(truncated)}")
            return truncated

        return text

    def get_preprocessing_statistics(self) -> Dict[str, any]:
        """
        获取预处理统计信息

        Returns:
            Dict[str, any]: 统计信息
        """
        return self.preprocessing_stats.copy()

    def reset_statistics(self):
        """重置统计信息"""
        self.preprocessing_stats = {
            "total_processed": 0,
            "terms_converted": 0,
            "expressions_converted": 0,
            "length_reduced": 0
        }
        logger.info("📊 预处理统计信息已重置")

    def validate_cantonese_text(self, text: str) -> Dict[str, bool]:
        """
        验证粤语文本质量

        Args:
            text: 待验证文本

        Returns:
            Dict[str, bool]: 验证结果
        """
        return {
            "has_cantonese_characters": bool(re.search(r'[粤語繁简]', text)),
            "uses_cantonese_terms": any(term in text for term in self.standard_terms.values()),
            "appropriate_length": 10 <= len(text) <= 500,
            "no_consecutive_punctuation": not re.search(r'[。，！？]{2,}', text),
            "proper_terminology": not any(pnoun in text for pnoun in self.proper_nouns.keys() if pnoun in text.lower())
        }

# 全局预处理器实例
_global_preprocessor = None

def get_cantonese_preprocessor() -> CantoneseTextProcessor:
    """
    获取全局粤语预处理器实例（单例模式）
    """
    global _global_preprocessor

    if _global_preprocessor is None:
        _global_preprocessor = CantoneseTextProcessor()

    return _global_preprocessor