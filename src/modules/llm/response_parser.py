#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 2.1: 通义千问API集成 - 响应解析器

API响应解析和处理模块，实现响应格式解析、结果提取、错误处理和格式化输出。
支持多种输出格式、文本清洗、语言检测和内容分析。

作者: Dev Agent
故事ID: Story 2.1
Epic: 2 - 智能对话模块
"""

import re
import json
import logging
from typing import Dict, Any, List, Optional, Union, Tuple
from dataclasses import dataclass
from enum import Enum
import jieba
import jieba.posseg as pseg
from langdetect import detect, LangDetectException

from .qwen_client import QwenResponse


logger = logging.getLogger(__name__)


class OutputFormat(Enum):
    """输出格式"""
    PLAIN = "plain"              # 纯文本
    MARKDOWN = "markdown"        # Markdown格式
    JSON = "json"                # JSON格式
    CONVERSATION = "conversation"  # 对话格式
    STRUCTURED = "structured"    # 结构化数据


class Language(Enum):
    """语言类型"""
    CHINESE = "zh"
    ENGLISH = "en"
    CANTONESE = "yue"  # 粤语
    MIXED = "mixed"
    UNKNOWN = "unknown"


@dataclass
class ParsedResponse:
    """解析后的响应对象"""
    text: str
    language: Language
    confidence: float
    word_count: int
    sentence_count: int
    has_code: bool
    has_links: bool
    sentiment: str
    topics: List[str]
    entities: List[Dict[str, str]]
    metadata: Dict[str, Any]
    raw_response: QwenResponse
    formatted_output: str
    output_format: OutputFormat


class ResponseParser:
    """
    API响应解析器

    功能特性:
    - 响应内容解析
    - 语言检测
    - 文本分析和清洗
    - 实体识别
    - 主题提取
    - 情感分析
    - 多格式输出
    - 错误处理
    """

    def __init__(self):
        """初始化响应解析器"""
        # 初始化分词工具
        jieba.initialize()

        # 常用停用词
        self.stop_words = {
            '的', '了', '是', '在', '我', '有', '和', '就', '不', '人', '都', '一', '个', '上', '也', '很', '到', '说', '要', '去', '你', '会', '着', '没有', '看', '好', '自己', '这', '那', '里', '后', '以', '所', '如果', '因为', '但是'
        }

        # 语言关键词
        self.chinese_keywords = {
            '的', '了', '是', '在', '我', '有', '和', '就', '不', '人', '都', '一', '个', '上', '也', '很', '到', '说', '要', '去', '你', '会', '着', '没有', '看', '好', '自己', '这', '那', '里', '后', '以', '所', '如果', '因为', '但是', '么', '啦', '呀', '嘛', '呢'
        }

        self.cantonese_keywords = {
            '嘅', '咗', '啦', '喇', '嘛', '呢', '咯', '啫', '咋', '咪', '唔', '冇', '佢', '嘅', '嘅', '之', '咁', '咁样', '咁嘅', '边度', '边个', '咩', '乜', '边', '呢', '嗰', '嗻', '喺', '嚟', '去', '番', '嚟', '返', '去', '嚟', '番', '嚟', '返', '去', '嚟', '番', '嚟', '返', '去'
        }

        logger.info("✅ 响应解析器初始化完成")
        logger.info("   - 语言检测: 支持中文、粤语、英文")
        logger.info("   - 文本分析: 分词、词性标注、停用词过滤")
        logger.info("   - 输出格式: 支持多种格式化输出")

    def parse_response(
        self,
        response: QwenResponse,
        output_format: OutputFormat = OutputFormat.PLAIN,
        language: Optional[Language] = None,
        include_metadata: bool = True
    ) -> ParsedResponse:
        """
        解析API响应

        Args:
            response: 通义千问响应对象
            output_format: 输出格式
            language: 指定语言（可选）
            include_metadata: 是否包含元数据

        Returns:
            ParsedResponse: 解析后的响应对象
        """
        try:
            # 基本解析
            raw_text = response.text.strip()

            # 语言检测
            detected_language = self._detect_language(raw_text, language)

            # 文本分析
            word_count = self._count_words(raw_text, detected_language)
            sentence_count = self._count_sentences(raw_text, detected_language)

            # 内容特征检测
            has_code = self._detect_code(raw_text)
            has_links = self._detect_links(raw_text)

            # 主题和实体提取
            topics = self._extract_topics(raw_text, detected_language)
            entities = self._extract_entities(raw_text, detected_language)

            # 情感分析
            sentiment = self._analyze_sentiment(raw_text, detected_language)

            # 置信度计算
            confidence = self._calculate_confidence(response)

            # 格式化输出
            formatted_output = self._format_output(
                raw_text, output_format, detected_language
            )

            # 构建元数据
            metadata = {}
            if include_metadata:
                metadata = {
                    'model': response.model,
                    'usage': response.usage,
                    'finish_reason': response.finish_reason,
                    'request_id': response.request_id,
                    'processing_time': self._estimate_processing_time(response),
                    'content_features': {
                        'word_count': word_count,
                        'sentence_count': sentence_count,
                        'has_code': has_code,
                        'has_links': has_links,
                        'language': detected_language.value
                    }
                }

            parsed_response = ParsedResponse(
                text=raw_text,
                language=detected_language,
                confidence=confidence,
                word_count=word_count,
                sentence_count=sentence_count,
                has_code=has_code,
                has_links=has_links,
                sentiment=sentiment,
                topics=topics,
                entities=entities,
                metadata=metadata,
                raw_response=response,
                formatted_output=formatted_output,
                output_format=output_format
            )

            logger.info(f"✅ 响应解析完成")
            logger.info(f"   - 语言: {detected_language.value}")
            logger.info(f"   - 词数: {word_count}")
            logger.info(f"   - 句子数: {sentence_count}")
            logger.info(f"   - 格式: {output_format.value}")

            return parsed_response

        except Exception as e:
            logger.error(f"❌ 响应解析失败: {e}")
            raise ValueError(f"无法解析响应: {e}")

    def _detect_language(
        self,
        text: str,
        specified_language: Optional[Language] = None
    ) -> Language:
        """检测语言类型"""
        if specified_language:
            return specified_language

        # 预处理文本
        text = text.lower().strip()

        # 检查是否包含中文关键词
        chinese_count = sum(1 for word in self.chinese_keywords if word in text)
        cantonese_count = sum(1 for word in self.cantonese_keywords if word in text)

        # 检查是否包含英文
        english_pattern = re.compile(r'[a-zA-Z]{3,}')
        english_matches = english_pattern.findall(text)

        # 判断语言
        if cantonese_count > chinese_count and cantonese_count > 0:
            return Language.CANTONESE
        elif chinese_count > 0:
            return Language.CHINESE
        elif len(english_matches) > len(text.split()) * 0.5:
            return Language.ENGLISH
        elif chinese_count > 0 and len(english_matches) > 0:
            return Language.MIXED
        else:
            try:
                detected = detect(text)
                if detected == 'zh':
                    return Language.CHINESE
                elif detected == 'en':
                    return Language.ENGLISH
                else:
                    return Language.UNKNOWN
            except LangDetectException:
                return Language.UNKNOWN

    def _count_words(self, text: str, language: Language) -> int:
        """计算词数"""
        if language == Language.CHINESE or language == Language.CANTONESE:
            # 中文使用结巴分词
            words = jieba.cut(text)
            words = [w for w in words if w.strip() and w not in self.stop_words]
            return len(words)
        else:
            # 英文按空格分词
            words = text.split()
            words = [w for w in words if w.strip() and w.lower() not in self.stop_words]
            return len(words)

    def _count_sentences(self, text: str, language: Language) -> int:
        """计算句子数"""
        if language == Language.CHINESE or language == Language.CANTONESE:
            # 中文按句号、问号、感叹号分割
            sentences = re.split(r'[。！？]', text)
            return len([s for s in sentences if s.strip()])
        else:
            # 英文按句号、问号、感叹号分割
            sentences = re.split(r'[.!?]+', text)
            return len([s for s in sentences if s.strip()])

    def _detect_code(self, text: str) -> bool:
        """检测是否包含代码"""
        code_patterns = [
            r'```[\s\S]*?```',  # 代码块
            r'`[^`]+`',        # 行内代码
            r'def\s+\w+\(',    # Python函数定义
            r'class\s+\w+',    # 类定义
            r'import\s+\w+',   # 导入语句
            r'function\s*\([^)]*\)',  # 函数调用
            r'console\.log\(', # JavaScript
            r'System\.out\.println'  # Java
        ]

        for pattern in code_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return True

        return False

    def _detect_links(self, text: str) -> bool:
        """检测是否包含链接"""
        url_pattern = re.compile(
            r'http[s]?://(?:[a-zA-Z]|[0-9]|[$-_@.&+]|[!*\\(\\),]|(?:%[0-9a-fA-F][0-9a-fA-F]))+'
        )
        return bool(url_pattern.search(text))

    def _extract_topics(self, text: str, language: Language) -> List[str]:
        """提取主题词"""
        topics = []

        if language == Language.CHINESE or language == Language.CANTONESE:
            # 中文使用结巴分词
            words = pseg.cut(text)
            for word, flag in words:
                if (flag.startswith('n') or flag.startswith('v')) and len(word) > 1:
                    if word not in self.stop_words and not re.match(r'^[0-9]+$', word):
                        topics.append(word)
        else:
            # 英文按词性提取
            words = text.split()
            for word in words:
                # 简单的名词提取（首字母大写的词或包含特定后缀的词）
                if (word[0].isupper() or word.endswith(('tion', 'ness', 'ment', 'ity', 'ism'))
                    and word.lower() not in self.stop_words):
                    topics.append(word)

        # 去重并返回前10个
        return list(dict.fromkeys(topics))[:10]

    def _extract_entities(self, text: str, language: Language) -> List[Dict[str, str]]:
        """提取命名实体"""
        entities = []

        if language == Language.CHINESE or language == Language.CANTONESE:
            # 中文实体识别
            words = pseg.cut(text)
            for word, flag in words:
                if flag.startswith('nr'):  # 人名
                    entities.append({'type': 'person', 'text': word})
                elif flag.startswith('ns'):  # 地名
                    entities.append({'type': 'location', 'text': word})
                elif flag.startswith('nt'):  # 机构名
                    entities.append({'type': 'organization', 'text': word})
        else:
            # 简单的英文实体识别
            # 匹配大写开头的词（人名、地名、机构名）
            entity_patterns = [
                (r'\b[A-Z][a-z]+\b', 'person'),
                (r'\b(?:Inc|LLC|Corp|Ltd|Corporation|Company)\b', 'organization'),
                (r'\b(?:New York|Los Angeles|Chicago|London|Paris|Tokyo)\b', 'location')
            ]

            for pattern, entity_type in entity_patterns:
                matches = re.findall(pattern, text)
                for match in matches:
                    entities.append({'type': entity_type, 'text': match})

        return entities

    def _analyze_sentiment(self, text: str, language: Language) -> str:
        """情感分析"""
        positive_words = {
            'zh': ['好', '棒', '优秀', '喜欢', '高兴', '满意', '赞', '不错', '厉害', '完美'],
            'en': ['good', 'great', 'excellent', 'amazing', 'wonderful', 'fantastic', 'perfect', 'love', 'like', 'happy']
        }

        negative_words = {
            'zh': ['坏', '差', '糟糕', '讨厌', '生气', '不满意', '烂', '差劲', '不行', '糟糕'],
            'en': ['bad', 'terrible', 'awful', 'hate', 'angry', 'sad', 'disgusting', 'horrible', 'worst', 'dislike']
        }

        # 根据语言选择词典
        if language == Language.CHINESE or language == Language.CANTONESE:
            pos_words = positive_words['zh']
            neg_words = negative_words['zh']
        else:
            pos_words = positive_words['en']
            neg_words = negative_words['en']

        text_lower = text.lower()

        pos_count = sum(1 for word in pos_words if word in text_lower)
        neg_count = sum(1 for word in neg_words if word in text_lower)

        if pos_count > neg_count:
            return 'positive'
        elif neg_count > pos_count:
            return 'negative'
        else:
            return 'neutral'

    def _calculate_confidence(self, response: QwenResponse) -> float:
        """计算置信度"""
        # 基于响应长度、使用情况等计算置信度
        text_length = len(response.text)

        # 长度得分 (0-0.3)
        length_score = min(0.3, text_length / 1000)

        # 使用情况得分 (0-0.4)
        usage = response.usage
        if usage:
            input_tokens = usage.get('input_tokens', 0)
            output_tokens = usage.get('output_tokens', 0)
            total_tokens = input_tokens + output_tokens
            usage_score = min(0.4, total_tokens / 10000)
        else:
            usage_score = 0.2

        # 结束原因得分 (0-0.3)
        if response.finish_reason == 'stop':
            reason_score = 0.3
        elif response.finish_reason == 'length':
            reason_score = 0.2
        else:
            reason_score = 0.1

        total_confidence = length_score + usage_score + reason_score

        # 限制在0-1范围内
        return min(1.0, max(0.0, total_confidence))

    def _estimate_processing_time(self, response: QwenResponse) -> float:
        """估算处理时间"""
        # 基于tokens数量估算处理时间
        usage = response.usage
        if usage:
            total_tokens = usage.get('input_tokens', 0) + usage.get('output_tokens', 0)
            # 假设每1000 tokens需要1秒处理时间
            estimated_time = total_tokens / 1000
            return round(estimated_time, 2)
        return 0.0

    def _format_output(
        self,
        text: str,
        output_format: OutputFormat,
        language: Language
    ) -> str:
        """格式化输出"""
        if output_format == OutputFormat.PLAIN:
            return text.strip()

        elif output_format == OutputFormat.MARKDOWN:
            # 基本Markdown格式化
            formatted = text

            # 检测代码块
            if '```' not in formatted:
                formatted = formatted.replace('```', '\n```')

            # 检测标题
            if language == Language.CHINESE or language == Language.CANTONESE:
                formatted = re.sub(r'^(.+?)[。！？]', r'## \1', formatted, flags=re.MULTILINE)
            else:
                formatted = re.sub(r'^(.+?)[.!?]', r'## \1', formatted, flags=re.MULTILINE)

            return formatted

        elif output_format == OutputFormat.JSON:
            # JSON格式化
            return json.dumps({
                'text': text,
                'language': language.value,
                'formatted': True
            }, ensure_ascii=False, indent=2)

        elif output_format == OutputFormat.CONVERSATION:
            # 对话格式
            return f"🤖 助手: {text}"

        elif output_format == OutputFormat.STRUCTURED:
            # 结构化输出
            lines = text.split('\n')
            structured = []

            for line in lines:
                line = line.strip()
                if line:
                    structured.append(f"- {line}")

            return '\n'.join(structured)

        else:
            return text

    def batch_parse(
        self,
        responses: List[QwenResponse],
        output_format: OutputFormat = OutputFormat.PLAIN
    ) -> List[ParsedResponse]:
        """
        批量解析响应

        Args:
            responses: 响应对象列表
            output_format: 输出格式

        Returns:
            List[ParsedResponse]: 解析后的响应列表
        """
        results = []

        for response in responses:
            try:
                parsed = self.parse_response(response, output_format)
                results.append(parsed)
            except Exception as e:
                logger.error(f"❌ 批量解析中出错: {e}")
                # 创建错误响应
                error_parsed = ParsedResponse(
                    text="",
                    language=Language.UNKNOWN,
                    confidence=0.0,
                    word_count=0,
                    sentence_count=0,
                    has_code=False,
                    has_links=False,
                    sentiment="neutral",
                    topics=[],
                    entities=[],
                    metadata={'error': str(e)},
                    raw_response=response,
                    formatted_output="解析失败",
                    output_format=output_format
                )
                results.append(error_parsed)

        logger.info(f"✅ 批量解析完成: {len(results)}个响应")
        return results

    def validate_response(self, response: QwenResponse) -> Tuple[bool, str]:
        """
        验证响应有效性

        Args:
            response: 响应对象

        Returns:
            Tuple[bool, str]: (是否有效, 错误信息)
        """
        if not response.text:
            return False, "响应文本为空"

        if len(response.text.strip()) < 1:
            return False, "响应文本长度不足"

        if response.confidence < 0.0 or response.confidence > 1.0:
            return False, "置信度值无效"

        return True, "响应有效"


# ROS2节点集成
class ResponseParserNode:
    """响应解析器ROS2节点"""

    def __init__(self, node):
        """
        初始化解析器节点

        Args:
            node: ROS2节点实例
        """
        self.parser = ResponseParser()
        self.node = node

    def parse_and_publish(self, response: QwenResponse) -> ParsedResponse:
        """
        解析响应并发布结果

        Args:
            response: 通义千问响应对象

        Returns:
            ParsedResponse: 解析后的响应对象
        """
        try:
            parsed = self.parser.parse_response(response)

            self.node.get_logger().info(f"📝 解析响应完成")
            self.node.get_logger().info(f"   - 语言: {parsed.language.value}")
            self.node.get_logger().info(f"   - 词数: {parsed.word_count}")
            self.node.get_logger().info(f"   - 主题: {len(parsed.topics)}个")

            return parsed

        except Exception as e:
            self.node.get_logger().error(f"❌ 响应解析失败: {e}")
            raise


if __name__ == '__main__':
    # 示例用法
    from .qwen_client import QwenResponse

    # 创建测试响应
    test_response = QwenResponse(
        text="你好！我是一个AI助手，很高兴为你服务。我可以帮助你解答问题、提供建议，或者进行对话。有什么我可以帮助你的吗？",
        model="qwen3-vl-plus",
        usage={"input_tokens": 10, "output_tokens": 50},
        finish_reason="stop",
        request_id="test-123"
    )

    # 初始化解析器
    parser = ResponseParser()

    # 解析响应
    parsed = parser.parse_response(test_response, OutputFormat.CONVERSATION)

    print(f"📝 原始文本: {parsed.text}")
    print(f"🌍 语言: {parsed.language.value}")
    print(f"📊 置信度: {parsed.confidence:.2f}")
    print(f"📏 词数: {parsed.word_count}")
    print(f"📑 句子数: {parsed.sentence_count}")
    print(f"💭 情感: {parsed.sentiment}")
    print(f"🏷️ 主题: {parsed.topics}")
    print(f"🔖 实体: {parsed.entities}")
    print(f"📄 格式化输出:\n{parsed.formatted_output}")
    print(f"📊 元数据: {json.dumps(parsed.metadata, indent=2, ensure_ascii=False)}")
