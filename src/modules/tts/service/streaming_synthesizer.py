#!/usr/bin/env python3
"""
流式语音合成器服务
==================

基于WebSocket TTS的高级流式合成服务，提供：
- 文本分块和流式处理
- 智能音频缓存
- 边合成边播放
- 性能优化和错误恢复

作者: Developer Agent
版本: 1.0
日期: 2025-11-16
"""

import logging
import time
import asyncio
import threading
from typing import List, Dict, Any, Optional, Callable
from dataclasses import dataclass
from datetime import datetime

from ..engine.aliyun_tts_websocket import WebSocketTTSService, StreamingTTSResult, TTSResult

logger = logging.getLogger(__name__)

@dataclass
class StreamingRequest:
    """流式合成请求"""
    text: str
    voice: str = "xiaoyan"
    volume: int = 50
    rate: int = 0
    pitch: int = 0
    format: str = "pcm"
    sample_rate: int = 16000
    chunk_size: int = 100  # 字符分块大小
    overlap_size: int = 20  # 重叠字符数
    callback: Optional[Callable] = None

@dataclass
class StreamingSession:
    """流式合成会话"""
    session_id: str
    request: StreamingRequest
    start_time: float
    chunks_processed: int = 0
    total_chunks: int = 0
    audio_chunks: List[bytes] = None
    is_completed: bool = False
    error: str = ""

    def __post_init__(self):
        if self.audio_chunks is None:
            self.audio_chunks = []

class StreamingSynthesizer:
    """流式语音合成器"""

    def __init__(self, tts_service: WebSocketTTSService):
        """
        初始化流式合成器

        Args:
            tts_service: WebSocket TTS服务实例
        """
        self.tts_service = tts_service
        self.active_sessions: Dict[str, StreamingSession] = {}
        self.session_counter = 0

        # 性能配置
        self.max_concurrent_sessions = 5
        self.default_chunk_size = 100
        self.default_overlap = 20

        logger.info("✅ 流式语音合成器初始化完成")

    def _generate_session_id(self) -> str:
        """生成会话ID"""
        self.session_counter += 1
        timestamp = int(time.time() * 1000)
        return f"stream_{timestamp}_{self.session_counter}"

    def _split_text_into_chunks(self, text: str, chunk_size: int, overlap: int) -> List[str]:
        """
        将文本分割成重叠的块

        Args:
            text: 原始文本
            chunk_size: 块大小
            overlap: 重叠大小

        Returns:
            文本块列表
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = min(start + chunk_size, len(text))
            chunk = text[start:end]
            chunks.append(chunk)

            # 计算下一个起始位置（考虑重叠）
            if end >= len(text):
                break
            start = end - overlap

        return chunks

    async def synthesize_streaming(self, request: StreamingRequest) -> StreamingSession:
        """
        异步流式语音合成

        Args:
            request: 流式合成请求

        Returns:
            StreamingSession: 合成会话
        """
        session_id = self._generate_session_id()
        session = StreamingSession(
            session_id=session_id,
            request=request,
            start_time=time.time()
        )

        # 检查并发限制
        if len(self.active_sessions) >= self.max_concurrent_sessions:
            session.error = "达到最大并发会话限制"
            session.is_completed = True
            return session

        self.active_sessions[session_id] = session

        try:
            # 分割文本
            text_chunks = self._split_text_into_chunks(
                request.text,
                request.chunk_size or self.default_chunk_size,
                request.overlap_size or self.default_overlap
            )

            session.total_chunks = len(text_chunks)
            logger.info(f"🌊 开始流式合成: {session_id}, 总块数: {len(text_chunks)}")

            # 逐块合成
            for i, chunk in enumerate(text_chunks):
                if session.error:  # 如果有错误则停止
                    break

                logger.debug(f"🎵 合成块 {i+1}/{len(text_chunks)}: '{chunk[:20]}...'")

                # 调用TTS服务
                result = self.tts_service.synthesize_streaming(
                    text=chunk,
                    voice=request.voice,
                    volume=request.volume,
                    rate=request.rate,
                    pitch=request.pitch,
                    format=request.format,
                    sample_rate=request.sample_rate
                )

                if result.success:
                    session.audio_chunks.extend(result.audio_chunks)
                    session.chunks_processed += 1

                    # 调用回调函数
                    if request.callback:
                        try:
                            await request.callback(session, i, result)
                        except Exception as e:
                            logger.warning(f"⚠️ 回调函数执行失败: {e}")
                else:
                    session.error = result.error
                    logger.error(f"❌ 块 {i+1} 合成失败: {result.error}")
                    break

            session.is_completed = True
            session_time = time.time() - session.start_time
            logger.info(f"✅ 流式合成完成: {session_id}, 耗时: {session_time:.2f}s")

        except Exception as e:
            session.error = f"流式合成异常: {e}"
            session.is_completed = True
            logger.error(f"❌ 流式合成异常: {e}")

        finally:
            # 清理会话
            if session_id in self.active_sessions:
                del self.active_sessions[session_id]

        return session

    def synthesize_streaming_sync(self, request: StreamingRequest) -> StreamingSession:
        """
        同步流式语音合成

        Args:
            request: 流式合成请求

        Returns:
            StreamingSession: 合成会话
        """
        # 运行异步函数
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            return loop.run_until_complete(self.synthesize_streaming(request))
        finally:
            loop.close()

    def get_session_status(self, session_id: str) -> Optional[StreamingSession]:
        """
        获取会话状态

        Args:
            session_id: 会话ID

        Returns:
            StreamingSession或None
        """
        return self.active_sessions.get(session_id)

    def cancel_session(self, session_id: str) -> bool:
        """
        取消会话

        Args:
            session_id: 会话ID

        Returns:
            是否成功取消
        """
        if session_id in self.active_sessions:
            session = self.active_sessions[session_id]
            session.error = "用户取消"
            session.is_completed = True
            del self.active_sessions[session_id]
            logger.info(f"🛑 会话已取消: {session_id}")
            return True
        return False

    def get_active_sessions_count(self) -> int:
        """获取活跃会话数量"""
        return len(self.active_sessions)

    def get_performance_stats(self) -> Dict[str, Any]:
        """获取性能统计"""
        return {
            "active_sessions": len(self.active_sessions),
            "max_concurrent_sessions": self.max_concurrent_sessions,
            "total_sessions_created": self.session_counter,
            "default_chunk_size": self.default_chunk_size,
            "default_overlap": self.default_overlap
        }

class TextChunker:
    """文本分块器"""

    @staticmethod
    def chunk_by_sentences(text: str, max_chunk_length: int = 200) -> List[str]:
        """
        按句子分块

        Args:
            text: 文本
            max_chunk_length: 最大块长度

        Returns:
            句子块列表
        """
        import re

        # 中文句子分隔符
        sentence_endings = r'[。！？；…]'
        sentences = re.split(sentence_endings, text)

        chunks = []
        current_chunk = ""

        for sentence in sentences:
            if not sentence.strip():
                continue

            # 如果加上这个句子会超过长度限制
            if len(current_chunk + sentence) > max_chunk_length and current_chunk:
                chunks.append(current_chunk.strip())
                current_chunk = sentence
            else:
                current_chunk += sentence + "。"

        # 添加最后一个块
        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    @staticmethod
    def chunk_by_punctuation(text: str, max_chunk_length: int = 150) -> List[str]:
        """
        按标点符号分块

        Args:
            text: 文本
            max_chunk_length: 最大块长度

        Returns:
            标点块列表
        """
        import re

        # 标点符号
        punctuation = r'[，。！？；：、""''（）【】《》…—]'
        segments = re.split(punctuation, text)

        chunks = []
        current_chunk = ""

        for segment in segments:
            if not segment.strip():
                continue

            if len(current_chunk + segment) > max_chunk_length and current_chunk:
                chunks.append(current_chunk.strip())
                current_chunk = segment
            else:
                current_chunk += segment + "，"

        if current_chunk.strip():
            chunks.append(current_chunk.strip())

        return chunks

    @staticmethod
    def chunk_smart(text: str, target_length: int = 100) -> List[str]:
        """
        智能分块（优先按句子，其次按长度）

        Args:
            text: 文本
            target_length: 目标长度

        Returns:
            智能块列表
        """
        # 首先尝试按句子分块
        sentence_chunks = TextChunker.chunk_by_sentences(text, target_length * 2)

        # 如果句子块太大，进一步分割
        final_chunks = []
        for chunk in sentence_chunks:
            if len(chunk) <= target_length:
                final_chunks.append(chunk)
            else:
                # 按长度分割
                length_chunks = TextChunker.chunk_by_length(chunk, target_length, 10)
                final_chunks.extend(length_chunks)

        return final_chunks

    @staticmethod
    def chunk_by_length(text: str, chunk_size: int, overlap: int = 10) -> List[str]:
        """
        按长度分块

        Args:
            text: 文本
            chunk_size: 块大小
            overlap: 重叠大小

        Returns:
            长度块列表
        """
        if len(text) <= chunk_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = min(start + chunk_size, len(text))
            chunk = text[start:end]
            chunks.append(chunk)

            if end >= len(text):
                break
            start = end - overlap

        return chunks

def create_streaming_synthesizer(tts_service: WebSocketTTSService) -> StreamingSynthesizer:
    """创建流式合成器实例"""
    return StreamingSynthesizer(tts_service)