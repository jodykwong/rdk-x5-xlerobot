#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 1.4: 连续语音识别 - 会话管理器
Session Manager for Continuous Speech Recognition

负责管理多个并发语音识别会话，支持30分钟连续对话。
功能特性:
- 多会话并发支持
- 会话状态管理 (INITIALIZING, ACTIVE, PAUSED, SUSPENDED, TERMINATED)
- 上下文保持和管理
- 自动资源分配和回收

作者: Dev Agent
故事ID: Story 1.4
"""

import uuid
import time
import threading
from typing import Dict, Optional, List, Any
from enum import Enum, auto
from dataclasses import dataclass, field
from queue import Queue, Empty
import logging
from pathlib import Path
import json

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SessionState(Enum):
    """会话状态枚举"""
    INITIALIZING = "INITIALIZING"  # 初始化中
    ACTIVE = "ACTIVE"              # 活跃状态
    PAUSED = "PAUSED"              # 暂停状态
    SUSPENDED = "SUSPENDED"        # 挂起状态 (错误或系统问题)
    TERMINATED = "TERMINATED"      # 已终止


@dataclass
class SessionContext:
    """会话上下文信息"""
    session_id: str
    start_time: float
    last_activity_time: float
    audio_frames_processed: int
    total_audio_duration: float
    transcription_history: List[dict] = field(default_factory=list)
    wake_word_detections: int = 0
    last_wake_word_confidence: float = 0.0
    state: SessionState = SessionState.INITIALIZING
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class SessionStats:
    """会话统计信息"""
    session_id: str
    duration_seconds: float
    frames_processed: int
    wake_word_count: int
    max_confidence: float
    avg_confidence: float
    state: SessionState
    throughput: float  # frames per second


class SessionManager:
    """
    会话管理器

    功能特性:
    - 创建和管理多个并发语音识别会话
    - 会话状态生命周期管理
    - 上下文保持和传递
    - 资源监控和自动清理
    - 性能统计和分析
    """

    def __init__(self,
                 max_sessions: int = 10,
                 session_timeout_minutes: int = 30,
                 enable_monitoring: bool = True):
        """
        初始化会话管理器

        Args:
            max_sessions: 最大并发会话数
            session_timeout_minutes: 会话超时时间 (分钟)
            enable_monitoring: 是否启用监控
        """
        self.max_sessions = max_sessions
        self.session_timeout_seconds = session_timeout_minutes * 60
        self.enable_monitoring = enable_monitoring

        # 会话存储
        self._sessions: Dict[str, SessionContext] = {}
        self._session_locks: Dict[str, threading.Lock] = {}

        # 统计信息
        self._stats_lock = threading.Lock()
        self.global_stats = {
            'total_sessions_created': 0,
            'total_sessions_terminated': 0,
            'total_wake_word_detections': 0,
            'total_audio_frames_processed': 0,
            'peak_concurrent_sessions': 0,
            'average_session_duration': 0.0
        }

        # 监控线程
        self._monitoring_active = False
        self._monitoring_thread: Optional[threading.Thread] = None

        # 线程安全
        self._global_lock = threading.RLock()

        # 启动监控
        if self.enable_monitoring:
            self._start_monitoring()

        logger.info(f"会话管理器初始化完成: 最大会话数={max_sessions}, "
                   f"超时时间={session_timeout_minutes}分钟")

    def create_session(self, session_id: Optional[str] = None,
                      metadata: Optional[Dict[str, Any]] = None) -> str:
        """
        创建新的语音识别会话

        Args:
            session_id: 会话ID (可选，如果为None则自动生成)
            metadata: 会话元数据

        Returns:
            会话ID

        Raises:
            ValueError: 如果超过最大会话数或会话ID已存在
        """
        with self._global_lock:
            # 检查是否超过最大会话数
            if len(self._sessions) >= self.max_sessions:
                raise ValueError(f"已达到最大会话数限制: {self.max_sessions}")

            # 生成或验证会话ID
            if session_id is None:
                session_id = str(uuid.uuid4())
            elif session_id in self._sessions:
                raise ValueError(f"会话ID已存在: {session_id}")

            # 创建会话
            current_time = time.time()
            session = SessionContext(
                session_id=session_id,
                start_time=current_time,
                last_activity_time=current_time,
                audio_frames_processed=0,
                total_audio_duration=0.0,
                metadata=metadata or {}
            )

            # 存储会话和锁
            self._sessions[session_id] = session
            self._session_locks[session_id] = threading.Lock()

            # 更新统计信息
            with self._stats_lock:
                self.global_stats['total_sessions_created'] += 1
                self.global_stats['peak_concurrent_sessions'] = max(
                    self.global_stats['peak_concurrent_sessions'],
                    len(self._sessions)
                )

            logger.info(f"✅ 会话创建成功: {session_id} (总共{len(self._sessions)}个活跃会话)")

            return session_id

    def destroy_session(self, session_id: str) -> bool:
        """
        销毁会话

        Args:
            session_id: 会话ID

        Returns:
            是否成功销毁
        """
        with self._global_lock:
            if session_id not in self._sessions:
                logger.warning(f"尝试销毁不存在的会话: {session_id}")
                return False

            try:
                # 获取会话
                session = self._sessions[session_id]

                # 更新状态
                with self._session_locks[session_id]:
                    session.state = SessionState.TERMINATED
                    session.last_activity_time = time.time()

                # 记录统计信息
                duration = session.last_activity_time - session.start_time
                with self._stats_lock:
                    self.global_stats['total_sessions_terminated'] += 1
                    # 更新平均会话时长
                    if self.global_stats['total_sessions_terminated'] == 1:
                        self.global_stats['average_session_duration'] = duration
                    else:
                        # 移动平均
                        n = self.global_stats['total_sessions_terminated']
                        old_avg = self.global_stats['average_session_duration']
                        self.global_stats['average_session_duration'] = (
                            (old_avg * (n - 1) + duration) / n
                        )

                # 清理资源
                del self._sessions[session_id]
                del self._session_locks[session_id]

                logger.info(f"🛑 会话已销毁: {session_id} (持续时间: {duration:.1f}秒, "
                           f"处理帧数: {session.audio_frames_processed})")

                return True

            except Exception as e:
                logger.error(f"销毁会话失败 {session_id}: {e}")
                return False

    def get_session(self, session_id: str) -> Optional[SessionContext]:
        """
        获取会话信息

        Args:
            session_id: 会话ID

        Returns:
            会话上下文，如果不存在则返回None
        """
        with self._global_lock:
            session = self._sessions.get(session_id)
            if session:
                # 更新最后活动时间
                session.last_activity_time = time.time()
            return session

    def list_active_sessions(self) -> List[str]:
        """
        获取所有活跃会话ID列表

        Returns:
            活跃会话ID列表
        """
        with self._global_lock:
            return [session_id for session_id, session in self._sessions.items()
                   if session.state in [SessionState.INITIALIZING, SessionState.ACTIVE, SessionState.PAUSED]]

    def update_session_state(self, session_id: str, new_state: SessionState) -> bool:
        """
        更新会话状态

        Args:
            session_id: 会话ID
            new_state: 新状态

        Returns:
            是否成功更新
        """
        if session_id not in self._sessions:
            logger.warning(f"尝试更新不存在会话的状态: {session_id}")
            return False

        try:
            with self._session_locks[session_id]:
                old_state = self._sessions[session_id].state
                self._sessions[session_id].state = new_state
                self._sessions[session_id].last_activity_time = time.time()

            logger.info(f"状态更新: {session_id} {old_state.value} → {new_state.value}")
            return True

        except Exception as e:
            logger.error(f"更新会话状态失败 {session_id}: {e}")
            return False

    def record_audio_frame(self, session_id: str,
                          frame_duration: float,
                          transcription: Optional[str] = None,
                          confidence: Optional[float] = None) -> bool:
        """
        记录音频帧信息

        Args:
            session_id: 会话ID
            frame_duration: 帧持续时间 (秒)
            transcription: 识别文本
            confidence: 识别置信度

        Returns:
            是否成功记录
        """
        if session_id not in self._sessions:
            return False

        try:
            with self._session_locks[session_id]:
                session = self._sessions[session_id]

                # 更新基本信息
                session.audio_frames_processed += 1
                session.total_audio_duration += frame_duration
                session.last_activity_time = time.time()

                # 记录转录文本
                if transcription is not None:
                    session.transcription_history.append({
                        'timestamp': time.time(),
                        'text': transcription,
                        'confidence': confidence,
                        'frame_id': session.audio_frames_processed
                    })

                # 记录唤醒词检测
                if confidence is not None and confidence > 0.8:
                    session.wake_word_detections += 1
                    session.last_wake_word_confidence = confidence

                    # 更新全局统计
                    with self._stats_lock:
                        self.global_stats['total_wake_word_detections'] += 1

            # 更新全局统计
            with self._stats_lock:
                self.global_stats['total_audio_frames_processed'] += 1

            return True

        except Exception as e:
            logger.error(f"记录音频帧失败 {session_id}: {e}")
            return False

    def get_session_statistics(self, session_id: str) -> Optional[SessionStats]:
        """
        获取会话统计信息

        Args:
            session_id: 会话ID

        Returns:
            会话统计信息
        """
        session = self.get_session(session_id)
        if not session:
            return None

        duration = time.time() - session.start_time
        throughput = session.audio_frames_processed / duration if duration > 0 else 0.0

        # 计算平均置信度
        confidences = [entry['confidence'] for entry in session.transcription_history
                      if entry.get('confidence') is not None]
        avg_confidence = sum(confidences) / len(confidences) if confidences else 0.0

        return SessionStats(
            session_id=session_id,
            duration_seconds=duration,
            frames_processed=session.audio_frames_processed,
            wake_word_count=session.wake_word_detections,
            max_confidence=max(confidences) if confidences else 0.0,
            avg_confidence=avg_confidence,
            state=session.state,
            throughput=throughput
        )

    def get_global_statistics(self) -> dict:
        """获取全局统计信息"""
        with self._stats_lock:
            stats = self.global_stats.copy()

            # 添加当前活跃会话数
            stats['active_sessions'] = len(self._sessions)
            stats['utilization_rate'] = len(self._sessions) / self.max_sessions

            return stats

    def _start_monitoring(self) -> None:
        """启动监控线程"""
        if self._monitoring_active:
            return

        self._monitoring_active = True
        self._monitoring_thread = threading.Thread(
            target=self._monitoring_worker,
            daemon=True
        )
        self._monitoring_thread.start()

        logger.info("📊 会话监控已启动")

    def _monitoring_worker(self) -> None:
        """监控工作线程"""
        while self._monitoring_active:
            try:
                # 检查超时会话
                current_time = time.time()
                timeout_sessions = []

                for session_id, session in self._sessions.items():
                    if (current_time - session.last_activity_time > self.session_timeout_seconds and
                        session.state != SessionState.TERMINATED):
                        timeout_sessions.append(session_id)

                # 自动清理超时会话
                for session_id in timeout_sessions:
                    logger.info(f"🧹 自动清理超时会话: {session_id}")
                    self.destroy_session(session_id)

                # 监控间隔
                time.sleep(10)  # 每10秒检查一次

            except Exception as e:
                logger.error(f"监控工作线程错误: {e}")
                if self._monitoring_active:
                    time.sleep(1)  # 错误后短暂等待

    def stop_monitoring(self) -> None:
        """停止监控"""
        self._monitoring_active = False
        if self._monitoring_thread and self._monitoring_thread.is_alive():
            self._monitoring_thread.join(timeout=2.0)
        logger.info("📊 会话监控已停止")

    def cleanup_all_sessions(self) -> None:
        """清理所有会话"""
        with self._global_lock:
            session_ids = list(self._sessions.keys())
            for session_id in session_ids:
                self.destroy_session(session_id)
        logger.info("🧹 所有会话已清理")

    def save_session_data(self, session_id: str, output_path: str) -> bool:
        """
        保存会话数据

        Args:
            session_id: 会话ID
            output_path: 输出文件路径

        Returns:
            是否成功保存
        """
        session = self.get_session(session_id)
        if not session:
            return False

        try:
            # 确保目录存在
            Path(output_path).parent.mkdir(parents=True, exist_ok=True)

            # 准备数据
            data = {
                'session_id': session.session_id,
                'start_time': session.start_time,
                'last_activity_time': session.last_activity_time,
                'duration_seconds': session.last_activity_time - session.start_time,
                'audio_frames_processed': session.audio_frames_processed,
                'total_audio_duration': session.total_audio_duration,
                'wake_word_detections': session.wake_word_detections,
                'last_wake_word_confidence': session.last_wake_word_confidence,
                'state': session.state.value,
                'metadata': session.metadata,
                'transcription_history': session.transcription_history,
                'statistics': self.get_session_statistics(session_id).__dict__
            }

            # 保存JSON文件
            with open(output_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)

            logger.info(f"💾 会话数据已保存: {output_path}")
            return True

        except Exception as e:
            logger.error(f"保存会话数据失败 {session_id}: {e}")
            return False

    def __len__(self) -> int:
        """返回活跃会话数"""
        return len(self._sessions)

    def __contains__(self, session_id: str) -> bool:
        """检查会话是否存在"""
        return session_id in self._sessions

    def __str__(self) -> str:
        """返回会话管理器的字符串表示"""
        stats = self.get_global_statistics()
        return (
            f"会话管理器\n"
            f"活跃会话: {stats['active_sessions']}/{self.max_sessions}\n"
            f"利用率: {stats['utilization_rate']:.1%}\n"
            f"总创建: {stats['total_sessions_created']}\n"
            f"总销毁: {stats['total_sessions_terminated']}\n"
            f"平均时长: {stats['average_session_duration']:.1f}秒\n"
            f"唤醒词检测: {stats['total_wake_word_detections']}\n"
            f"峰值并发: {stats['peak_concurrent_sessions']}"
        )

    def __repr__(self) -> str:
        return (f"SessionManager(max_sessions={self.max_sessions}, "
                f"timeout={self.session_timeout_seconds}s)")


# 示例使用
if __name__ == "__main__":
    # 创建会话管理器
    manager = SessionManager(max_sessions=5, session_timeout_minutes=30)

    print("=" * 50)
    print(manager)
    print("=" * 50)

    # 创建测试会话
    session_ids = []
    for i in range(3):
        session_id = manager.create_session(metadata={'user_id': f'user_{i}'})
        session_ids.append(session_id)
        print(f"✅ 创建会话: {session_id}")

    print("\n活跃会话列表:")
    for session_id in manager.list_active_sessions():
        print(f"  - {session_id}")

    # 模拟音频帧处理
    import time
    for session_id in session_ids:
        for frame_id in range(5):
            manager.record_audio_frame(
                session_id=session_id,
                frame_duration=0.5,
                transcription=f"Frame {frame_id} for {session_id}",
                confidence=0.9
            )
            time.sleep(0.1)

    # 获取统计信息
    print("\n会话统计信息:")
    for session_id in session_ids:
        stats = manager.get_session_statistics(session_id)
        if stats:
            print(f"\n会话 {session_id}:")
            print(f"  持续时间: {stats.duration_seconds:.1f}秒")
            print(f"  处理帧数: {stats.frames_processed}")
            print(f"  唤醒词检测: {stats.wake_word_count}")
            print(f"  平均置信度: {stats.avg_confidence:.3f}")
            print(f"  状态: {stats.state.value}")

    print("\n" + "=" * 50)
    print("全局统计信息:")
    global_stats = manager.get_global_statistics()
    for key, value in global_stats.items():
        print(f"  {key}: {value}")

    # 保存会话数据
    for session_id in session_ids:
        output_path = f"/tmp/session_{session_id}.json"
        manager.save_session_data(session_id, output_path)

    # 清理
    manager.cleanup_all_sessions()
    manager.stop_monitoring()
    print("\n🧹 会话管理器已清理")
