#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 1.4: 连续语音识别 - 流式状态机
Streaming State Machine for Continuous Speech Recognition

实现流式语音识别的状态管理和转换逻辑。
状态转换: IDLE → LISTENING → SPEECH_DETECTED → PROCESSING → TRANSCRIBING → COMPLETED

作者: Dev Agent
故事ID: Story 1.4
"""

import time
import threading
from enum import Enum, auto
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
import logging
import queue

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class StreamingState(Enum):
    """流式识别状态枚举"""
    IDLE = auto()              # 空闲状态，等待唤醒
    LISTENING = auto()         # 监听状态，检测唤醒词
    SPEECH_DETECTED = auto()   # 检测到语音，开始处理
    PROCESSING = auto()        # 正在处理音频
    TRANSCRIBING = auto()      # 正在转录
    COMPLETED = auto()         # 转录完成
    ERROR = auto()             # 错误状态


@dataclass
class StateTransition:
    """状态转换记录"""
    from_state: StreamingState
    to_state: StreamingState
    timestamp: float
    trigger: str
    data: Dict[str, Any] = field(default_factory=dict)


@dataclass
class StateContext:
    """状态上下文信息"""
    state: StreamingState
    enter_time: float
    duration: float = 0.0
    metadata: Dict[str, Any] = field(default_factory=dict)
    transition_history: List[StateTransition] = field(default_factory=list)


class StreamingStateMachine:
    """
    流式语音识别状态机

    状态转换流程:
    IDLE → LISTENING (启动监听)
    LISTENING → SPEECH_DETECTED (检测到语音)
    SPEECH_DETECTED → PROCESSING (开始处理)
    PROCESSING → TRANSCRIBING (开始转录)
    TRANSCRIBING → COMPLETED (转录完成)
    COMPLETED → IDLE (返回空闲)

    错误处理:
    任意状态 → ERROR (发生错误)
    ERROR → IDLE (恢复空闲)
    """

    def __init__(self, state_timeout_seconds: float = 30.0):
        """
        初始化状态机

        Args:
            state_timeout_seconds: 状态超时时间 (秒)
        """
        self.state_timeout_seconds = state_timeout_seconds

        # 当前状态
        self._current_state = StreamingState.IDLE
        self._state_context = StateContext(
            state=StreamingState.IDLE,
            enter_time=time.time()
        )

        # 状态转换规则
        self._transitions: Dict[StreamingState, Dict[str, StreamingState]] = {
            StreamingState.IDLE: {
                'start_listening': StreamingState.LISTENING,
                'error': StreamingState.ERROR
            },
            StreamingState.LISTENING: {
                'wake_word_detected': StreamingState.SPEECH_DETECTED,
                'timeout': StreamingState.IDLE,
                'error': StreamingState.ERROR
            },
            StreamingState.SPEECH_DETECTED: {
                'start_processing': StreamingState.PROCESSING,
                'speech_ended': StreamingState.IDLE,
                'timeout': StreamingState.IDLE,
                'error': StreamingState.ERROR
            },
            StreamingState.PROCESSING: {
                'start_transcribing': StreamingState.TRANSCRIBING,
                'processing_failed': StreamingState.ERROR,
                'timeout': StreamingState.IDLE,
                'error': StreamingState.ERROR
            },
            StreamingState.TRANSCRIBING: {
                'transcription_complete': StreamingState.COMPLETED,
                'transcription_failed': StreamingState.ERROR,
                'timeout': StreamingState.IDLE,
                'error': StreamingState.ERROR
            },
            StreamingState.COMPLETED: {
                'reset': StreamingState.IDLE,
                'error': StreamingState.ERROR
            },
            StreamingState.ERROR: {
                'reset': StreamingState.IDLE,
                'fatal_error': StreamingState.IDLE
            }
        }

        # 状态回调函数
        self._callbacks: Dict[StreamingState, List[Callable]] = {
            state: [] for state in StreamingState
        }

        # 线程安全
        self._lock = threading.RLock()

        # 统计信息
        self._stats = {
            'total_transitions': 0,
            'state_durations': {},
            'transition_count': {},
            'last_transition_time': time.time(),
            'errors': 0,
            'timeouts': 0
        }

        logger.info(f"流式状态机初始化完成，当前状态: {self._current_state.name}")

    def transition(self, event: str, data: Optional[Dict[str, Any]] = None) -> bool:
        """
        执行状态转换

        Args:
            event: 触发事件
            data: 转换数据

        Returns:
            是否成功转换
        """
        with self._lock:
            current_state = self._current_state
            transitions = self._transitions.get(current_state, {})

            # 检查是否允许该转换
            if event not in transitions:
                logger.warning(f"不允许的转换: {current_state.name} --({event})--> [无]")

                # 特殊处理: 错误事件总是允许
                if event == 'error':
                    new_state = StreamingState.ERROR
                else:
                    return False
            else:
                new_state = transitions[event]

            # 记录转换前的状态
            old_state = self._current_state

            # 执行转换
            self._current_state = new_state

            # 更新状态上下文
            current_time = time.time()
            duration = current_time - self._state_context.enter_time
            self._state_context.duration = duration
            self._state_context.state = new_state
            self._state_context.enter_time = current_time

            # 记录状态转换
            transition = StateTransition(
                from_state=old_state,
                to_state=new_state,
                timestamp=current_time,
                trigger=event,
                data=data or {}
            )
            self._state_context.transition_history.append(transition)

            # 更新统计信息
            self._stats['total_transitions'] += 1
            self._stats['last_transition_time'] = current_time

            # 记录状态时长
            if old_state not in self._stats['state_durations']:
                self._stats['state_durations'][old_state] = []
            self._stats['state_durations'][old_state].append(duration)

            # 记录转换次数
            transition_key = f"{old_state.name}->{new_state.name}"
            self._stats['transition_count'][transition_key] = \
                self._stats['transition_count'].get(transition_key, 0) + 1

            # 记录错误和超时
            if event == 'error':
                self._stats['errors'] += 1
            elif event == 'timeout':
                self._stats['timeouts'] += 1

            # 记录日志
            logger.info(f"状态转换: {old_state.name} --({event})--> {new_state.name}")

            # 调用回调函数
            self._trigger_callbacks(new_state, old_state, event, data)

            return True

    def _trigger_callbacks(self, new_state: StreamingState,
                          old_state: StreamingState,
                          event: str,
                          data: Optional[Dict[str, Any]]) -> None:
        """触发状态回调函数"""
        callbacks = self._callbacks.get(new_state, [])
        for callback in callbacks:
            try:
                callback(new_state, old_state, event, data or {})
            except Exception as e:
                logger.error(f"状态回调函数执行失败: {e}")

    def add_callback(self, state: StreamingState, callback: Callable) -> None:
        """
        添加状态回调函数

        Args:
            state: 监听的状态
            callback: 回调函数，签名: callback(new_state, old_state, event, data)
        """
        with self._lock:
            self._callbacks[state].append(callback)
            logger.debug(f"添加状态回调: {state.name}")

    def remove_callback(self, state: StreamingState, callback: Callable) -> bool:
        """
        移除状态回调函数

        Args:
            state: 监听的状态
            callback: 回调函数

        Returns:
            是否成功移除
        """
        with self._lock:
            try:
                self._callbacks[state].remove(callback)
                logger.debug(f"移除状态回调: {state.name}")
                return True
            except ValueError:
                logger.warning(f"尝试移除不存在的回调函数: {state.name}")
                return False

    def current_state(self) -> StreamingState:
        """获取当前状态"""
        with self._lock:
            return self._current_state

    def get_state_context(self) -> StateContext:
        """获取状态上下文"""
        with self._lock:
            # 返回副本以避免外部修改
            return StateContext(
                state=self._state_context.state,
                enter_time=self._state_context.enter_time,
                duration=self._state_context.duration,
                metadata=self._state_context.metadata.copy(),
                transition_history=self._state_context.transition_history.copy()
            )

    def get_available_transitions(self) -> List[str]:
        """获取当前状态可用的转换事件列表"""
        with self._lock:
            transitions = self._transitions.get(self._current_state, {})
            return list(transitions.keys())

    def is_in_state(self, *states: StreamingState) -> bool:
        """检查当前状态是否在指定状态列表中"""
        with self._lock:
            return self._current_state in states

    def force_reset(self, reason: str = "手动重置") -> bool:
        """强制重置到IDLE状态"""
        with self._lock:
            return self.transition('reset', {'reason': reason})

    def get_statistics(self) -> dict:
        """获取状态机统计信息"""
        with self._lock:
            stats = self._stats.copy()

            # 计算平均状态时长
            avg_durations = {}
            for state, durations in self._stats['state_durations'].items():
                avg_durations[state.name] = sum(durations) / len(durations) if durations else 0.0
            stats['average_state_durations'] = avg_durations

            # 当前状态信息
            current_duration = time.time() - self._state_context.enter_time
            stats['current_state'] = self._current_state.name
            stats['current_state_duration'] = current_duration

            return stats

    def print_transition_history(self) -> None:
        """打印状态转换历史"""
        with self._lock:
            print("\n" + "=" * 60)
            print("状态转换历史")
            print("=" * 60)

            for i, transition in enumerate(self._state_context.transition_history, 1):
                timestamp = time.strftime('%H:%M:%S', time.localtime(transition.timestamp))
                print(f"{i:2d}. {timestamp} | {transition.from_state.name} --({trigger})--> {transition.to_state.name}")

                if transition.data:
                    print(f"     数据: {transition.data}")
            print("=" * 60)

    def __str__(self) -> str:
        """返回状态机的字符串表示"""
        context = self.get_state_context()
        stats = self.get_statistics()

        return (
            f"流式状态机\n"
            f"当前状态: {context.state.name}\n"
            f"持续时间: {context.duration:.1f}秒\n"
            f"总转换次数: {stats['total_transitions']}\n"
            f"错误次数: {stats['errors']}\n"
            f"超时次数: {stats['timeouts']}"
        )

    def __repr__(self) -> str:
        return f"StreamingStateMachine(timeout={self.state_timeout_seconds}s)"


# 示例使用
if __name__ == "__main__":
    # 创建状态机
    state_machine = StreamingStateMachine(state_timeout_seconds=10.0)

    print("=" * 60)
    print("流式状态机演示")
    print("=" * 60)

    # 添加状态回调
    def on_listening_enter(new_state, old_state, event, data):
        print(f"  🎧 进入监听状态: {event}")

    def on_speech_detected(new_state, old_state, event, data):
        print(f"  🗣️ 检测到语音: {data.get('confidence', 0.0)}")

    def on_processing(new_state, old_state, event, data):
        print(f"  ⚙️ 开始处理: {data.get('duration', 0.0)}秒音频")

    def on_transcribing(new_state, old_state, event, data):
        print(f"  📝 开始转录: {data.get('text_length', 0)}字符")

    state_machine.add_callback(StreamingState.LISTENING, on_listening_enter)
    state_machine.add_callback(StreamingState.SPEECH_DETECTED, on_speech_detected)
    state_machine.add_callback(StreamingState.PROCESSING, on_processing)
    state_machine.add_callback(StreamingState.TRANSCRIBING, on_transcribing)

    # 模拟状态转换流程
    print("\n模拟完整流程:")
    print("1. 启动监听")
    state_machine.transition('start_listening')

    print(f"\n当前状态: {state_machine.current_state().name}")
    print(f"可用转换: {state_machine.get_available_transitions()}")

    print("\n2. 检测到唤醒词")
    state_machine.transition('wake_word_detected', {'confidence': 0.95, 'wake_word': '傻强'})

    print("\n3. 开始处理")
    state_machine.transition('start_processing', {'duration': 2.5, 'audio_length': 40000})

    print("\n4. 开始转录")
    state_machine.transition('start_transcribing', {'text_length': 25, 'language': 'cantonese'})

    print("\n5. 转录完成")
    state_machine.transition('transcription_complete', {
        'text': '你好，我想查询天气',
        'confidence': 0.92
    })

    print("\n6. 重置到IDLE")
    state_machine.transition('reset')

    # 打印统计信息
    print("\n" + "=" * 60)
    print("统计信息:")
    stats = state_machine.get_statistics()
    for key, value in stats.items():
        if key != 'average_state_durations':
            print(f"  {key}: {value}")

    print("\n平均状态时长:")
    for state, duration in stats['average_state_durations'].items():
        print(f"  {state}: {duration:.2f}秒")

    # 打印转换历史
    state_machine.print_transition_history()

    print("\n" + "=" * 60)
    print(state_machine)
    print("=" * 60)
