#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
语音助手主控协调节点 - ROS2节点实现

负责监控所有节点状态，协调整体语音交互流程，
实现会话管理、错误恢复和性能监控。

作者: Claude Code
故事ID: Epic 1 ASR→LLM→TTS串联修复
"""

import os
import sys
import time
import asyncio
import logging
import traceback
from typing import Dict, Any, Optional, List
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict, deque

# 确保XLeRobot环境路径
project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if project_root not in sys.path:
    sys.path.insert(0, project_root)
if os.path.join(project_root, 'src') not in sys.path:
    sys.path.insert(0, os.path.join(project_root, 'src'))

# ROS2相关导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from audio_msg.msg import ASRStatus, LLMStatus, TTSStatus, LLMResponse

# 添加项目路径到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class SystemState(Enum):
    """系统状态枚举"""
    INITIALIZING = "initializing"
    READY = "ready"
    PROCESSING = "processing"
    ERROR = "error"
    SHUTTING_DOWN = "shutting_down"


@dataclass
class NodeStatus:
    """节点状态"""
    name: str
    state: int = 0  # 0=idle, 1=processing, 2=error
    last_update: float = 0.0
    error_count: int = 0
    last_error: str = ""
    avg_response_time: float = 0.0


@dataclass
class SessionMetrics:
    """会话指标"""
    session_id: str
    start_time: float
    asr_responses: int = 0
    llm_responses: int = 0
    tts_playbacks: int = 0
    total_response_time: float = 0.0
    error_count: int = 0


@dataclass
class CoordinatorConfig:
    """协调器配置"""
    status_check_interval: float = 1.0
    session_timeout: int = 600  # 10分钟
    max_error_count: int = 5
    performance_window: int = 100  # 最近100次请求的性能窗口
    log_retention_hours: int = 24


class VoiceAssistantCoordinator(Node):
    """语音助手主控协调节点"""

    def __init__(self):
        super().__init__('voice_assistant_coordinator')

        # 配置
        self.config = CoordinatorConfig()

        # 系统状态
        self.system_state = SystemState.INITIALIZING
        self.start_time = time.time()

        # 节点状态监控 - 使用当前时间初始化避免启动时报离线
        current_time = time.time()
        self.node_statuses: Dict[str, NodeStatus] = {
            'asr_bridge_node': NodeStatus('asr_bridge_node', last_update=current_time),
            'llm_service_node': NodeStatus('llm_service_node', last_update=current_time),
            'tts_service_node': NodeStatus('tts_service_node', last_update=current_time)
        }

        # 会话管理
        self.active_sessions: Dict[str, SessionMetrics] = {}
        self.session_history: deque = deque(maxlen=self.config.performance_window)

        # 性能统计
        self.total_requests = 0
        self.successful_requests = 0
        self.error_count = 0
        self.response_times = deque(maxlen=self.config.performance_window)

        # 回调组
        self.callback_group = ReentrantCallbackGroup()

        # 创建状态订阅者
        qos = QoSProfile(depth=10)

        self.asr_status_sub = self.create_subscription(
            ASRStatus,
            '/xlerobot/asr/status',
            self.asr_status_callback,
            qos,
            callback_group=self.callback_group
        )

        self.llm_status_sub = self.create_subscription(
            LLMStatus,
            '/xlerobot/llm/status',
            self.llm_status_callback,
            qos,
            callback_group=self.callback_group
        )

        self.tts_status_sub = self.create_subscription(
            TTSStatus,
            '/xlerobot/tts/status',
            self.tts_status_callback,
            qos,
            callback_group=self.callback_group
        )

        # 创建服务
        self.start_dialogue_service = self.create_service(
            SetBool,
            'start_dialogue',
            self.start_dialogue_callback,
            callback_group=self.callback_group
        )

        # 创建LLM请求发布者
        self.llm_request_pub = self.create_publisher(
            Header,  # 使用Header作为LLM请求消息
            '/xlerobot/llm/request',
            qos
        )

        # 创建TTS请求发布者
        self.tts_request_pub = self.create_publisher(
            LLMResponse,  # 复用LLMResponse作为TTS请求消息
            '/xlerobot/tts/request',
            qos
        )

        # 创建系统状态发布者
        self.system_status_pub = self.create_publisher(
            Header,  # 使用Header作为系统状态消息
            '/xlerobot/system/status',
            qos
        )

        # 创建定时器
        self.status_timer = self.create_timer(
            self.config.status_check_interval,
            self.check_system_health,
            callback_group=self.callback_group
        )

        self.cleanup_timer = self.create_timer(
            60.0,  # 每分钟清理一次
            self.cleanup_expired_sessions,
            callback_group=self.callback_group
        )

        self.get_logger().info("🚀 语音助手主控协调节点启动完成")

    def asr_status_callback(self, msg: ASRStatus):
        """处理ASR状态更新"""
        self.update_node_status('asr_bridge_node', msg)
        self.get_logger().debug(f"🎤 ASR状态更新: {msg.state}")

    def llm_status_callback(self, msg: LLMStatus):
        """处理LLM状态更新"""
        self.update_node_status('llm_service_node', msg)
        self.get_logger().debug(f"🤖 LLM状态更新: {msg.state}")

    def tts_status_callback(self, msg: TTSStatus):
        """处理TTS状态更新"""
        self.update_node_status('tts_service_node', msg)
        self.get_logger().debug(f"🔊 TTS状态更新: {msg.state}")

    def update_node_status(self, node_name: str, status_msg):
        """更新节点状态"""
        if node_name not in self.node_statuses:
            return

        node_status = self.node_statuses[node_name]
        node_status.last_update = time.time()

        # 更新状态码
        if hasattr(status_msg, 'state'):
            node_status.state = status_msg.state

        # 更新错误信息
        if hasattr(status_msg, 'state') and status_msg.state == 2:  # error
            node_status.error_count += 1
            if hasattr(status_msg, 'last_error'):
                node_status.last_error = status_msg.last_error

        # 更新响应时间
        if hasattr(status_msg, 'avg_response_time'):
            node_status.avg_response_time = status_msg.avg_response_time

        # 检查是否需要处理错误
        if node_status.error_count >= self.config.max_error_count:
            self.handle_node_error(node_name, node_status)

    def handle_node_error(self, node_name: str, node_status: NodeStatus):
        """处理节点错误"""
        self.get_logger().error(f"❌ 节点 {node_name} 错误次数过多: {node_status.error_count}")
        self.get_logger().error(f"最后一次错误: {node_status.last_error}")

        # 触发错误恢复机制 - 使用线程避免asyncio事件循环问题
        import threading
        thread = threading.Thread(
            target=self._recover_from_error_sync,
            args=(node_name,),
            daemon=True
        )
        thread.start()

    def _recover_from_error_sync(self, node_name: str):
        """从错误中恢复（同步版本）"""
        self.get_logger().info(f"🔄 尝试恢复节点 {node_name}...")

        try:
            # 等待恢复
            time.sleep(2)

            # 重置错误计数
            if node_name in self.node_statuses:
                self.node_statuses[node_name].error_count = 0
                self.node_statuses[node_name].last_error = ""

            self.get_logger().info(f"✅ 节点 {node_name} 恢复完成")

        except Exception as e:
            self.get_logger().error(f"❌ 节点 {node_name} 恢复失败: {e}")

    def start_dialogue_callback(self, request: SetBool.Request, response: SetBool.Response):
        """启动对话服务回调"""
        if request.data:
            # 启动对话
            session_id = f"dialogue_{int(time.time())}"
            self.active_sessions[session_id] = SessionMetrics(
                session_id=session_id,
                start_time=time.time()
            )

            response.success = True
            response.message = f"对话已启动，会话ID: {session_id}"
            self.get_logger().info(f"🎤 启动对话会话: {session_id}")
        else:
            # 停止对话
            stopped_count = len(self.active_sessions)
            self.active_sessions.clear()

            response.success = True
            response.message = f"已停止 {stopped_count} 个活跃会话"
            self.get_logger().info(f"🛑 停止所有对话会话")

        return response

    def send_llm_request(self, text: str, session_id: str = None):
        """发送LLM请求"""
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = text if session_id is None else f"{session_id}:{text}"

        self.llm_request_pub.publish(msg)
        self.get_logger().info(f"📤 发送LLM请求: {text[:30]}...")

    def send_tts_request(self, text: str, session_id: str = None):
        """发送TTS请求"""
        msg = LLMResponse()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = session_id or "default"

        msg.text = text
        msg.session_id = session_id or "default"
        msg.confidence = 0.9
        msg.status_code = 0
        msg.error_message = ""
        msg.user_input = text
        msg.response_time = 0.0
        msg.model_name = "coordinator"

        self.tts_request_pub.publish(msg)
        self.get_logger().info(f"📤 发送TTS请求: {text[:30]}...")

    def handle_voice_command_flow(self, text: str, session_id: str = None):
        """处理完整的语音命令流程：ASR → LLM → TTS"""
        if session_id is None:
            session_id = f"flow_{int(time.time())}"

        # 更新统计
        self.total_requests += 1

        # 1. 发送LLM请求
        self.send_llm_request(text, session_id)

        # 注意：在实际实现中，应该监听LLM响应然后发送TTS请求
        # 这里为了演示，直接模拟LLM响应并发送TTS请求
        self.get_logger().info(f"🔄 处理语音命令流程: {text[:30]}...")

        # 可以在这里添加异步处理逻辑
        # asyncio.create_task(self.async_voice_command_flow(text, session_id))

    def check_system_health(self):
        """检查系统健康状态"""
        current_time = time.time()
        uptime = current_time - self.start_time

        # 启动宽限期：前15秒不报告节点离线
        if uptime < 15.0:
            self.get_logger().debug(f"🚀 系统启动中... ({uptime:.1f}s)")
            return

        # 检查所有节点是否在线
        all_nodes_healthy = True
        for node_name, node_status in self.node_statuses.items():
            time_since_update = current_time - node_status.last_update

            # 如果超过10秒没有更新，认为节点离线
            if time_since_update > 10.0:
                self.get_logger().warning(f"⚠️ 节点 {node_name} 可能离线 ({time_since_update:.1f}s)")
                all_nodes_healthy = False

            # 检查节点是否处于错误状态
            if node_status.state == 2:  # error
                all_nodes_healthy = False

        # 更新系统状态
        if all_nodes_healthy and len(self.node_statuses) == 3:
            if self.system_state == SystemState.INITIALIZING:
                self.system_state = SystemState.READY
                self.get_logger().info("✅ 系统初始化完成，准备就绪")
        else:
            self.system_state = SystemState.ERROR
            self.get_logger().warning("⚠️ 系统状态异常")

        # 发布系统状态
        self.publish_system_status()

    def publish_system_status(self):
        """发布系统状态"""
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = str(self.system_state.value)

        # 可以在注释中添加更多系统信息
        # system_info = {
        #     "uptime": time.time() - self.start_time,
        #     "active_sessions": len(self.active_sessions),
        #     "total_requests": self.total_requests,
        #     "success_rate": self.successful_requests / max(1, self.total_requests),
        # }

        self.system_status_pub.publish(msg)

    def cleanup_expired_sessions(self):
        """清理过期的会话"""
        current_time = time.time()
        expired_sessions = []

        for session_id, metrics in self.active_sessions.items():
            if current_time - metrics.start_time > self.config.session_timeout:
                expired_sessions.append(session_id)

        for session_id in expired_sessions:
            # 移动到历史记录
            session_metrics = self.active_sessions.pop(session_id)
            self.session_history.append(session_metrics)

            self.get_logger().info(f"🗑️ 清理过期会话: {session_id}")

    def get_system_metrics(self) -> Dict[str, Any]:
        """获取系统指标"""
        current_time = time.time()
        uptime = current_time - self.start_time

        # 计算平均响应时间
        avg_response_time = sum(self.response_times) / len(self.response_times) if self.response_times else 0.0

        # 计算成功率
        success_rate = self.successful_requests / max(1, self.total_requests) * 100

        return {
            "uptime": uptime,
            "system_state": self.system_state.value,
            "active_sessions": len(self.active_sessions),
            "total_requests": self.total_requests,
            "successful_requests": self.successful_requests,
            "error_count": self.error_count,
            "success_rate": success_rate,
            "avg_response_time": avg_response_time,
            "node_statuses": {
                name: {
                    "state": status.state,
                    "error_count": status.error_count,
                    "avg_response_time": status.avg_response_time
                }
                for name, status in self.node_statuses.items()
            }
        }

    def log_system_status(self):
        """记录系统状态"""
        metrics = self.get_system_metrics()

        self.get_logger().info("📊 系统状态报告:")
        self.get_logger().info(f"  运行时间: {metrics['uptime']:.1f}秒")
        self.get_logger().info(f"  系统状态: {metrics['system_state']}")
        self.get_logger().info(f"  活跃会话: {metrics['active_sessions']}")
        self.get_logger().info(f"  总请求数: {metrics['total_requests']}")
        self.get_logger().info(f"  成功率: {metrics['success_rate']:.1f}%")
        self.get_logger().info(f"  平均响应时间: {metrics['avg_response_time']:.2f}秒")


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2
        rclpy.init(args=args)

        # 创建协调器节点
        coordinator = VoiceAssistantCoordinator()

        # 创建多线程执行器
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(coordinator)

        # 创建状态日志定时器
        log_timer = coordinator.create_timer(
            300.0,  # 每5分钟记录一次状态
            coordinator.log_system_status
        )

        try:
            # 运行节点
            executor.spin()
        except KeyboardInterrupt:
            coordinator.get_logger().info("🛑 收到中断信号，正在关闭协调器...")
        finally:
            # 最终状态报告
            coordinator.log_system_status()

            # 清理资源
            log_timer.cancel()
            coordinator.destroy_node()
            executor.shutdown()
            rclpy.shutdown()

    except Exception as e:
        print(f"❌ 协调器启动失败: {e}")
        print(f"详细错误: {traceback.format_exc()}")
        sys.exit(1)


if __name__ == '__main__':
    main()