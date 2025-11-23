#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
LLM服务节点 - ROS2节点实现

负责接收ASR识别结果，调用LLM生成响应，并发布LLM响应消息。
实现完整的对话上下文管理和错误处理机制。

作者: Claude Code
故事ID: Epic 1 ASR→LLM→TTS串联修复
"""

import os
import sys
import time
import asyncio
import logging
import traceback
from typing import Optional, Dict, Any
from dataclasses import dataclass

# ROS2相关导入
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Header, String
from audio_msg.msg import ASRResult, LLMResponse, LLMStatus

# 添加项目路径到Python路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

# 导入现有LLM模块
try:
    from modules.llm.qwen_client import QwenAPIClient
    from modules.asr.siqiang_intelligent_dialogue import SiQiangIntelligentDialogue
    from modules.llm.dialogue_context import DialogueContext
except ImportError as e:
    print(f"❌ 导入LLM模块失败: {e}")
    print("请确保PYTHONPATH设置正确")
    sys.exit(1)


# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class LLMNodeConfig:
    """LLM节点配置"""
    qwen_api_key: str = ""
    model_name: str = "qwen-turbo"
    max_tokens: int = 2000
    temperature: float = 0.7
    session_timeout: int = 300  # 5分钟会话超时
    request_timeout: float = 15.0  # 15秒请求超时


class LLMServiceNode(Node):
    """LLM服务节点"""

    def __init__(self):
        super().__init__('llm_service_node')

        # 配置
        self.config = LLMNodeConfig(
            qwen_api_key=os.getenv('QWEN_API_KEY', ''),
            model_name=os.getenv('QWEN_MODEL', 'qwen-turbo')
        )

        # 状态管理
        self.current_state = 0  # 0=idle, 1=processing, 2=error
        self.session_contexts: Dict[str, DialogueContext] = {}
        self.start_time = time.time()

        # 性能统计
        self.total_requests = 0
        self.failed_requests = 0
        self.response_times = []

        # 初始化LLM客户端
        try:
            self.qwen_client = QwenAPIClient()
            self.dialogue_engine = SiQiangIntelligentDialogue()
            self.get_logger().info("✅ LLM客户端初始化成功")
        except Exception as e:
            self.get_logger().error(f"❌ LLM客户端初始化失败: {e}")
            self.current_state = 2
            return

        # 创建订阅者 - 订阅语音命令
        qos = QoSProfile(depth=10)
        self.voice_command_subscription = self.create_subscription(
            ASRResult,
            '/xlerobot/asr/result',
            self.voice_command_callback,
            qos
        )

        # 创建订阅者 - 订阅LLM请求（来自协调器）
        self.llm_request_subscription = self.create_subscription(
            Header,  # 使用Header作为简单请求消息
            '/xlerobot/llm/request',
            self.llm_request_callback,
            qos
        )

        # 创建发布者 - 发布LLM响应
        self.llm_publisher = self.create_publisher(
            LLMResponse,
            '/xlerobot/llm/response',
            qos
        )

        # 兼容性String发布者（用于std_msgs通信）
        self.llm_response_string_publisher = self.create_publisher(
            String,
            '/xlerobot/llm/response_string',
            qos
        )

        # 创建状态发布者
        self.status_publisher = self.create_publisher(
            LLMStatus,
            '/xlerobot/llm/status',
            qos
        )

        # 创建状态定时器 - 定期发布状态
        self.status_timer = self.create_timer(
            1.0,  # 每秒发布一次状态
            self.publish_status
        )

        self.get_logger().info("🚀 LLM服务节点启动完成")

    def voice_command_callback(self, msg: ASRResult):
        """处理语音命令"""
        self.get_logger().info(f"🎤 收到语音命令: {msg.text} (置信度: {msg.confidence:.2f})")

        # 检查ASR是否成功
        if not msg.text.strip():
            self.get_logger().warning("⚠️ ASR结果为空，跳过处理")
            return

        # 🔧 修复：阿里云ASR API不返回置信度字段，移除置信度检查
        # 如果识别到文本内容，直接处理（置信度问题已在ASR服务层处理）
        self.get_logger().info(f"✅ ASR识别成功: {msg.text} (置信度: {msg.confidence:.2f})")

        # 更新状态
        self.current_state = 1  # processing
        self.total_requests += 1

        # 异步处理LLM请求 - 使用线程避免asyncio事件循环问题
        import concurrent.futures
        if not hasattr(self, '_executor'):
            self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)
        self._executor.submit(self._run_llm_request_sync, msg)

    def llm_request_callback(self, msg: Header):
        """处理LLM请求（来自协调器）"""
        self.get_logger().info(f"📨 收到LLM请求: {msg.frame_id}")

        # 从frame_id提取请求文本
        request_text = msg.frame_id if msg.frame_id else "你好"

        # 创建模拟ASR结果
        mock_asr_result = ASRResult()
        mock_asr_result.header = msg
        mock_asr_result.text = request_text
        mock_asr_result.confidence = 1.0
        mock_asr_result.status_code = 0

        # 更新状态
        self.current_state = 1  # processing
        self.total_requests += 1

        # 异步处理LLM请求 - 使用线程避免asyncio事件循环问题
        import concurrent.futures
        if not hasattr(self, '_executor'):
            self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)
        self._executor.submit(self._run_llm_request_sync, mock_asr_result)

    def _run_llm_request_sync(self, asr_msg: ASRResult):
        """在线程中运行LLM请求"""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self.process_llm_request(asr_msg))
        except Exception as e:
            self.get_logger().error(f"❌ LLM请求处理失败: {e}")
        finally:
            loop.close()

    async def process_llm_request(self, asr_msg: ASRResult):
        """异步处理LLM请求"""
        start_time = time.time()
        session_id = self.get_session_id(asr_msg.header)

        try:
            # 获取或创建会话上下文
            if session_id not in self.session_contexts:
                self.session_contexts[session_id] = DialogueContext()
                self.get_logger().info(f"🆕 创建新会话: {session_id}")

            context = self.session_contexts[session_id]

            # 构建LLM请求
            user_input = asr_msg.text.strip()

            # 添加用户输入到上下文
            context.add_user_message(user_input)

            # 调用LLM生成响应（带超时控制）
            try:
                response_text = await asyncio.wait_for(
                    self.generate_llm_response(user_input, context),
                    timeout=self.config.request_timeout
                )
            except asyncio.TimeoutError:
                self.get_logger().error(f"❌ LLM响应超时（{self.config.request_timeout}秒）")
                self.failed_requests += 1
                self.current_state = 2  # error

                # 创建临时ASR消息用于错误响应
                temp_asr = ASRResult()
                temp_asr.header = Header()
                temp_asr.header.stamp = self.get_clock().now().to_msg()
                temp_asr.text = f"LLM响应超时（{self.config.request_timeout}秒），请重试"
                temp_asr.confidence = 0.0

                # 发布超时错误响应
                await self.publish_error_response(
                    session_id=session_id,
                    error_message=f"LLM响应超时（{self.config.request_timeout}秒），请重试",
                    original_asr=temp_asr
                )
                return

            # 添加助手响应到上下文
            context.add_assistant_message(response_text)

            # 计算响应时间
            response_time = time.time() - start_time
            self.response_times.append(response_time)

            # 发布LLM响应
            await self.publish_llm_response(
                text=response_text,
                session_id=session_id,
                user_input=user_input,
                response_time=response_time,
                original_asr=asr_msg
            )

            self.get_logger().info(f"✅ LLM响应生成完成: {response_text[:50]}...")
            self.current_state = 0  # idle

        except Exception as e:
            self.get_logger().error(f"❌ LLM处理失败: {e}")
            self.get_logger().error(f"详细错误: {traceback.format_exc()}")
            self.failed_requests += 1
            self.current_state = 2  # error

            # 发布错误响应
            await self.publish_error_response(
                session_id=session_id,
                error_message=str(e),
                original_asr=asr_msg
            )

    async def generate_llm_response(self, user_input: str, context: DialogueContext) -> str:
        """生成LLM响应"""
        try:
            # 使用对话引擎生成响应
            response = await self.dialogue_engine.generate_response_async(
                user_input=user_input,
                context=context.get_conversation_history()
            )

            # 如果响应为空，使用备用响应
            if not response or response.strip() == "":
                response = "不好意思，我暂时无法理解你的意思，可以再试一次吗？"

            return response.strip()

        except Exception as e:
            self.get_logger().error(f"❌ LLM生成响应失败: {e}")
            # 降级处理：返回默认响应
            return "系统暂时繁忙，请稍后再试。"

    async def publish_llm_response(self, text: str, session_id: str,
                                 user_input: str, response_time: float,
                                 original_asr: ASRResult):
        """发布LLM响应消息"""
        msg = LLMResponse()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.text = text
        msg.session_id = session_id
        msg.confidence = 0.8  # 固定置信度
        msg.status_code = 0  # 成功
        msg.error_message = ""

        msg.user_input = user_input
        msg.response_time = float(response_time)
        msg.model_name = self.config.model_name

        self.llm_publisher.publish(msg)

        # 同时发布String格式消息（兼容性）
        string_msg = String()
        string_msg.data = text
        self.llm_response_string_publisher.publish(string_msg)

        self.get_logger().debug(f"📤 发布LLM响应: {text[:30]}...")

    async def publish_error_response(self, session_id: str, error_message: str,
                                   original_asr: ASRResult):
        """发布错误响应消息"""
        msg = LLMResponse()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.text = "抱歉，我现在无法处理您的请求。"
        msg.session_id = session_id
        msg.confidence = 0.0
        msg.status_code = 1  # 错误
        msg.error_message = error_message

        msg.user_input = original_asr.text
        msg.response_time = 0.0
        msg.model_name = ""

        self.llm_publisher.publish(msg)

        # 同时发布String格式消息（兼容性）
        string_msg = String()
        string_msg.data = msg.text  # 错误响应文本
        self.llm_response_string_publisher.publish(string_msg)

    def publish_status(self):
        """发布节点状态"""
        msg = LLMStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.node_name = self.get_name()
        msg.state = self.current_state
        msg.avg_response_time = sum(self.response_times) / len(self.response_times) if self.response_times else 0.0
        msg.total_requests = self.total_requests
        msg.failed_requests = self.failed_requests
        msg.cpu_usage = 0.0  # TODO: 实现CPU使用率监控
        msg.memory_usage = 0.0  # TODO: 实现内存使用率监控
        msg.last_error = ""

        self.status_publisher.publish(msg)

    def get_session_id(self, header: Header) -> str:
        """获取或生成会话ID"""
        # 如果ASR消息有session_id，使用它；否则生成基于时间的ID
        if hasattr(header, 'frame_id') and header.frame_id:
            return header.frame_id
        else:
            return f"session_{int(time.time())}"

    def cleanup_sessions(self):
        """清理超时的会话"""
        current_time = time.time()
        timeout_sessions = []

        for session_id, context in self.session_contexts.items():
            if current_time - context.last_activity > self.config.session_timeout:
                timeout_sessions.append(session_id)

        for session_id in timeout_sessions:
            del self.session_contexts[session_id]
            self.get_logger().info(f"🗑️ 清理超时会话: {session_id}")


def main(args=None):
    """主函数"""
    try:
        # 初始化ROS2（先初始化才能使用logger）
        rclpy.init(args=args)

        # 创建LLM服务节点
        node = LLMServiceNode()

        # 检查环境变量（改为警告而非强制退出）
        qwen_api_key = os.getenv('QWEN_API_KEY')
        if not qwen_api_key:
            node.get_logger().warning("⚠️ QWEN_API_KEY环境变量未设置")
            node.get_logger().warning("💡 LLM功能将受限，系统将使用演示模式")
            node.get_logger().warning("   设置方法: export QWEN_API_KEY='your_api_key_here'")

            # 调试信息
            env_debug = {k: v for k, v in os.environ.items() if 'QWEN' in k or 'API' in k}
            node.get_logger().debug(f"🔍 相关环境变量: {env_debug}")
        else:
            node.get_logger().info(f"✅ QWEN_API_KEY已设置: {qwen_api_key[:10]}...")

        # 清理会话定时器
        session_cleanup_timer = node.create_timer(
            60.0,  # 每分钟检查一次
            node.cleanup_sessions
        )

        # 运行节点
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("🛑 收到中断信号，正在关闭节点...")
        finally:
            # 清理资源
            node.destroy_node()
            rclpy.shutdown()

    except Exception as e:
        print(f"❌ 节点启动失败: {e}")
        print(f"详细错误: {traceback.format_exc()}")
        sys.exit(1)


if __name__ == '__main__':
    main()