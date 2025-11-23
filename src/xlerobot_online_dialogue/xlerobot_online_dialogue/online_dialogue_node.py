"""
OnlineDialogueNode - 在线对话ROS2节点
Story 1.7: 多模态在线对话API集成
严格遵循Epic 1纯在线架构 - 仅作为API包装器，无本地对话逻辑
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import asyncio
import time
import base64
import logging
from typing import Dict, Optional
import threading

# 标准消息类型
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image

# 自定义消息类型 - 暂时注释，等待编译
# from xlerobot_online_dialogue.msg import OnlineDialogueInput, OnlineDialogueResponse

# 导入纯在线组件
from .online_dialogue_api import OnlineDialogueAPI, DialogueRequest
from .simple_session_manager import SimpleSessionManager
from .cantonese_text_processor import CantoneseTextProcessor

class OnlineDialogueNode(Node):
    """
    在线对话ROS2节点
    严格遵循Epic 1纯在线架构 - 仅作为API包装器
    """

    def __init__(self):
        """初始化在线对话节点"""
        super().__init__('online_dialogue_node')

        logger = self.get_logger()
        logger.info("🤖 初始化OnlineDialogueNode - 纯在线架构")

        # 配置QoS
        self.qos_profile = QoSProfile(depth=10)

        # 初始化纯在线组件
        try:
            self.dialogue_api = OnlineDialogueAPI()
            self.session_manager = SimpleSessionManager()
            self.text_processor = CantoneseTextProcessor()
            logger.info("✅ 在线组件初始化成功")
        except Exception as e:
            logger.error(f"❌ 在线组件初始化失败: {e}")
            raise

        # 订阅器
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            self.qos_profile
        )

        self.image_subscriber = self.create_subscription(
            Image,
            'image_input',
            self.image_callback,
            self.qos_profile
        )

        self.text_subscriber = self.create_subscription(
            OnlineDialogueInput,
            'dialogue_input',
            self.dialogue_input_callback,
            self.qos_profile
        )

        # 发布器
        self.response_publisher = self.create_publisher(
            OnlineDialogueResponse,
            'dialogue_response',
            self.qos_profile
        )

        self.status_publisher = self.create_publisher(
            OnlineDialogueResponse,
            'dialogue_status',
            self.qos_profile
        )

        # 内部状态管理
        self.current_audio: Optional[AudioData] = None
        self.current_image: Optional[Image] = None
        self.last_dialogue_time: Dict[str, float] = {}

        # 节点统计
        self.node_stats = {
            "audio_messages_received": 0,
            "image_messages_received": 0,
            "text_messages_received": 0,
            "dialogues_processed": 0,
            "successful_responses": 0,
            "failed_responses": 0
        }

        # 异步事件循环
        self.executor = asyncio.new_event_loop()
        self.processing_thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self.processing_thread.start()

        # 定时器
        self.cleanup_timer = self.create_timer(60.0, self._cleanup_expired_sessions)
        self.status_timer = self.create_timer(30.0, self._publish_status)

        logger.info("✅ OnlineDialogueNode初始化完成")
        logger.info("📡 订阅话题: /audio_input, /image_input, /dialogue_input")
        logger.info("💬 发布话题: /dialogue_response, /dialogue_status")

    def audio_callback(self, msg: AudioData):
        """音频输入回调"""
        try:
            self.current_audio = msg
            self.node_stats["audio_messages_received"] += 1
            logger.debug("收到音频数据")
            self._process_multimodal_input("audio")
        except Exception as e:
            logger.error(f"音频回调错误: {e}")
            self.node_stats["failed_responses"] += 1

    def image_callback(self, msg: Image):
        """图像输入回调"""
        try:
            self.current_image = msg
            self.node_stats["image_messages_received"] += 1
            logger.debug("收到图像数据")
            self._process_multimodal_input("image")
        except Exception as e:
            logger.error(f"图像回调错误: {e}")
            self.node_stats["failed_responses"] += 1

    def dialogue_input_callback(self, msg: OnlineDialogueInput):
        """对话输入回调"""
        try:
            self.node_stats["text_messages_received"] += 1

            # 在异步线程中处理
            asyncio.run_coroutine_threadsafe(
                self._async_process_dialogue_input(msg),
                self.executor
            )
        except Exception as e:
            logger.error(f"对话输入回调错误: {e}")
            self.node_stats["failed_responses"] += 1

    def _process_multimodal_input(self, input_type: str):
        """处理多模态输入（音频或图像触发）"""
        # 检查是否有音频输入
        if not self.current_audio:
            return

        # 生成或获取会话ID
        session_id = self._get_or_create_session_id()

        # 转换为在线对话请求
        request = self._create_dialogue_request(session_id, input_type)

        # 在异步线程中处理
        asyncio.run_coroutine_threadsafe(
            self._async_process_dialogue_request(request),
            self.executor
        )

        # 重置输入数据（避免重复处理）
        self.current_audio = None
        self.current_image = None

    def _get_or_create_session_id(self) -> str:
        """获取或创建会话ID"""
        # 尝试获取现有会话
        active_sessions = self.session_manager.get_all_active_sessions()
        if active_sessions:
            # 使用最近活跃的会话
            most_recent_session = max(
                active_sessions,
                key=lambda sid: self.session_manager.get_session_age(sid) or 0
            )
            return most_recent_session
        else:
            # 创建新会话
            return self.session_manager.create_session()

    def _create_dialogue_request(self, session_id: str, input_type: str) -> DialogueRequest:
        """创建在线对话请求"""
        # 编码音频数据
        audio_base64 = None
        if self.current_audio and input_type == "audio":
            audio_base64 = base64.b64encode(self.current_audio.data).decode('utf-8')

        # 编码图像数据
        image_base64 = None
        if self.current_image:
            # 将ROS2 Image转换为Base64
            image_base64 = base64.b64encode(self.current_image.data).decode('utf-8')

        return DialogueRequest(
            session_id=session_id,
            user_input="",  # 可以后续通过文本输入补充
            audio_base64=audio_base64,
            image_base64=image_base64
        )

    async def _async_process_dialogue_input(self, msg: OnlineDialogueInput):
        """异步处理对话输入"""
        try:
            start_time = time.time()

            # 更新会话活动
            self.session_manager.update_session_activity(msg.session_id)

            # 预处理文本
            processed_text = self.text_processor.preprocess_text(msg.user_input)

            # 创建请求
            request = DialogueRequest(
                session_id=msg.session_id,
                user_input=processed_text,
                audio_base64=msg.audio_base64,
                image_base64=msg.image_base64,
                input_type=msg.input_type
            )

            # 调用在线API
            response = await self.dialogue_api.process_dialogue(request)

            # 发布响应
            await self._publish_response(response, msg.session_id)

            # 更新统计
            self.node_stats["dialogues_processed"] += 1
            if response.success:
                self.node_stats["successful_responses"] += 1
            else:
                self.node_stats["failed_responses"] += 1

            processing_time = int((time.time() - start_time) * 1000)
            logger.info(f"📞 对话处理完成 - 响应时间: {processing_time}ms")

        except Exception as e:
            logger.error(f"❌ 对话处理失败: {e}")
            self.node_stats["failed_responses"] += 1

            # 发布错误响应
            error_response = OnlineDialogueResponse()
            error_response.header.stamp = self.get_clock().now().to_msg()
            error_response.session_id = msg.session_id
            error_response.text_response = f"唔好意思，对话服务出现问题: {str(e)}"
            error_response.response_status = 2  # ERROR
            error_response.error_message = str(e)
            error_response.success = False
            self.response_publisher.publish(error_response)

    async def _async_process_dialogue_request(self, request: DialogueRequest):
        """异步处理对话请求"""
        try:
            # 更新会话活动
            self.session_manager.update_session_activity(request.session_id)

            # 调用在线API
            response = await self.dialogue_api.process_dialogue(request)

            # 发布响应
            await self._publish_response(response, request.session_id)

            # 更新统计
            self.node_stats["dialogues_processed"] += 1
            if response.success:
                self.node_stats["successful_responses"] += 1
            else:
                self.node_stats["failed_responses"] += 1

            logger.info(f"📞 对话处理完成 - 会话: {request.session_id}")

        except Exception as e:
            logger.error(f"❌ 对话请求处理失败: {e}")
            self.node_stats["failed_responses"] += 1

    async def _publish_response(self, response, session_id: str):
        """发布对话响应"""
        try:
            msg = OnlineDialogueResponse()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.session_id = session_id
            msg.text_response = response.text_response
            msg.response_status = 1 if response.success else 2
            msg.error_message = response.error_message
            msg.response_latency_ms = response.response_time_ms
            msg.success = response.success

            if response.api_metadata:
                msg.api_metadata = list(response.api_metadata.values())

            self.response_publisher.publish(msg)
            logger.info(f"💬 发布对话回复: {response.text_response[:50]}...")

        except Exception as e:
            logger.error(f"❌ 发布响应失败: {e}")

    def _cleanup_expired_sessions(self):
        """清理过期会话"""
        try:
            self.session_manager.cleanup_all_sessions()
            logger.info("🧹 定期清理过期会话完成")
        except Exception as e:
            logger.error(f"❌ 会话清理失败: {e}")

    def _publish_status(self):
        """发布节点状态"""
        try:
            # 获取组件统计信息
            api_stats = self.dialogue_api.get_api_statistics()
            session_stats = self.session_manager.get_session_statistics()

            status_msg = OnlineDialogueResponse()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.session_id = "status"
            status_msg.text_response = (
                f"在线对话节点状态 - "
                f"API调用: {api_stats['total_calls']}(成功率:{api_stats.get('success_rate', 0):.1%}), "
                f"活跃会话: {session_stats['active_sessions_count']}, "
                f"处理对话: {self.node_stats['dialogues_processed']}"
            )
            status_msg.response_status = 1
            status_msg.response_latency_ms = 0
            status_msg.success = True

            self.status_publisher.publish(status_msg)

        except Exception as e:
            logger.error(f"❌ 状态发布失败: {e}")

    def _run_async_loop(self):
        """运行异步事件循环"""
        asyncio.set_event_loop(self.executor)
        self.executor.run_forever()

    def get_node_statistics(self) -> Dict[str, any]:
        """获取节点统计信息"""
        return {
            "node_stats": self.node_stats,
            "api_stats": self.dialogue_api.get_api_statistics(),
            "session_stats": self.session_manager.get_session_statistics()
        }

    def destroy_node(self):
        """销毁节点"""
        try:
            logger.info("🛑 正在关闭在线对话节点...")

            # 关闭异步循环
            self.executor.call_soon_threadsafe(self.executor.stop)
            self.processing_thread.join(timeout=5.0)

            # 清理资源
            self.session_manager.cleanup_all_sessions()

            logger.info("🏁 在线对话节点已关闭")

        except Exception as e:
            logger.error(f"❌ 节点关闭错误: {e}")

        super().destroy_node()

def main(args=None):
    """主函数"""
    try:
        rclpy.init(args=args)

        logger.info("🚀 启动XleRobot在线对话节点...")
        logger.info("📋 Story 1.7: 多模态在线对话API集成")
        logger.info("📏 BMad Method v6 Brownfield Level 4 - 纯在线架构")

        node = OnlineDialogueNode()

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            logger.info("\n🛑 收到中断信号，正在关闭节点...")
        finally:
            node.destroy_node()

    except Exception as e:
        logger.error(f"❌ 节点启动失败: {e}")
    finally:
        try:
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()