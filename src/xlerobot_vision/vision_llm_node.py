#!/usr/bin/env python3.10
"""
视觉LLM ROS2节点 - 集成Qwen3-VL-Plus到XleRobot
Story 1.6: 视觉理解集成开发 - 完整实现

功能特性:
- ROS2话题接口
- 多模态输入处理
- 实时视觉问答
- 粤语对话优化
- 上下文管理
- 错误处理和降级
- Brownfield Level 4企业级标准
"""

import os
import sys
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from audio_msg.msg import AudioFrame
from geometry_msgs.msg import PointStamped
import threading
import time
from typing import Dict, Any, List, Optional

# 添加项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

from xlerobot_vision.qwen_vl_client import QwenVLPlusClient, QwenVLConfig, XleRobotVisionError
from xlerobot_vision.multimodal_context import MultimodalContextProcessor


class VisionLLMNode(Node):
    """视觉LLM ROS2节点"""

    def __init__(self):
        super().__init__('vision_llm_node')

        self.get_logger().info("🤖 初始化XleRobot视觉LLM节点...")

        # 配置 - API密钥将从环境变量读取
        self.config = QwenVLConfig(
            timeout=30,
            retry_times=2
        )

        # 核心组件
        self.vision_client = QwenVLPlusClient(self.config)
        self.context_processor = MultimodalContextProcessor()

        # 状态管理
        self.current_image = None
        self.current_image_path = None
        self.processing_lock = threading.Lock()
        self.active_sessions = set()

        # QoS配置
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # 发布者
        self.response_publisher = self.create_publisher(
            String, '/vision/response', qos_profile)
        self.stream_publisher = self.create_publisher(
            String, '/vision/stream', qos_profile)
        self.status_publisher = self.create_publisher(
            String, '/vision/status', qos_profile)

        # LLM集成发布者
        self.llm_response_publisher = self.create_publisher(
            String, '/llm_response', qos_profile)

        # 订阅者
        self.image_subscriber = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, qos_profile)
        self.query_subscriber = self.create_subscription(
            String, '/vision/query', self.query_callback, qos_profile)
        self.session_subscriber = self.create_subscription(
            String, '/vision/session', self.session_callback, qos_profile)

        # LLM集成订阅者
        self.llm_request_subscriber = self.create_subscription(
            String, '/llm_request', self.llm_request_callback, qos_profile)

        # 定时器 - 状态更新
        self.status_timer = self.create_timer(
            5.0, self.publish_status)

        self.get_logger().info("✅ 视觉LLM节点初始化完成")
        self.publish_status()

    def image_callback(self, msg: Image):
        """图像回调 - 缓存最新图像"""
        try:
            # 将ROS图像保存为临时文件
            import tempfile
            import cv2
            from cv_bridge import CvBridge

            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")

            # 保存临时文件
            if self.current_image_path and os.path.exists(self.current_image_path):
                os.remove(self.current_image_path)

            temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
            cv2.imwrite(temp_file.name, cv_image)
            temp_file.close()

            self.current_image_path = temp_file.name
            self.current_image = msg

            self.get_logger().debug(f"📸 接收到图像: {self.current_image_path}")

        except Exception as e:
            self.get_logger().error(f"图像处理失败: {e}")

    def query_callback(self, msg: String):
        """查询回调 - 处理视觉问答请求"""
        with self.processing_lock:
            if not self.current_image_path:
                self.get_logger().warn("⚠️ 没有可用图像")
                self._publish_error_response("没有可用图像，请先拍摄照片")
                return

            # 异步处理
            thread = threading.Thread(
                target=self._process_query,
                args=(msg.data, "default_session"),
                daemon=True
            )
            thread.start()

    def session_callback(self, msg: String):
        """会话回调 - 处理会话管理请求"""
        try:
            data = msg.data
            if data.startswith("CREATE:"):
                # 创建新会话
                session_id = data[7:].strip()
                if session_id:
                    self.context_processor.get_or_create_session(session_id)
                    self.active_sessions.add(session_id)
                    self.get_logger().info(f"🆔 创建会话: {session_id}")

            elif data.startswith("END:"):
                # 结束会话
                session_id = data[4:].strip()
                if session_id in self.active_sessions:
                    self.active_sessions.remove(session_id)
                    self.get_logger().info(f"🔚 结束会话: {session_id}")

            elif data.startswith("QUERY:"):
                # 会话查询
                parts = data[6:].split(":", 1)
                if len(parts) == 2:
                    session_id, query = parts[0].strip(), parts[1].strip()
                    if session_id in self.active_sessions:
                        with self.processing_lock:
                            if not self.current_image_path:
                                self._publish_error_response("没有可用图像，请先拍摄照片")
                                return

                            thread = threading.Thread(
                                target=self._process_query,
                                args=(query, session_id),
                                daemon=True
                            )
                            thread.start()

        except Exception as e:
            self.get_logger().error(f"会话处理失败: {e}")

    def llm_request_callback(self, msg: String):
        """LLM请求回调 - 处理来自主协调器的LLM请求"""
        try:
            request_text = msg.data.strip()
            self.get_logger().info(f"📨 收到LLM请求: {request_text[:30]}...")

            # 检查是否包含视觉相关关键词
            vision_keywords = ["圖", "图", "睇", "看", "影像", "图片", "照片", "畫面", "画面"]
            is_vision_request = any(keyword in request_text for keyword in vision_keywords)

            if is_vision_request and self.current_image_path:
                # 有图像且是视觉相关请求，使用视觉理解
                session_id = f"llm_vision_{int(time.time())}"
                self.active_sessions.add(session_id)

                thread = threading.Thread(
                    target=self._process_query,
                    args=(request_text, session_id),
                    daemon=True
                )
                thread.start()

            elif not is_vision_request:
                # 非视觉相关请求，发布到LLM响应供其他节点处理
                response_msg = String()
                response_msg.data = f"vision_skip:{request_text}"
                self.llm_response_publisher.publish(response_msg)
                self.get_logger().info(f"⏭️ 跳过非视觉请求: {request_text[:30]}...")

            else:
                # 视觉请求但没有图像
                response_msg = String()
                response_msg.data = f"vision_error:没有可用图像，请先拍摄照片"
                self.llm_response_publisher.publish(response_msg)
                self.get_logger().warning("⚠️ 视觉请求但没有可用图像")

        except Exception as e:
            self.get_logger().error(f"LLM请求处理失败: {e}")
            error_msg = String()
            error_msg.data = f"vision_error:LLM请求处理失败: {str(e)}"
            self.llm_response_publisher.publish(error_msg)

    def _process_query(self, query: str, session_id: str):
        """处理视觉查询"""
        try:
            start_time = time.time()
            self.get_logger().info(f"🤔 处理查询 [{session_id}]: {query}")

            # 添加文本输入到上下文
            text_entry_id = self.context_processor.add_multimodal_input(
                session_id, 'text', query)

            # 添加图像输入到上下文
            image_entry_id = None
            if self.current_image_path:
                image_entry_id = self.context_processor.add_multimodal_input(
                    session_id, 'image', self.current_image_path,
                    {'format': 'jpeg', 'timestamp': time.time()})

            # 处理上下文
            context_info = self.context_processor.process_current_context(
                session_id, query, [self.current_image_path] if self.current_image_path else [])

            # 使用优化后的提示词
            optimized_prompt = context_info['optimized_prompt']
            suggested_tokens = context_info['suggested_max_tokens']

            # 调用视觉LLM API
            response_content = ""
            try:
                # 流式响应
                stream_count = 0
                for chunk in self.vision_client.stream_analyze_image(
                    self.current_image_path,
                    optimized_prompt,
                    use_cantonese=True
                ):
                    response_content += chunk
                    stream_count += 1

                    # 发布流式结果
                    stream_msg = String()
                    stream_msg.data = f"{session_id}:{chunk}"
                    self.stream_publisher.publish(stream_msg)

                    # 限制流式输出长度
                    if stream_count > 100:  # 防止过长输出
                        break

                self.get_logger().info(f"🌊 流式响应完成: {len(response_content)}字符")

            except XleRobotVisionError as e:
                self.get_logger().error(f"视觉LLM调用失败: {e.message}")

                # 降级到基础响应
                response_content = "抱歉，视觉理解服務暫時無法使用，請稍後再試。"
                self.get_logger().warn("🔄 降级到基础响应")

            # 存储响应到上下文
            if image_entry_id:
                self.context_processor.store_response(session_id, image_entry_id, response_content)
            else:
                self.context_processor.store_response(session_id, text_entry_id, response_content)

            # 发布完整响应
            response_msg = String()
            response_msg.data = f"{session_id}:{response_content}"
            self.response_publisher.publish(response_msg)

            # 同时发布到LLM响应话题（用于主通信流）
            llm_response_msg = String()
            llm_response_msg.data = f"vision_response:{response_content}"
            self.llm_response_publisher.publish(llm_response_msg)

            # 记录处理时间
            processing_time = time.time() - start_time
            self.get_logger().info(f"✅ 查询处理完成: {processing_time:.2f}秒")

            # 检查性能
            if processing_time > 4.0:
                self.get_logger().warn(f"⚠️ 响应时间过长: {processing_time:.2f}秒")

        except Exception as e:
            self.get_logger().error(f"查询处理失败: {e}")
            self._publish_error_response(f"视觉理解失败: {str(e)}")

    def _publish_error_response(self, error_message: str):
        """发布错误响应"""
        error_msg = String()
        error_msg.data = f"error:{error_message}"
        self.response_publisher.publish(error_msg)

    def publish_status(self):
        """发布节点状态"""
        try:
            # 获取处理器统计
            context_stats = self.context_processor.get_processor_stats()
            client_stats = self.vision_client.get_call_statistics()

            # 构建状态信息
            status_info = {
                'node_name': 'vision_llm_node',
                'timestamp': time.time(),
                'current_image': self.current_image_path is not None,
                'active_sessions': len(self.active_sessions),
                'processing_queries': self.processing_lock.locked(),
                'context_stats': context_stats,
                'client_stats': client_stats
            }

            status_msg = String()
            status_msg.data = str(status_info).replace("'", '"')  # 转换为JSON格式
            self.status_publisher.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"状态发布失败: {e}")

    def cleanup_resources(self):
        """清理资源"""
        try:
            if self.current_image_path and os.path.exists(self.current_image_path):
                os.remove(self.current_image_path)
                self.get_logger().info("🧹 清理临时图像文件")

        except Exception as e:
            self.get_logger().error(f"资源清理失败: {e}")


def main():
    """主函数"""
    try:
        rclpy.init()

        # 创建节点
        node = VisionLLMNode()

        # 设置日志级别
        logger = rclpy.logging.get_logger('vision_llm_node')
        logger.set_level(rclpy.logging.LoggingSeverity.INFO)

        print("🤖 XleRobot视觉LLM节点已启动")
        print("📡 监听话题:")
        print("   /camera/image_raw - 图像输入")
        print("   /vision/query - 视觉查询")
        print("   /vision/session - 会话管理")
        print("   /llm_request - LLM请求（主协调器）")
        print("📢 发布话题:")
        print("   /vision/response - 完整响应")
        print("   /vision/stream - 流式响应")
        print("   /vision/status - 节点状态")
        print("   /llm_response - LLM响应（主通信流）")
        print("\n💡 使用示例:")
        print("   rostopic pub /vision/query std_msgs/String 'data: \"呢張圖有乜嘢？\"'")
        print("   rostopic pub /vision/session std_msgs/String 'data: \"CREATE:my_session\"'")

        # 运行节点
        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\n⏹️ 用户中断")
    except Exception as e:
        print(f"❌ 节点运行失败: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        try:
            node.cleanup_resources()
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("🔚 节点已关闭")


if __name__ == '__main__':
    main()