#!/usr/bin/env python3
"""
XleRobot IMX219 ROS2 Camera Node
基于D-Robotics RDK X5 VIN接口的ROS2摄像头节点
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParameters

import time
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import Header

from .vin_camera_driver import VINCameraDriver


class XleRobotCameraNode(Node):
    """
    XleRobot IMX219摄像头节点

    通过VIN接口访问IMX219摄像头，发布标准ROS2图像消息
    """

    def __init__(self):
        super().__init__('xlerobot_camera_node')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_id', 0),
                ('camera_name', 'imx219'),
                ('frame_id', 'camera_link'),
                ('image_width', 1920),
                ('image_height', 1080),
                ('fps', 30),
                ('pixel_format', 'rgb8'),
                ('publish_compressed', True),
                ('publish_raw', True),
                ('jpeg_quality', 85),
                ('camera_info_url', ''),
                ('auto_exposure', True),
                ('brightness', 50),
                ('contrast', 50),
            ]
        )

        # 获取参数
        self._load_parameters()

        # 摄像头驱动
        self.camera_driver = None
        self.is_camera_ready = False

        # 发布器
        self._setup_publishers()

        # 服务
        self._setup_services()

        # 统计信息
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0

        # 定时器
        self.timer = None
        self._start_camera()

        # 参数回调
        self.add_on_set_parameters_callback(self._parameter_callback)

        self.get_logger().info(f'XleRobot {self.camera_name} camera node initialized')

    def _load_parameters(self):
        """加载参数"""
        self.camera_id = self.get_parameter('camera_id').value
        self.camera_name = self.get_parameter('camera_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.fps = self.get_parameter('fps').value
        self.pixel_format = self.get_parameter('pixel_format').value
        self.publish_compressed = self.get_parameter('publish_compressed').value
        self.publish_raw = self.get_parameter('publish_raw').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

    def _setup_publishers(self):
        """设置发布器"""
        # 原始图像发布器
        if self.publish_raw:
            self.image_pub = self.create_publisher(
                Image, f'/{self.camera_name}/image_raw', 10
            )

        # 压缩图像发布器
        if self.publish_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage, f'/{self.camera_name}/image_compressed', 10
            )

        # 相机信息发布器
        self.camera_info_pub = self.create_publisher(
            CameraInfo, f'/{self.camera_name}/camera_info', 10
        )

    def _setup_services(self):
        """设置服务"""
        self.set_parameters_service = self.create_service(
            SetParameters,
            f'/{self.camera_name}/set_parameters',
            self._set_parameters_callback
        )

    def _start_camera(self):
        """启动摄像头"""
        try:
            self.get_logger().info(f"Starting camera {self.camera_id}...")

            # 创建摄像头驱动
            self.camera_driver = VINCameraDriver(
                camera_id=self.camera_id,
                width=self.image_width,
                height=self.image_height,
                fps=self.fps,
                format=self.pixel_format
            )

            # 初始化摄像头
            if self.camera_driver.initialize():
                self.is_camera_ready = True
                self.get_logger().info("Camera initialized successfully")

                # 启动定时器
                self.timer = self.create_timer(
                    1.0 / self.fps, self._timer_callback
                )
            else:
                self.get_logger().error("Camera initialization failed")

        except Exception as e:
            self.get_logger().error(f"Camera startup failed: {e}")

    def _timer_callback(self):
        """定时器回调"""
        if not self.is_camera_ready:
            return

        try:
            # 捕获图像
            frame = self.camera_driver.capture_frame()

            if frame is not None:
                self._publish_frame(frame)
                self._update_fps()
            else:
                self.get_logger().debug("Frame capture returned None")

        except Exception as e:
            self.get_logger().error(f"Timer callback error: {e}")

    def _publish_frame(self, frame: np.ndarray):
        """发布图像帧"""
        timestamp = self.get_clock().now().to_msg()

        # 发布原始图像
        if self.publish_raw and hasattr(self, 'image_pub'):
            raw_msg = self._create_image_message(frame, timestamp)
            self.image_pub.publish(raw_msg)

        # 发布压缩图像
        if self.publish_compressed and hasattr(self, 'compressed_pub'):
            compressed_msg = self._create_compressed_image_message(frame, timestamp)
            if compressed_msg:
                self.compressed_pub.publish(compressed_msg)

        # 发布相机信息 (降低频率)
        if self.frame_count % 10 == 0 and hasattr(self, 'camera_info_pub'):
            camera_info_msg = self._create_camera_info_message(timestamp)
            self.camera_info_pub.publish(camera_info_msg)

    def _create_image_message(self, frame: np.ndarray, timestamp) -> Image:
        """创建ROS2 Image消息"""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id

        msg.height = frame.shape[0]
        msg.width = frame.shape[1]

        # 确定编码格式
        if len(frame.shape) == 3:
            if frame.shape[2] == 3:
                msg.encoding = "rgb8"
                msg.step = frame.shape[1] * 3
            elif frame.shape[2] == 4:
                msg.encoding = "rgba8"
                msg.step = frame.shape[1] * 4
            else:
                msg.encoding = "mono8"
                msg.step = frame.shape[1]
        else:
            msg.encoding = "mono8"
            msg.step = frame.shape[1]

        msg.is_bigendian = 0
        msg.data = frame.tobytes()

        return msg

    def _create_compressed_image_message(self, frame: np.ndarray, timestamp) -> CompressedImage:
        """创建压缩图像消息"""
        try:
            import cv2

            # JPEG压缩
            encode_param = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            success, compressed = cv2.imencode('.jpg', frame, encode_param)

            if success:
                msg = CompressedImage()
                msg.header = Header()
                msg.header.stamp = timestamp
                msg.header.frame_id = self.frame_id
                msg.format = "jpeg"
                msg.data = compressed.tobytes()
                return msg
            else:
                self.get_logger().error("JPEG compression failed")
                return None

        except ImportError:
            self.get_logger().warning("OpenCV not available, cannot create compressed image")
            return None
        except Exception as e:
            self.get_logger().error(f"Compressed image creation failed: {e}")
            return None

    def _create_camera_info_message(self, timestamp) -> CameraInfo:
        """创建相机信息消息"""
        msg = CameraInfo()
        msg.header = Header()
        msg.header.stamp = timestamp
        msg.header.frame_id = self.frame_id

        msg.height = self.image_height
        msg.width = self.image_width
        msg.distortion_model = "plumb_bob"

        # IMX219相机参数 (需要实际标定)
        # 这些是示例参数，需要通过相机标定获得
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # 内参矩阵 (假设无畸变)
        fx = fy = 1000.0  # 焦距示例值
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        msg.k = [fx, 0.0, cx,
                 0.0, fy, cy,
                 0.0, 0.0, 1.0]

        # 旋转矩阵 (单位矩阵)
        msg.r = [1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0]

        # 投影矩阵
        msg.p = [fx, 0.0, cx, 0.0,
                 0.0, fy, cy, 0.0,
                 0.0, 0.0, 1.0, 0.0]

        return msg

    def _update_fps(self):
        """更新FPS统计"""
        self.frame_count += 1
        current_time = time.time()

        if current_time - self.last_fps_time >= 1.0:
            self.current_fps = self.frame_count / (current_time - self.last_fps_time)
            self.frame_count = 0
            self.last_fps_time = current_time

            if self.frame_count % 30 == 0:  # 每30帧输出一次
                self.get_logger().info(f"Camera FPS: {self.current_fps:.1f}")

    def _parameter_callback(self, params):
        """参数变更回调"""
        for param in params:
            if param.name == 'fps':
                if self.timer:
                    self.timer.destroy_timer()
                self.timer = self.create_timer(1.0 / param.value, self._timer_callback)
                self.get_logger().info(f"FPS updated to {param.value}")

            elif param.name == 'jpeg_quality':
                self.jpeg_quality = param.value
                self.get_logger().info(f"JPEG quality updated to {param.value}")

        return SetParametersResult(successful=True)

    def _set_parameters_callback(self, request, response):
        """设置参数服务回调"""
        result = self._parameter_callback(request.parameters)
        response.success = result.successful
        return response

    def get_camera_info_dict(self):
        """获取摄像头信息字典"""
        if self.camera_driver:
            return self.camera_driver.get_camera_info()
        return {}

    def destroy_node(self):
        """销毁节点"""
        if self.camera_driver:
            self.camera_driver.cleanup()
        super().destroy_node()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)

    try:
        node = XleRobotCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()