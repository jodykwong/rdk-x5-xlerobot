#!/usr/bin/env python3
"""
D-Robotics RDK X5 Camera Driver for ROS2
基于官方libsrcampy API的摄像头驱动
"""
import os
import sys
import time
import numpy as np
import cv2
from typing import Optional, Tuple, Dict, Any

# 尝试导入官方API
try:
    from hobot_vio import libsrcampy as srcampy
    OFFICIAL_API_AVAILABLE = True
    print("✓ Using official hobot_vio.libsrcampy API")
except ImportError:
    try:
        from hobot_vio_rdkx5 import libsrcampy as srcampy
        OFFICIAL_API_AVAILABLE = True
        print("✓ Using hobot_vio_rdkx5.libsrcampy API")
    except ImportError:
        OFFICIAL_API_AVAILABLE = False
        print("✗ Official D-Robotics API not available, falling back to VIN driver")


class DRoboticsCameraDriver:
    """
    D-Robotics RDK X5摄像头驱动

    使用官方libsrcampy API访问IMX219摄像头
    """

    def __init__(self,
                 camera_id: int = 0,
                 width: int = 1920,
                 height: int = 1080,
                 fps: int = 30,
                 output_width: Optional[int] = None,
                 output_height: Optional[int] = None):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.output_width = output_width or width
        self.output_height = output_height or height

        # 摄像头对象
        self.camera = None
        self.is_opened = False
        self.is_initialized = False

        # 显示对象 (可选)
        self.display = None
        self.use_display = False

        # 图像缓存
        self.frame_count = 0
        self.last_capture_time = 0.0

    def initialize(self) -> bool:
        """初始化摄像头"""
        try:
            print(f"Initializing D-Robotics Camera {self.camera_id}...")

            if not OFFICIAL_API_AVAILABLE:
                print("Official API not available")
                return False

            # 创建摄像头对象
            self.camera = srcampy.Camera()
            print("✓ Camera object created")

            # 打开摄像头
            # 参数：camera_id, format, flip, resize_params, crop_params, sensor_height, sensor_width
            success = self.camera.open_cam(
                self.camera_id,           # camera_id
                -1,                       # format (-1 = default)
                -1,                       # flip (-1 = no flip)
                [self.output_width, self.output_width],  # resize_params [input_w, output_w]
                [self.output_height, self.output_height], # crop_params [input_h, output_h]
                self.height,             # sensor_height
                self.width               # sensor_width
            )

            if success:
                print(f"✓ Camera {self.camera_id} opened successfully")
                self.is_opened = True
                self.is_initialized = True
                return True
            else:
                print(f"✗ Failed to open camera {self.camera_id}")
                return False

        except Exception as e:
            print(f"Camera initialization failed: {e}")
            return False

    def capture_frame(self, format_type: int = 2,
                     width: Optional[int] = None,
                     height: Optional[int] = None) -> Optional[np.ndarray]:
        """
        捕获一帧图像

        Args:
            format_type: 图像格式类型 (2 = NV12, 其他格式参考官方文档)
            width: 输出宽度 (None=使用默认值)
            height: 输出高度 (None=使用默认值)

        Returns:
            numpy.ndarray: NV12格式的图像数据
        """
        if not self.is_initialized:
            if not self.initialize():
                return None

        try:
            # 使用官方API获取图像
            output_w = width or self.output_width
            output_h = height or self.output_height

            # 获取图像数据 (NV12格式)
            img_data = self.camera.get_img(format_type, output_w, output_h)

            if img_data is not None:
                # 转换为numpy数组
                frame = np.frombuffer(img_data, dtype=np.uint8)

                # 更新统计信息
                self.frame_count += 1
                self.last_capture_time = time.time()

                return frame
            else:
                print("Camera get_img returned None")
                return None

        except Exception as e:
            print(f"Frame capture failed: {e}")
            return None

    def capture_rgb_frame(self, width: Optional[int] = None,
                         height: Optional[int] = None) -> Optional[np.ndarray]:
        """
        捕获RGB格式图像

        Args:
            width: 输出宽度
            height: 输出高度

        Returns:
            numpy.ndarray: RGB格式的图像数据 (H, W, 3)
        """
        # 获取NV12格式图像
        nv12_frame = self.capture_frame(2, width, height)

        if nv12_frame is None:
            return None

        # 转换NV12到RGB
        return self.nv12_to_rgb(nv12_frame, width or self.output_width, height or self.output_height)

    def nv12_to_rgb(self, nv12_data: np.ndarray, width: int, height: int) -> Optional[np.ndarray]:
        """
        NV12格式转换为RGB

        Args:
            nv12_data: NV12格式的图像数据
            width: 图像宽度
            height: 图像高度

        Returns:
            numpy.ndarray: RGB格式的图像数据
        """
        try:
            # NV12格式: Y平面 (width*height) + UV平面 (width*height/2)
            y_size = width * height
            uv_size = y_size // 2

            if len(nv12_data) < y_size + uv_size:
                print(f"Invalid NV12 data size: {len(nv12_data)}, expected: {y_size + uv_size}")
                return None

            # 提取Y平面
            y_plane = nv12_data[:y_size].reshape(height, width)

            # 使用OpenCV转换
            # 重构NV12格式
            nv12_image = np.zeros((height * 3 // 2, width), dtype=np.uint8)
            nv12_image[:height, :] = y_plane
            nv12_image[height:, :] = nv12_data[y_size:y_size + uv_size]

            # 转换为RGB
            rgb_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2RGB_NV12)
            return rgb_image

        except Exception as e:
            print(f"NV12 to RGB conversion failed: {e}")
            return None

    def setup_display(self, display_id: int = 0) -> bool:
        """设置HDMI显示 (可选)"""
        try:
            if not OFFICIAL_API_AVAILABLE:
                return False

            self.display = srcampy.Display()

            # 获取支持的分辨率
            resolution_list = self.display.get_display_res()
            print(f"Available display resolutions: {resolution_list}")

            # 启动显示
            self.display.display(display_id, self.output_width, self.output_height)
            self.use_display = True

            print(f"✓ Display initialized: {self.output_width}x{self.output_height}")
            return True

        except Exception as e:
            print(f"Display setup failed: {e}")
            return False

    def bind_camera_to_display(self) -> bool:
        """绑定摄像头到显示 (可选)"""
        try:
            if self.camera and self.display:
                srcampy.bind(self.camera, self.display)
                print("✓ Camera bound to display")
                return True
            else:
                print("Camera or display not available")
                return False

        except Exception as e:
            print(f"Camera-Display binding failed: {e}")
            return False

    def get_camera_info(self) -> Dict[str, Any]:
        """获取摄像头信息"""
        return {
            'camera_id': self.camera_id,
            'width': self.width,
            'height': self.height,
            'output_width': self.output_width,
            'output_height': self.output_height,
            'fps': self.fps,
            'is_opened': self.is_opened,
            'is_initialized': self.is_initialized,
            'official_api_available': OFFICIAL_API_AVAILABLE,
            'frame_count': self.frame_count,
            'last_capture_time': self.last_capture_time
        }

    def get_fps(self) -> float:
        """计算当前FPS"""
        if self.frame_count == 0:
            return 0.0

        elapsed_time = time.time() - (self.last_capture_time - (1.0 / self.fps))
        if elapsed_time > 0:
            return self.frame_count / elapsed_time
        else:
            return float(self.fps)

    def cleanup(self):
        """清理资源"""
        try:
            if self.camera and self.is_opened:
                self.camera.close_cam()
                print("Camera closed")

            if self.display:
                self.display.close()
                print("Display closed")

            self.is_opened = False
            self.is_initialized = False

        except Exception as e:
            print(f"Cleanup error: {e}")

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()


def test_camera():
    """测试摄像头功能"""
    print("=== D-Robotics Camera Test ===")

    # 测试基本功能
    try:
        with DRoboticsCameraDriver(
            camera_id=0,
            width=1920,
            height=1080,
            output_width=512,  # 降低分辨率以进行测试
            output_height=512
        ) as camera:

            if camera.is_initialized:
                print("✓ Camera initialized successfully")

                # 测试图像捕获
                print("Testing frame capture...")
                for i in range(5):
                    frame = camera.capture_frame()
                    if frame is not None:
                        print(f"✓ Frame {i+1} captured: {frame.shape} bytes")
                    else:
                        print(f"✗ Frame {i+1} capture failed")

                    time.sleep(0.1)

                # 测试RGB转换
                print("Testing RGB conversion...")
                rgb_frame = camera.capture_rgb_frame(512, 512)
                if rgb_frame is not None:
                    print(f"✓ RGB frame: {rgb_frame.shape}")
                else:
                    print("✗ RGB conversion failed")

                # 显示FPS
                fps = camera.get_fps()
                print(f"Current FPS: {fps:.1f}")

            else:
                print("✗ Camera initialization failed")

    except Exception as e:
        print(f"Camera test failed: {e}")


if __name__ == "__main__":
    test_camera()