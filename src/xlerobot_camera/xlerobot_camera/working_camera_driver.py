#!/usr/bin/env python3
"""
Working D-Robotics Camera Driver for ROS2
基于测试成功的工作参数
"""
import sys
import os
import time
import numpy as np
import cv2
from typing import Optional, Tuple, Dict, Any

# 直接加载官方API
sys.path.append('/usr/local/lib/python3.10/dist-packages/hobot_vio')
import libsrcampy as srcampy


class WorkingDRoboticsCameraDriver:
    """
    工作中的D-Robotics摄像头驱动

    基于测试成功的工作参数配置
    """

    def __init__(self,
                 camera_id: int = 0,
                 width: int = 1920,
                 height: int = 1080,
                 output_width: int = 1920,
                 output_height: int = 1080,
                 fps: int = 30):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.output_width = output_width
        self.output_height = output_height
        self.fps = fps

        # 摄像头对象
        self.camera = None
        self.is_opened = False
        self.is_initialized = False

        # 工作参数 (基于测试成功的组合)
        self.working_configs = [
            # 配置1: 全分辨率 (测试成功)
            {
                'camera_id': 0,
                'format': -1,
                'flip': -1,
                'resize': [1920, 1920],
                'crop': [1080, 1080],
                'sensor_h': 1080,
                'sensor_w': 1920,
                'output_w': 1920,
                'output_h': 1080
            },
            # 配置2: camera_id=1 (测试成功)
            {
                'camera_id': 1,
                'format': -1,
                'flip': -1,
                'resize': [512, 512],
                'crop': [512, 512],
                'sensor_h': 1080,
                'sensor_w': 1920,
                'output_w': 512,
                'output_h': 512
            }
        ]

        # 统计信息
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0

    def initialize(self) -> bool:
        """初始化摄像头 - 尝试工作配置"""
        print(f"Initializing D-Robotics Camera {self.camera_id}...")

        for i, config in enumerate(self.working_configs):
            print(f"Trying configuration {i+1}: camera_id={config['camera_id']}")

            try:
                # 创建摄像头对象
                self.camera = srcampy.Camera()

                # 使用工作参数打开摄像头
                success = self.camera.open_cam(
                    config['camera_id'],
                    config['format'],
                    config['flip'],
                    config['resize'],
                    config['crop'],
                    config['sensor_h'],
                    config['sensor_w']
                )

                if success:
                    print(f"✓ Configuration {i+1} worked!")
                    self.is_opened = True
                    self.is_initialized = True
                    self.current_config = config
                    return True
                else:
                    print(f"✗ Configuration {i+1} failed")
                    if self.camera:
                        self.camera.close_cam()

            except Exception as e:
                print(f"✗ Configuration {i+1} error: {e}")
                if self.camera:
                    try:
                        self.camera.close_cam()
                    except:
                        pass

        print("All configurations failed")
        return False

    def capture_frame(self, format_type: int = 2) -> Optional[np.ndarray]:
        """
        捕获一帧图像

        Args:
            format_type: 图像格式 (2 = NV12)

        Returns:
            numpy.ndarray: NV12格式的图像数据
        """
        if not self.is_initialized:
            if not self.initialize():
                return None

        try:
            # 使用当前配置的输出尺寸
            output_w = self.current_config['output_w']
            output_h = self.current_config['output_h']

            # 获取图像数据
            img_data = self.camera.get_img(format_type, output_w, output_h)

            if img_data is not None:
                # 转换为numpy数组
                frame = np.frombuffer(img_data, dtype=np.uint8)

                # 更新统计信息
                self.frame_count += 1
                current_time = time.time()
                if current_time - self.last_fps_time >= 1.0:
                    self.current_fps = self.frame_count / (current_time - self.last_fps_time)
                    self.frame_count = 0
                    self.last_fps_time = current_time

                return frame
            else:
                return None

        except Exception as e:
            print(f"Frame capture failed: {e}")
            return None

    def capture_rgb_frame(self) -> Optional[np.ndarray]:
        """
        捕获RGB格式图像

        Returns:
            numpy.ndarray: RGB格式的图像数据 (H, W, 3)
        """
        # 获取NV12格式图像
        nv12_frame = self.capture_frame(2)

        if nv12_frame is None:
            return None

        # 转换NV12到RGB
        return self.nv12_to_rgb(nv12_frame)

    def nv12_to_rgb(self, nv12_data: np.ndarray) -> Optional[np.ndarray]:
        """
        NV12格式转换为RGB

        Args:
            nv12_data: NV12格式的图像数据

        Returns:
            numpy.ndarray: RGB格式的图像数据
        """
        try:
            # 获取输出尺寸
            width = self.current_config['output_w']
            height = self.current_config['output_h']

            # NV12格式: Y平面 (width*height) + UV平面 (width*height/2)
            y_size = width * height
            uv_size = y_size // 2

            if len(nv12_data) < y_size + uv_size:
                print(f"Invalid NV12 data size: {len(nv12_data)}, expected: {y_size + uv_size}")
                return None

            # 重构NV12格式用于OpenCV处理
            nv12_image = np.zeros((height * 3 // 2, width), dtype=np.uint8)
            nv12_image[:height, :] = nv12_data[:y_size].reshape(height, width)
            nv12_image[height:, :] = nv12_data[y_size:y_size + uv_size].reshape(height // 2, width)

            # 使用OpenCV转换为RGB
            rgb_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2RGB_NV12)
            return rgb_image

        except Exception as e:
            print(f"NV12 to RGB conversion failed: {e}")
            return None

    def get_camera_info(self) -> Dict[str, Any]:
        """获取摄像头信息"""
        info = {
            'camera_id': self.camera_id,
            'width': self.width,
            'height': self.height,
            'output_width': self.output_width,
            'output_height': self.output_height,
            'fps': self.fps,
            'is_opened': self.is_opened,
            'is_initialized': self.is_initialized,
            'current_fps': self.current_fps,
        }

        if hasattr(self, 'current_config'):
            info.update({
                'working_camera_id': self.current_config['camera_id'],
                'actual_output_width': self.current_config['output_w'],
                'actual_output_height': self.current_config['output_h']
            })

        return info

    def cleanup(self):
        """清理资源"""
        try:
            if self.camera and self.is_opened:
                self.camera.close_cam()
                print("Camera closed")

            self.is_opened = False
            self.is_initialized = False

        except Exception as e:
            print(f"Cleanup error: {e}")

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()


def test_working_driver():
    """测试工作驱动"""
    print("=== Testing Working D-Robotics Camera Driver ===")

    try:
        with WorkingDRoboticsCameraDriver() as camera:
            if camera.is_initialized:
                print("✓ Camera initialized successfully")
                print(f"Camera info: {camera.get_camera_info()}")

                # 测试图像捕获
                print("Testing frame capture...")
                for i in range(5):
                    rgb_frame = camera.capture_rgb_frame()
                    if rgb_frame is not None:
                        print(f"✓ RGB frame {i+1}: {rgb_frame.shape}")
                    else:
                        print(f"✗ RGB frame {i+1} failed")

                    time.sleep(0.5)

                # 显示最终FPS
                final_info = camera.get_camera_info()
                print(f"Final FPS: {final_info['current_fps']:.1f}")

            else:
                print("✗ Camera initialization failed")

    except Exception as e:
        print(f"Test failed: {e}")


if __name__ == "__main__":
    test_working_driver()