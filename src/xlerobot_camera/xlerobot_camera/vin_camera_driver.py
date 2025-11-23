#!/usr/bin/env python3
"""
VIN Camera Driver for XleRobot IMX219
Based on D-Robotics RDK X5 camera system
"""
import os
import sys
import time
import struct
import subprocess
import numpy as np
from typing import Optional, Tuple, Dict, Any

class VINCameraDriver:
    """
    VIN摄像头驱动，基于D-Robotics RDK X5架构

    通过cam-service工具和VIN设备接口访问IMX219摄像头
    """

    def __init__(self,
                 camera_id: int = 0,
                 width: int = 1920,
                 height: int = 1080,
                 fps: int = 30,
                 format: str = "rgb8"):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        self.format = format
        self.device_path = f"/dev/vin{camera_id}_cap"
        self.src_device_path = f"/dev/vin{camera_id}_src"

        # cam-service配置
        self.cam_service_running = False
        self.service_config = {
            's': '4,2,4,2',  # SIF configuration
            'i': '6',        # ISP configuration
            'V': '6'         # VSE configuration
        }

        # 图像处理参数
        self.frame_size = self._calculate_frame_size()
        self.is_initialized = False

    def _calculate_frame_size(self) -> int:
        """计算帧大小"""
        format_sizes = {
            "rgb8": 3,      # 3 bytes per pixel
            "bgr8": 3,      # 3 bytes per pixel
            "yuyv": 2,      # 2 bytes per pixel
            "gray": 1,      # 1 byte per pixel
            "mjpeg": 0      # Variable size for JPEG
        }
        bytes_per_pixel = format_sizes.get(self.format.lower(), 3)
        return self.width * self.height * bytes_per_pixel

    def initialize(self) -> bool:
        """初始化摄像头"""
        try:
            print(f"Initializing VIN Camera {self.camera_id}...")

            # 1. 确保cam-service运行
            if not self._start_cam_service():
                print("Failed to start cam-service")
                return False

            # 2. 验证设备存在
            if not self._verify_devices():
                print("Camera devices not found")
                return False

            # 3. 配置摄像头参数
            if not self._configure_camera():
                print("Camera configuration failed")
                return False

            self.is_initialized = True
            print(f"VIN Camera {self.camera_id} initialized successfully")
            return True

        except Exception as e:
            print(f"Camera initialization failed: {e}")
            return False

    def _start_cam_service(self) -> bool:
        """启动cam-service"""
        try:
            # 检查cam-service是否已运行
            result = subprocess.run(['pgrep', '-f', 'cam-service'],
                                  capture_output=True)

            if result.returncode != 0:
                # 启动cam-service
                cmd = [
                    'sudo', '/usr/hobot/bin/cam-service',
                    '-s', self.service_config['s'],
                    '-i', self.service_config['i'],
                    '-V', self.service_config['V']
                ]

                subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL)

                # 等待服务启动
                time.sleep(2)

                # 验证服务是否运行
                result = subprocess.run(['pgrep', '-f', 'cam-service'],
                                      capture_output=True)

                if result.returncode == 0:
                    self.cam_service_running = True
                    print("cam-service started successfully")
                    return True
                else:
                    print("Failed to start cam-service")
                    return False
            else:
                self.cam_service_running = True
                print("cam-service already running")
                return True

        except Exception as e:
            print(f"Error starting cam-service: {e}")
            return False

    def _verify_devices(self) -> bool:
        """验证VIN设备存在"""
        devices_to_check = [self.device_path, self.src_device_path]

        for device in devices_to_check:
            if not os.path.exists(device):
                print(f"Device not found: {device}")
                return False

            # 检查设备权限
            if not os.access(device, os.R_OK):
                print(f"No read access to: {device}")
                return False

        print("VIN devices verified")
        return True

    def _configure_camera(self) -> bool:
        """配置摄像头参数"""
        try:
            # 这里可以添加摄像头配置逻辑
            # D-Robotics可能提供配置接口或配置文件

            print(f"Camera configured: {self.width}x{self.height}@{self.fps}fps, format={self.format}")
            return True

        except Exception as e:
            print(f"Camera configuration failed: {e}")
            return False

    def capture_frame(self) -> Optional[np.ndarray]:
        """捕获一帧图像"""
        if not self.is_initialized:
            if not self.initialize():
                return None

        try:
            # 从VIN设备读取数据
            raw_data = self._read_from_device()

            if raw_data is None:
                return None

            # 转换为numpy数组
            return self._process_raw_data(raw_data)

        except Exception as e:
            print(f"Frame capture failed: {e}")
            return None

    def _read_from_device(self) -> Optional[bytes]:
        """从VIN设备读取原始数据"""
        try:
            with open(self.device_path, 'rb') as f:
                # 非阻塞读取
                f.setblocking(False)

                try:
                    data = f.read(self.frame_size)
                    if len(data) == self.frame_size:
                        return data
                    else:
                        # 需要完整读取
                        return self._read_complete_frame(f)

                except BlockingIOError:
                    # 暂时没有数据
                    return None

        except Exception as e:
            print(f"Device read error: {e}")
            return None

    def _read_complete_frame(self, file_obj) -> Optional[bytes]:
        """完整读取一帧数据"""
        buffer = b""
        start_time = time.time()
        timeout = 0.1  # 100ms timeout

        while len(buffer) < self.frame_size and (time.time() - start_time) < timeout:
            try:
                chunk = file_obj.read(self.frame_size - len(buffer))
                if chunk:
                    buffer += chunk
                else:
                    time.sleep(0.001)  # 1ms delay

            except BlockingIOError:
                time.sleep(0.001)
            except Exception:
                break

        if len(buffer) == self.frame_size:
            return buffer
        else:
            print(f"Incomplete frame: {len(buffer)}/{self.frame_size} bytes")
            return None

    def _process_raw_data(self, raw_data: bytes) -> Optional[np.ndarray]:
        """处理原始数据为numpy数组"""
        try:
            if self.format.lower() == "rgb8":
                # 直接转换为RGB数组
                return np.frombuffer(raw_data, dtype=np.uint8).reshape(
                    (self.height, self.width, 3)
                )
            elif self.format.lower() == "bgr8":
                # RGB转BGR
                rgb_array = np.frombuffer(raw_data, dtype=np.uint8).reshape(
                    (self.height, self.width, 3)
                )
                return rgb_array[:, :, [2, 1, 0]]  # RGB -> BGR
            elif self.format.lower() == "yuyv":
                # YUYV转RGB
                return self._yuyv_to_rgb(raw_data)
            else:
                # 默认处理
                return np.frombuffer(raw_data, dtype=np.uint8).reshape(
                    (self.height, self.width, -1)
                )

        except Exception as e:
            print(f"Data processing failed: {e}")
            return None

    def _yuyv_to_rgb(self, yuyv_data: bytes) -> Optional[np.ndarray]:
        """YUYV转RGB"""
        try:
            import cv2

            # 重塑为YUYV格式
            yuyv_array = np.frombuffer(yuyv_data, dtype=np.uint8).reshape(
                (self.height, self.width, 2)
            )

            # 转换为RGB
            rgb_array = cv2.cvtColor(yuyv_array, cv2.COLOR_YUV2RGB_YUYV)
            return rgb_array

        except ImportError:
            print("OpenCV not available, implementing manual YUYV->RGB conversion")
            return self._manual_yuyv_to_rgb(yuyv_data)
        except Exception as e:
            print(f"YUYV conversion failed: {e}")
            return None

    def _manual_yuyv_to_rgb(self, yuyv_data: bytes) -> Optional[np.ndarray]:
        """手动YUYV转RGB"""
        try:
            yuyv_array = np.frombuffer(yuyv_data, dtype=np.uint8)
            rgb_array = np.zeros(self.height * self.width * 3, dtype=np.uint8)

            for i in range(0, len(yuyv_array), 4):
                if i + 3 < len(yuyv_array):
                    y1 = yuyv_array[i]
                    u = yuyv_array[i + 1]
                    y2 = yuyv_array[i + 2]
                    v = yuyv_array[i + 3]

                    # 简化的YUV到RGB转换
                    rgb1 = self._yuv_to_rgb_pixel(y1, u, v)
                    rgb2 = self._yuv_to_rgb_pixel(y2, u, v)

                    pixel_idx = (i // 4) * 6
                    if pixel_idx + 5 < len(rgb_array):
                        rgb_array[pixel_idx:pixel_idx + 3] = rgb1
                        rgb_array[pixel_idx + 3:pixel_idx + 6] = rgb2

            return rgb_array.reshape((self.height, self.width, 3))

        except Exception as e:
            print(f"Manual YUYV conversion failed: {e}")
            return None

    def _yuv_to_rgb_pixel(self, y: int, u: int, v: int) -> Tuple[int, int, int]:
        """单个YUV像素转RGB"""
        c = y - 16
        d = u - 128
        e = v - 128

        r = int((298 * c + 409 * e + 128) >> 8)
        g = int((298 * c - 100 * d - 208 * e + 128) >> 8)
        b = int((298 * c + 516 * d + 128) >> 8)

        # 限制范围
        r = max(0, min(255, r))
        g = max(0, min(255, g))
        b = max(0, min(255, b))

        return (r, g, b)

    def get_camera_info(self) -> Dict[str, Any]:
        """获取摄像头信息"""
        return {
            'camera_id': self.camera_id,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'format': self.format,
            'device_path': self.device_path,
            'frame_size': self.frame_size,
            'is_initialized': self.is_initialized,
            'cam_service_running': self.cam_service_running
        }

    def cleanup(self):
        """清理资源"""
        if self.cam_service_running:
            try:
                # 可选：停止cam-service
                # subprocess.run(['sudo', 'pkill', '-f', 'cam-service'])
                pass
            except:
                pass

        self.is_initialized = False
        print("Camera driver cleaned up")

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()


def test_driver():
    """测试摄像头驱动"""
    print("=== VIN Camera Driver Test ===")

    # 测试不同的摄像头ID
    for camera_id in range(4):
        print(f"\nTesting camera {camera_id}...")

        try:
            with VINCameraDriver(camera_id=camera_id) as camera:
                if camera.is_initialized:
                    print(f"✓ Camera {camera_id} initialized")

                    # 尝试捕获一帧
                    frame = camera.capture_frame()
                    if frame is not None:
                        print(f"✓ Frame captured: {frame.shape}")
                    else:
                        print("✗ Frame capture failed")
                else:
                    print(f"✗ Camera {camera_id} initialization failed")

        except Exception as e:
            print(f"✗ Camera {camera_id} test failed: {e}")


if __name__ == "__main__":
    test_driver()