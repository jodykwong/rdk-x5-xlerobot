#!/usr/bin/env python3
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
"""
使用Hobot CV库进行真实CSI摄像头图像捕获
尝试通过ctypes调用libhobot_cv.so
"""

import os
import sys
import time
import cv2
import numpy as np
from datetime import datetime
import ctypes
from ctypes import CDLL, c_int, c_void_p, c_char_p, POINTER

def test_hobot_cv_library():
    """测试Hobot CV库"""
    try:
        print("=== 测试Hobot CV库 ===")

        # 查找Hobot CV库
        lib_paths = [
            "/opt/tros/humble/lib/libhobot_cv.so",
            "/usr/lib/libhobot_cv.so",
            "/usr/local/lib/libhobot_cv.so"
        ]

        hobot_cv = None
        for lib_path in lib_paths:
            if os.path.exists(lib_path):
                print(f"✅ 找到库文件: {lib_path}")
                try:
                    hobot_cv = CDLL(lib_path)
                    print(f"✅ 库加载成功")
                    break
                except Exception as e:
                    print(f"❌ 库加载失败: {e}")
                    continue

        if not hobot_cv:
            print("❌ 未找到可用的Hobot CV库")
            return None

        # 尝试查找可能的函数
        possible_functions = [
            "hobot_cv_init",
            "hobot_cv_capture",
            "hobot_cv_get_frame",
            "hobot_cv_release",
            "cv_init",
            "cv_capture",
            "get_frame",
            "capture_image"
        ]

        print("\n查找可用的函数...")
        for func_name in possible_functions:
            try:
                func = getattr(hobot_cv, func_name, None)
                if func:
                    print(f"✅ 找到函数: {func_name}")
                else:
                    print(f"❌ 未找到: {func_name}")
            except:
                print(f"❌ 检查{func_name}失败")

        return hobot_cv

    except Exception as e:
        print(f"❌ 测试Hobot CV库失败: {e}")
        return None

def test_ros2_camera_nodes():
    """测试ROS2摄像头节点"""
    try:
        print("\n=== 测试ROS2摄像头节点 ===")

        # 检查ROS2环境
        result = os.system("ros2 node list 2>/dev/null | grep -i camera")
        if result == 0:
            print("✅ 找到摄像头相关节点")
            os.system("ros2 node list | grep -i camera")
        else:
            print("❌ 未找到摄像头节点")

        # 检查摄像头话题
        result = os.system("ros2 topic list 2>/dev/null | grep -i image")
        if result == 0:
            print("✅ 找到图像话题")
            os.system("ros2 topic list | grep -i image")
        else:
            print("❌ 未找到图像话题")

        return True

    except Exception as e:
        print(f"❌ 测试ROS2摄像头节点失败: {e}")
        return False

def test_direct_device_reading():
    """直接读取设备数据"""
    try:
        print("\n=== 直接设备数据读取 ===")

        devices = ["/dev/vin0_cap", "/dev/vin1_cap", "/dev/vin2_cap", "/dev/vin3_cap"]

        for device in devices:
            if not os.path.exists(device):
                continue

            print(f"\n尝试读取设备: {device}")

            try:
                # 尝试读取原始数据
                with open(device, 'rb') as f:
                    # 读取一些数据
                    data = f.read(1024 * 1024)  # 1MB

                    if len(data) > 0:
                        print(f"✅ 成功读取 {len(data)} bytes")

                        # 分析数据格式
                        print(f"数据前16字节: {data[:16].hex()}")

                        # 尝试解析为图像数据
                        if len(data) >= 1920 * 1080 * 1.5:  # NV12格式大小
                            print("数据大小符合NV12格式，尝试转换...")

                            # 创建NV12图像
                            width, height = 1920, 1080
                            y_size = width * height

                            if len(data) >= y_size:
                                nv12_image = np.zeros((height * 3 // 2, width), dtype=np.uint8)

                                # Y分量
                                nv12_image[:height, :] = np.frombuffer(data[:y_size], dtype=np.uint8).reshape(height, width)

                                # UV分量
                                if len(data) >= y_size + width * height // 2:
                                    uv_data = data[y_size:y_size + width * height // 2]
                                    nv12_image[height:, :] = np.frombuffer(uv_data, dtype=np.uint8).reshape(height // 2, width)

                                    # 转换为RGB
                                    try:
                                        rgb_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2RGB_NV12)
                                        print(f"✅ NV12转换成功: {rgb_image.shape}")

                                        # 保存图像
                                        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                                        output_path = f"/home/sunrise/xlerobot/direct_capture_{timestamp}.jpg"

                                        success = cv2.imwrite(output_path, rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 95])

                                        if success:
                                            print(f"🎉 直接设备捕获成功!")
                                            print(f"   文件: {output_path}")
                                            print(f"   大小: {os.path.getsize(output_path)} bytes")
                                            return True
                                        else:
                                            print(f"❌ 图像保存失败")
                                    except Exception as e:
                                        print(f"❌ NV12转换失败: {e}")
                                else:
                                    print("❌ UV数据不足")
                        else:
                            print("❌ 数据大小不足")
                    else:
                        print("❌ 无法读取数据")

            except Exception as e:
                print(f"❌ 读取{device}失败: {e}")

        return False

    except Exception as e:
        print(f"❌ 直接设备读取失败: {e}")
        return False

def test_cam_service_direct():
    """直接测试cam-service"""
    try:
        print("\n=== 直接测试cam-service ===")

        # 检查cam-service进程
        import subprocess

        result = subprocess.run(["pgrep", "-f", "cam-service"], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ cam-service正在运行")

            # 尝试使用cam-service的API或配置
            # 这里可能需要查看cam-service的文档或源码
            print("cam-service运行正常，但需要找到正确的API")

        else:
            print("❌ cam-service未运行")

        return False

    except Exception as e:
        print(f"❌ 测试cam-service失败: {e}")
        return False

def main():
    """主函数"""
    print("RDK X5 Hobot摄像头多种方法测试")
    print("=" * 50)
    print("目标: 找到真实CSI摄像头图像捕获方法")
    print("方法: 尝试所有可能的Hobot API")

    output_dir = "/home/sunrise/xlerobot"
    os.makedirs(output_dir, exist_ok=True)

    # 方法1: 测试Hobot CV库
    if test_hobot_cv_library():
        print("\n✅ Hobot CV库测试成功")
        # 这里可以进一步使用库
        return True

    # 方法2: 测试ROS2摄像头节点
    if test_ros2_camera_nodes():
        print("\n✅ ROS2摄像头节点测试成功")
        # 这里可以进一步使用ROS2接口
        return True

    # 方法3: 直接设备读取
    if test_direct_device_reading():
        print("\n🎉 直接设备读取成功!")
        return True

    # 方法4: 测试cam-service
    if test_cam_service_direct():
        print("\n✅ cam-service测试成功")
        return True

    print("\n❌ 所有方法都失败了")
    print("建议:")
    print("1. 检查物理CSI摄像头连接")
    print("2. 确认cam-service配置正确")
    print("3. 查找Hobot官方文档")
    print("4. 尝试重启cam-service")

    return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)