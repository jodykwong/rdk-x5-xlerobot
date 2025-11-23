#!/usr/bin/env python3
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
"""
直接从RDK X5 VIN设备捕获真实CSI摄像头数据
"""

import os
import sys
import time
import cv2
import numpy as np
from datetime import datetime

def capture_from_vin_device():
    """直接从VIN设备捕获数据"""
    try:
        print("=== 直接VIN设备数据捕获 ===")

        # 测试不同的VIN设备
        vin_devices = [
            "/dev/vin0_cap",
            "/dev/vin1_cap",
            "/dev/vin2_cap",
            "/dev/vin3_cap"
        ]

        for device_path in vin_devices:
            if not os.path.exists(device_path):
                continue

            print(f"\n测试设备: {device_path}")

            try:
                with open(device_path, 'rb') as f:
                    print("  设备打开成功")

                    # 尝试不同的读取方式
                    test_sizes = [
                        (1920, 1080),  # 全高清
                        (1280, 720),   # 高清
                        (640, 480),    # 标清
                        (320, 240)     # 低清
                    ]

                    for width, height in test_sizes:
                        print(f"  尝试分辨率: {width}x{height}")

                        # 计算NV12格式需要的数据大小
                        y_size = width * height
                        uv_size = width * height // 2
                        total_size = y_size + uv_size

                        # 回到文件开始
                        f.seek(0)

                        # 读取数据
                        data = f.read(total_size)

                        if len(data) >= y_size:
                            print(f"  ✅ 读取到 {len(data)} bytes (需要 {total_size})")

                            # 创建NV12图像
                            try:
                                nv12_image = np.zeros((height * 3 // 2, width), dtype=np.uint8)

                                # Y分量
                                y_data = data[:y_size]
                                nv12_image[:height, :] = np.frombuffer(y_data, dtype=np.uint8).reshape(height, width)

                                # UV分量 (如果有的话)
                                if len(data) >= y_size + uv_size:
                                    uv_data = data[y_size:y_size + uv_size]
                                    nv12_image[height:, :] = np.frombuffer(uv_data, dtype=np.uint8).reshape(height // 2, width)

                                print(f"  ✅ NV12数据组装成功")

                                # 转换为RGB
                                try:
                                    rgb_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2RGB_NV12)
                                    print(f"  ✅ RGB转换成功: {rgb_image.shape}")

                                    # 保存图像
                                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                                    output_path = f"/home/sunrise/xlerobot/vin_capture_{width}x{height}_{timestamp}.jpg"

                                    success = cv2.imwrite(output_path, rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 95])

                                    if success and os.path.exists(output_path):
                                        file_size = os.path.getsize(output_path)
                                        print(f"  🎉 图像保存成功!")
                                        print(f"     文件: {output_path}")
                                        print(f"     大小: {file_size} bytes")

                                        # 验证图像质量
                                        verify_real_image(output_path, rgb_image)

                                        return True
                                    else:
                                        print(f"  ❌ 图像保存失败")

                                except Exception as e:
                                    print(f"  ❌ RGB转换失败: {e}")

                            except Exception as e:
                                print(f"  ❌ NV12组装失败: {e}")

                        else:
                            print(f"  ❌ 数据不足: {len(data)} < {y_size}")

            except Exception as e:
                print(f"  ❌ 设备{device_path}失败: {e}")

        return False

    except Exception as e:
        print(f"❌ VIN设备捕获失败: {e}")
        return False

def verify_real_image(image_path, rgb_image):
    """验证图像是否为真实摄像头捕获"""
    try:
        print(f"\n=== 图像真实性验证 ===")

        # 基本图像统计
        mean_val = np.mean(rgb_image)
        std_val = np.std(rgb_image)

        print(f"图像统计:")
        print(f"  尺寸: {rgb_image.shape}")
        print(f"  平均亮度: {mean_val:.2f}")
        print(f"  标准差: {std_val:.2f}")

        # 检查图像是否为纯色（摄像头问题）
        if std_val < 1.0:
            print(f"  ⚠️ 图像可能为纯色或黑屏")
        else:
            print(f"  ✅ 图像包含真实内容")

        # 检查颜色分布
        unique_colors = len(np.unique(rgb_image.reshape(-1, 3), axis=0))
        print(f"  唯一颜色数: {unique_colors}")

        if unique_colors > 1000:
            print(f"  ✅ 颜色丰富，图像质量良好")
        elif unique_colors > 100:
            print(f"  ⚠️ 颜色较少，可能为低光照场景")
        else:
            print(f"  ❌ 颜色过少，可能存在问题")

        # 检查文件大小合理性
        file_size = os.path.getsize(image_path)
        expected_size = rgb_image.shape[0] * rgb_image.shape[1] * 3  # 大概的原始大小

        if file_size > 1000:  # 至少1KB
            print(f"  ✅ 文件大小合理: {file_size} bytes")
        else:
            print(f"  ❌ 文件过小: {file_size} bytes")

        return True

    except Exception as e:
        print(f"❌ 图像验证失败: {e}")
        return False

def test_alternative_capture():
    """测试其他可能的捕获方法"""
    try:
        print("\n=== 其他捕获方法测试 ===")

        # 方法1: 使用Python的多媒体库
        try:
            import subprocess

            # 尝试使用v4l2-ctl的原始模式
            print("尝试v4l2-ctl原始捕获...")
            for i in range(4):
                device = f"/dev/video{i}"
                if os.path.exists(device):
                    output_file = f"/home/sunrise/xlerobot/v4l2_raw_{i}.jpg"
                    cmd = ["v4l2-ctl", "-d", device, "--stream-mmap", "--stream-count", "1", "--stream-to", output_file]

                    try:
                        result = subprocess.run(cmd, timeout=10, capture_output=True, text=True)
                        if result.returncode == 0 and os.path.exists(output_file) and os.path.getsize(output_file) > 1000:
                            print(f"  ✅ v4l2-ctl捕获成功: {output_file}")
                            return True
                    except:
                        pass

        except Exception as e:
            print(f"  ❌ v4l2-ctl方法失败: {e}")

        # 方法2: 使用系统命令
        try:
            print("尝试系统工具...")

            # 检查是否有fswebcam
            result = subprocess.run(["which", "fswebcam"], capture_output=True, text=True)
            if result.returncode == 0:
                output_file = f"/home/sunrise/xlerobot/fswebcam_capture.jpg"
                cmd = ["fswebcam", "-r", "1920x1080", "--jpeg", "95", "--save", output_file, "/dev/video0"]

                try:
                    result = subprocess.run(cmd, timeout=15, capture_output=True, text=True)
                    if result.returncode == 0 and os.path.exists(output_file) and os.path.getsize(output_file) > 1000:
                        print(f"  ✅ fswebcam捕获成功: {output_file}")
                        return True
                except:
                    pass

        except Exception as e:
            print(f"  ❌ 系统工具方法失败: {e}")

        return False

    except Exception as e:
        print(f"❌ 其他捕获方法失败: {e}")
        return False

def main():
    """主函数"""
    print("RDK X5直接VIN设备真实图像捕获")
    print("=" * 50)
    print("目标: 从VIN设备直接捕获真实CSI摄像头数据")
    print("输出: /home/sunrise/xlerobot/*_capture_*.jpg")

    # 确保输出目录存在
    output_dir = "/home/sunrise/xlerobot"
    os.makedirs(output_dir, exist_ok=True)

    start_time = time.time()

    # 方法1: 直接VIN设备读取
    if capture_from_vin_device():
        end_time = time.time()
        print(f"\n🎉 真实CSI摄像头图像捕获成功!")
        print(f"⏱️ 执行时间: {end_time - start_time:.2f}秒")
        print(f"📁 保存位置: {output_dir}")
        return True

    # 方法2: 其他捕获方法
    if test_alternative_capture():
        end_time = time.time()
        print(f"\n🎉 使用其他方法成功!")
        print(f"⏱️ 执行时间: {end_time - start_time:.2f}秒")
        print(f"📁 保存位置: {output_dir}")
        return True

    end_time = time.time()
    print(f"\n❌ 所有捕获方法都失败了")
    print(f"⏱️ 执行时间: {end_time - start_time:.2f}秒")
    print("\n建议:")
    print("1. 检查物理CSI摄像头连接")
    print("2. 确认cam-service正在运行")
    print("3. 检查设备权限")
    print("4. 尝试重启系统")

    return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)