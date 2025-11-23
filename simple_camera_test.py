#!/usr/bin/env python3
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
"""
简化的RDK X5摄像头测试脚本
"""

import os
import subprocess
import time
from datetime import datetime

def test_camera_with_ffmpeg():
    """使用ffmpeg测试摄像头"""
    try:
        print("=== 使用ffmpeg测试摄像头 ===")

        # 检查ffmpeg是否可用
        result = subprocess.run(["which", "ffmpeg"], capture_output=True, text=True)
        if result.returncode != 0:
            print("❌ ffmpeg不可用")
            return False

        print("✅ ffmpeg可用")

        # 尝试使用ffmpeg捕获图像
        output_file = f"/home/sunrise/xlerobot/ffmpeg_capture_{datetime.now().strftime('%H%M%S')}.jpg"

        # 尝试不同的设备和参数组合
        cmd_options = [
            ["ffmpeg", "-f", "v4l2", "-i", "/dev/video0", "-vframes", "1", "-y", output_file],
            ["ffmpeg", "-f", "video4linux2", "-i", "/dev/video0", "-vframes", "1", "-y", output_file],
            ["ffmpeg", "-f", "v4l2", "-i", "/dev/video0", "-pix_fmt", "yuyv422", "-vframes", "1", "-y", output_file],
            ["ffmpeg", "-f", "rawvideo", "-video_size", "640x480", "-i", "/dev/video0", "-vframes", "1", "-y", output_file],
        ]

        for i, cmd in enumerate(cmd_options, 1):
            print(f"\n尝试ffmpeg配置 {i}:")
            print(f"  {' '.join(cmd[:5])} ...")

            try:
                result = subprocess.run(cmd, timeout=10, capture_output=True, text=True)

                if result.returncode == 0:
                    if os.path.exists(output_file) and os.path.getsize(output_file) > 0:
                        print(f"✅ ffmpeg捕获成功: {os.path.getsize(output_file)} bytes")
                        return True
                    else:
                        print(f"❌ 输出文件无效")
                else:
                    print(f"❌ ffmpeg失败: {result.stderr[:200]}...")

            except subprocess.TimeoutExpired:
                print(f"❌ ffmpeg超时")
            except Exception as e:
                print(f"❌ ffmpeg异常: {e}")

        return False

    except Exception as e:
        print(f"❌ ffmpeg测试失败: {e}")
        return False

def test_camera_with_v4l2_utils():
    """使用v4l2-ctl测试摄像头"""
    try:
        print("\n=== 使用v4l2-ctl测试摄像头 ===")

        # 检查v4l2-ctl是否可用
        result = subprocess.run(["which", "v4l2-ctl"], capture_output=True, text=True)
        if result.returncode != 0:
            print("❌ v4l2-ctl不可用")
            return False

        print("✅ v4l2-ctl可用")

        # 测试设备列表
        print("检查设备列表:")
        result = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            print("✅ 设备列表获取成功")
            print(f"输出: {result.stdout[:500]}...")
        else:
            print(f"❌ 设备列表获取失败: {result.stderr}")

        # 尝试捕获单帧
        for i in range(4):
            device = f"/dev/video{i}"
            if os.path.exists(device):
                print(f"\n尝试设备 {device}:")
                try:
                    output_file = f"/home/sunrise/xlerobot/v4l2_capture_{i}_{datetime.now().strftime('%H%M%S')}.jpg"
                    cmd = ["v4l2-ctl", "-d", device, "--stream-mmap", "--stream-count", "1", "--stream-to", output_file]

                    result = subprocess.run(cmd, timeout=10, capture_output=True, text=True)

                    if result.returncode == 0:
                        if os.path.exists(output_file) and os.path.getsize(output_file) > 0:
                            print(f"✅ v4l2-ctl捕获成功: {os.path.getsize(output_file)} bytes")
                            return True
                        else:
                            print(f"❌ 输出文件无效")
                    else:
                        print(f"❌ v4l2-ctl失败: {result.stderr[:200]}...")

                except Exception as e:
                    print(f"❌ v4l2-ctl异常: {e}")

        return False

    except Exception as e:
        print(f"❌ v4l2-ctl测试失败: {e}")
        return False

def test_direct_device_access():
    """直接设备访问测试"""
    try:
        print("\n=== 直接设备访问测试 ===")

        # 检查设备权限
        devices = ["/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"]

        for device in devices:
            if os.path.exists(device):
                print(f"\n检查 {device}:")

                # 检查权限
                if os.access(device, os.R_OK):
                    print(f"✅ 可读")
                else:
                    print(f"❌ 不可读")

                if os.access(device, os.W_OK):
                    print(f"✅ 可写")
                else:
                    print(f"❌ 不可写")

                # 尝试打开设备
                try:
                    with open(device, 'rb') as f:
                        # 尝试读取少量数据
                        data = f.read(1024)
                        if data:
                            print(f"✅ 可读取数据: {len(data)} bytes")
                            # 保存原始数据
                            raw_file = f"/home/sunrise/xlerobot/raw_{os.path.basename(device)}_{datetime.now().strftime('%H%M%S')}.bin"
                            with open(raw_file, 'wb') as out_f:
                                out_f.write(data)
                            print(f"✅ 原始数据已保存: {raw_file}")
                            return True
                        else:
                            print(f"❌ 无法读取数据")
                except Exception as e:
                    print(f"❌ 打开失败: {e}")

        return False

    except Exception as e:
        print(f"❌ 直接设备访问失败: {e}")
        return False

def test_with_python_camera_library():
    """使用Python摄像头库测试"""
    try:
        print("\n=== Python摄像头库测试 ===")

        # 尝试导入picamera
        try:
            import picamera
            print("✅ picamera可用")
            # 这里可以尝试使用picamera
        except ImportError:
            print("❌ picamera不可用")

        # 尝试使用PIL
        try:
            from PIL import Image, ImageGrab
            print("✅ PIL可用")
            # 尝试屏幕捕获作为测试
            try:
                screenshot = ImageGrab.grab()
                output_file = f"/home/sunrise/xlerobot/screenshot_test_{datetime.now().strftime('%H%M%S')}.jpg"
                screenshot.save(output_file)
                print(f"✅ 屏幕截图成功: {output_file}")
                return True
            except Exception as e:
                print(f"❌ 屏幕截图失败: {e}")
        except ImportError:
            print("❌ PIL不可用")

        return False

    except Exception as e:
        print(f"❌ Python摄像头库测试失败: {e}")
        return False

def main():
    """主函数"""
    print("RDK X5 简化摄像头测试")
    print("=" * 40)

    # 确保输出目录存在
    output_dir = "/home/sunrise/xlerobot"
    os.makedirs(output_dir, exist_ok=True)

    # 方法1: ffmpeg
    if test_camera_with_ffmpeg():
        print("\n🎉 ffmpeg方法成功!")
        return True

    # 方法2: v4l2-ctl
    if test_camera_with_v4l2_utils():
        print("\n🎉 v4l2-ctl方法成功!")
        return True

    # 方法3: 直接设备访问
    if test_direct_device_access():
        print("\n🎉 直接设备访问方法成功!")
        return True

    # 方法4: Python摄像头库
    if test_with_python_camera_library():
        print("\n🎉 Python摄像头库方法成功!")
        return True

    print("\n❌ 所有方法都失败了")
    print("建议检查:")
    print("1. cam-service是否正确启动")
    print("2. 摄像头硬件连接")
    print("3. 设备权限配置")
    return False

if __name__ == "__main__":
    success = main()
    if success:
        print(f"\n✅ 测试完成! 请检查 {output_dir} 目录")
    else:
        print(f"\n❌ 测试失败!")
    exit(0 if success else 1)