#!/usr/bin/env python3
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
"""
使用Hobot DNN接口捕获RDK X5 CSI摄像头图像
"""

import os
import sys
import time
import cv2
import numpy as np
from datetime import datetime

def capture_with_hobot_dnn():
    """使用Hobot DNN接口捕获图像"""
    try:
        print("=== 使用Hobot DNN接口捕获图像 ===")

        # 导入Hobot相关库
        import hobot_dnn
        import hobot_vio

        print("✅ Hobot DNN库导入成功")

        # 尝试使用hobot_vio接口
        print("尝试初始化摄像头...")

        # 方法1: 尝试使用hobot_vio创建摄像头
        try:
            # 尝试找到正确的hobot摄像头API
            print("查找hobot摄像头接口...")

            # 检查hobot_vio模块的可用方法
            import inspect
            hobot_vio_members = [name for name, obj in inspect.getmembers(hobot_vio)]
            print(f"hobot_vio可用方法: {hobot_vio_members[:10]}...")

        except Exception as e:
            print(f"❌ hobot_vio接口检查失败: {e}")

        # 方法2: 尝试使用hobot_dnn处理
        try:
            print("尝试创建DNN处理管道...")

            # 尝试使用hobot的特定方法
            # 这里需要根据实际的hobot API调整
            print("hobot_dnn模块方法:")
            dnn_members = [name for name, obj in inspect.getmembers(hobot_dnn)]
            print(f"{dnn_members[:10]}...")

        except Exception as e:
            print(f"❌ hobot_dnn处理失败: {e}")

        # 方法3: 尝试使用可能的hobot摄像头工具
        try:
            print("尝试查找hobot摄像头工具...")

            # 检查是否有hobot相关的摄像头工具
            import subprocess
            result = subprocess.run(["find", "/opt/tros", "-name", "*camera*", "-type", "f", "-executable"],
                                  capture_output=True, text=True, timeout=10)

            if result.returncode == 0:
                camera_tools = [line for line in result.stdout.split('\n') if line.strip()]
                if camera_tools:
                    print(f"找到hobot摄像头工具: {camera_tools[:5]}")

                    # 尝试运行第一个工具
                    for tool in camera_tools[:3]:  # 只尝试前3个
                        if 'python' not in tool and os.access(tool, os.X_OK):
                            print(f"尝试运行工具: {tool}")
                            try:
                                result = subprocess.run([tool], capture_output=True, text=True, timeout=5)
                                if result.returncode == 0:
                                    print(f"✅ 工具 {tool} 运行成功")
                                    print(f"输出: {result.stdout[:200]}...")
                                else:
                                    print(f"❌ 工具 {tool} 运行失败: {result.stderr}")
                            except Exception as e:
                                print(f"❌ 运行工具 {tool} 异常: {e}")
                else:
                    print("未找到hobot摄像头工具")
            else:
                print("查找摄像头工具失败")

        except Exception as e:
            print(f"❌ 查找hobot工具失败: {e}")

    except ImportError as e:
        print(f"❌ Hobot DNN库导入失败: {e}")
        return False
    except Exception as e:
        print(f"❌ Hobot DNN捕获失败: {e}")
        return False

    return False

def capture_with_opencv_advanced():
    """使用高级OpenCV方法"""
    try:
        print("\n=== 高级OpenCV方法 ===")

        # 尝试不同的API后端
        backends = [
            ("CAP_V4L2", cv2.CAP_V4L2),
            ("CAP_GSTREAMER", cv2.CAP_GSTREAMER),
            ("CAP_FFMPEG", cv2.CAP_FFMPEG),
            ("CAP_ANY", cv2.CAP_ANY)
        ]

        devices = ["/dev/video0", "/dev/video1", "/dev/video2", "/dev/video3"]

        for backend_name, backend_id in backends:
            print(f"\n尝试后端: {backend_name}")

            for device in devices:
                print(f"  尝试设备: {device}")

                try:
                    cap = cv2.VideoCapture(device, backend_id)

                    if not cap.isOpened():
                        print(f"    ❌ 无法打开设备")
                        continue

                    print(f"    ✅ 设备打开成功")

                    # 尝试获取设备属性
                    try:
                        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                        fps = cap.get(cv2.CAP_PROP_FPS)
                        print(f"    设备属性: {width}x{height} @ {fps}fps")
                    except:
                        print(f"    无法获取设备属性")

                    # 尝试设置参数
                    try:
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)
                        cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
                        print(f"    ✅ 参数设置成功")
                    except Exception as e:
                        print(f"    ⚠️ 参数设置失败: {e}")

                    # 尝试读取帧
                    ret, frame = cap.read()
                    if ret and frame is not None:
                        print(f"    ✅ 成功读取帧: {frame.shape}")

                        # 保存图像
                        output_path = f"/home/sunrise/xlerobot/hobot_capture_{backend_name}_{datetime.now().strftime('%H%M%S')}.jpg"
                        success = cv2.imwrite(output_path, frame)

                        if success:
                            print(f"    ✅ 图像已保存: {output_path}")
                            print(f"    图像大小: {os.path.getsize(output_path)} bytes")

                            # 尝试读取更多帧验证稳定性
                            success_count = 0
                            for i in range(5):
                                ret, frame = cap.read()
                                if ret and frame is not None:
                                    success_count += 1
                                time.sleep(0.1)

                            print(f"    稳定性测试: {success_count}/5 帧成功")

                            cap.release()
                            return True
                        else:
                            print(f"    ❌ 图像保存失败")
                    else:
                        print(f"    ❌ 无法读取帧")

                    cap.release()

                except Exception as e:
                    print(f"    ❌ 访问失败: {e}")

        return False

    except Exception as e:
        print(f"❌ 高级OpenCV方法失败: {e}")
        return False

def test_gstreamer_capture():
    """测试GStreamer捕获"""
    try:
        print("\n=== GStreamer捕获测试 ===")

        import subprocess

        # 尝试不同的GStreamer管道
        pipelines = [
            # 管道1: 基本v4l2捕获
            "v4l2src device=/dev/video0 ! videoconvert ! jpegenc ! filesink location=/home/sunrise/xlerobot/gstreamer_test1.jpg",

            # 管道2: 指定格式
            "v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480 ! videoconvert ! jpegenc ! filesink location=/home/sunrise/xlerobot/gstreamer_test2.jpg",

            # 管道3: 使用image/jpeg
            "v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480 ! jpegenc ! filesink location=/home/sunrise/xlerobot/gstreamer_test3.jpg",

            # 管道4: 尝试不同设备
            "v4l2src device=/dev/video1 ! videoconvert ! jpegenc ! filesink location=/home/sunrise/xlerobot/gstreamer_test4.jpg",
        ]

        for i, pipeline in enumerate(pipelines, 1):
            print(f"\n尝试管道 {i}:")
            print(f"  {pipeline[:80]}...")

            try:
                cmd = ["gst-launch-1.0", "-q"] + pipeline.split()
                result = subprocess.run(cmd, timeout=15, capture_output=True, text=True)

                if result.returncode == 0:
                    print(f"  ✅ 管道 {i} 成功")

                    # 检查输出文件
                    output_file = f"/home/sunrise/xlerobot/gstreamer_test{i}.jpg"
                    if os.path.exists(output_file) and os.path.getsize(output_file) > 0:
                        print(f"  ✅ 图像文件已生成: {os.path.getsize(output_file)} bytes")
                        return True
                    else:
                        print(f"  ❌ 图像文件无效")
                else:
                    print(f"  ❌ 管道 {i} 失败")
                    if result.stderr:
                        print(f"    错误: {result.stderr[:200]}...")

            except subprocess.TimeoutExpired:
                print(f"  ❌ 管道 {i} 超时")
            except Exception as e:
                print(f"  ❌ 管道 {i} 异常: {e}")

        return False

    except Exception as e:
        print(f"❌ GStreamer测试失败: {e}")
        return False

def main():
    """主函数"""
    print("RDK X5 Hobot摄像头图像捕获工具")
    print("=" * 50)

    output_dir = "/home/sunrise/xlerobot"
    os.makedirs(output_dir, exist_ok=True)

    # 方法1: Hobot DNN接口
    if capture_with_hobot_dnn():
        print("\n🎉 Hobot DNN方法成功!")
        return True

    # 方法2: 高级OpenCV
    if capture_with_opencv_advanced():
        print("\n🎉 高级OpenCV方法成功!")
        return True

    # 方法3: GStreamer
    if test_gstreamer_capture():
        print("\n🎉 GStreamer方法成功!")
        return True

    print("\n❌ 所有方法都失败了")
    return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)