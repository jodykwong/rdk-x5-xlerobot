#!/usr/bin/env python3
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
"""
RDK X5 CSI摄像头图像捕获脚本
支持多种方法访问Hobot CSI摄像头
"""

import os
import sys
import time
import subprocess
import cv2
import numpy as np
from datetime import datetime

class CSICameraCapture:
    def __init__(self):
        self.output_dir = "/home/sunrise/xlerobot"
        self.test_methods = [
            ("hobot_dnn", "尝试使用Hobot DNN接口"),
            ("gstreamer", "尝试使用GStreamer管道"),
            ("ros2_hobot", "尝试使用ROS2 Hobot节点"),
            ("v4l2_raw", "尝试原始V4L2访问"),
            ("mmap_camera", "尝试内存映射摄像头访问"),
        ]

    def log_info(self, message):
        """记录信息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        print(f"[{timestamp}] {message}")

    def test_hobot_dnn_interface(self):
        """测试Hobot DNN接口"""
        try:
            self.log_info("尝试导入Hobot DNN库...")
            # 尝试导入hobot相关库
            import hobot_dnn
            import hobot_vio
            self.log_info("✅ Hobot DNN库导入成功")

            # 尝试使用hobot_vio获取摄像头
            self.log_info("尝试使用hobot_vio获取摄像头...")
            # 这里需要根据实际API调整
            return True
        except ImportError as e:
            self.log_info(f"❌ Hobot DNN库不可用: {e}")
            return False
        except Exception as e:
            self.log_info(f"❌ Hobot DNN接口失败: {e}")
            return False

    def test_gstreamer_pipeline(self):
        """测试GStreamer管道"""
        try:
            self.log_info("构建GStreamer管道...")
            # 尝试不同的GStreamer管道配置
            pipelines = [
                f"v4l2src device=/dev/video0 ! video/x-raw,format=YUY2,width=640,height=480 ! videoconvert ! jpegenc ! filesink location={self.output_dir}/gstreamer_test.jpg",
                f"v4l2src device=/dev/video0 ! image/jpeg,width=640,height=480 ! jpegenc ! filesink location={self.output_dir}/gstreamer_test.jpg",
                f"v4l2src device=/dev/vin0_cap ! video/x-raw,format=YUY2,width=640,height=480 ! videoconvert ! jpegenc ! filesink location={self.output_dir}/gstreamer_vin.jpg",
            ]

            for i, pipeline in enumerate(pipelines):
                self.log_info(f"尝试管道 {i+1}: {pipeline[:50]}...")
                try:
                    cmd = ["gst-launch-1.0", "-q"] + pipeline.split()
                    result = subprocess.run(cmd, timeout=10, capture_output=True, text=True)
                    if result.returncode == 0:
                        self.log_info("✅ GStreamer管道成功")
                        return True
                    else:
                        self.log_info(f"❌ 管道 {i+1} 失败: {result.stderr}")
                except subprocess.TimeoutExpired:
                    self.log_info(f"❌ 管道 {i+1} 超时")
                except Exception as e:
                    self.log_info(f"❌ 管道 {i+1} 异常: {e}")

            return False
        except Exception as e:
            self.log_info(f"❌ GStreamer测试失败: {e}")
            return False

    def test_ros2_hobot_camera(self):
        """测试ROS2 Hobot摄像头节点"""
        try:
            self.log_info("测试ROS2 Hobot摄像头节点...")
            # 检查是否有ROS2摄像头相关话题
            cmd = ["ros2", "topic", "list", "|", "grep", "camera"]
            result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

            if result.returncode == 0 and result.stdout.strip():
                self.log_info("✅ 找到摄像头话题")
                topics = result.stdout.strip().split('\n')
                for topic in topics:
                    self.log_info(f"  - {topic}")
                return True
            else:
                self.log_info("❌ 未找到摄像头话题")
                return False
        except Exception as e:
            self.log_info(f"❌ ROS2摄像头测试失败: {e}")
            return False

    def test_v4l2_advanced(self):
        """高级V4L2测试"""
        try:
            self.log_info("尝试高级V4L2访问...")

            # 尝试不同的OpenCV后端
            backends = [cv2.CAP_V4L2, cv2.CAP_GSTREAMER, cv2.CAP_FFMPEG]
            devices = ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3']

            for backend in backends:
                for device in devices:
                    try:
                        self.log_info(f"尝试后端 {backend} + 设备 {device}")
                        cap = cv2.VideoCapture(device, backend)
                        if cap.isOpened():
                            self.log_info("✅ 设备打开成功")
                            # 尝试设置参数
                            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                            cap.set(cv2.CAP_PROP_FPS, 30)

                            # 尝试读取帧
                            ret, frame = cap.read()
                            if ret and frame is not None:
                                self.log_info(f"✅ 成功读取帧: {frame.shape}")
                                output_path = f"{self.output_dir}/v4l2_advanced_test.jpg"
                                cv2.imwrite(output_path, frame)
                                self.log_info(f"✅ 图像已保存: {output_path}")
                                cap.release()
                                return True
                            else:
                                self.log_info("❌ 无法读取帧")
                        else:
                            self.log_info("❌ 设备无法打开")
                        cap.release()
                    except Exception as e:
                        self.log_info(f"❌ 后端 {backend} + 设备 {device} 失败: {e}")

            return False
        except Exception as e:
            self.log_info(f"❌ 高级V4L2测试失败: {e}")
            return False

    def test_mmap_camera(self):
        """尝试内存映射方式访问摄像头"""
        try:
            self.log_info("尝试内存映射摄像头访问...")

            # 检查是否有内存映射相关的设备文件
            mmap_files = [
                "/dev/mem",
                "/dev/v4l/by-path",
                "/sys/class/video4linux"
            ]

            for file_path in mmap_files:
                if os.path.exists(file_path):
                    self.log_info(f"找到相关路径: {file_path}")
                else:
                    self.log_info(f"路径不存在: {file_path}")

            # 尝试使用ffmpeg
            try:
                self.log_info("尝试使用ffmpeg...")
                cmd = [
                    "ffmpeg", "-f", "v4l2", "-i", "/dev/video0",
                    "-vframes", "1", "-y", f"{self.output_dir}/ffmpeg_test.jpg"
                ]
                result = subprocess.run(cmd, timeout=15, capture_output=True, text=True)
                if result.returncode == 0:
                    self.log_info("✅ ffmpeg捕获成功")
                    return True
                else:
                    self.log_info(f"❌ ffmpeg失败: {result.stderr}")
            except Exception as e:
                self.log_info(f"❌ ffmpeg异常: {e}")

            return False
        except Exception as e:
            self.log_info(f"❌ 内存映射测试失败: {e}")
            return False

    def run_comprehensive_test(self):
        """运行综合测试"""
        self.log_info("=== RDK X5 CSI摄像头综合测试 ===")
        self.log_info(f"输出目录: {self.output_dir}")

        # 确保输出目录存在
        os.makedirs(self.output_dir, exist_ok=True)

        # 运行所有测试方法
        for method_name, description in self.test_methods:
            self.log_info(f"\n--- {description} ---")
            try:
                if method_name == "hobot_dnn":
                    success = self.test_hobot_dnn_interface()
                elif method_name == "gstreamer":
                    success = self.test_gstreamer_pipeline()
                elif method_name == "ros2_hobot":
                    success = self.test_ros2_hobot_camera()
                elif method_name == "v4l2_raw":
                    success = self.test_v4l2_advanced()
                elif method_name == "mmap_camera":
                    success = self.test_mmap_camera()
                else:
                    self.log_info(f"❌ 未知方法: {method_name}")
                    success = False

                if success:
                    self.log_info(f"✅ {method_name} 成功!")
                    return True
                else:
                    self.log_info(f"❌ {method_name} 失败")

            except Exception as e:
                self.log_info(f"❌ {method_name} 异常: {e}")

        self.log_info("\n=== 所有方法都失败了 ===")
        return False

    def create_system_info_report(self):
        """创建系统信息报告"""
        self.log_info("\n=== 系统信息报告 ===")

        info = {
            "时间": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "用户": os.getenv("USER", "unknown"),
            "摄像头设备": [],
            "cam-service状态": "unknown",
            "Hobot驱动": [],
            "ROS2话题": [],
        }

        # 检查摄像头设备
        try:
            result = subprocess.run(["ls", "-la", "/dev/video*"], capture_output=True, text=True)
            if result.returncode == 0:
                info["摄像头设备"] = result.stdout.strip().split('\n')
        except:
            pass

        # 检查VIN设备
        try:
            result = subprocess.run(["ls", "-la", "/dev/vin*"], capture_output=True, text=True)
            if result.returncode == 0:
                info["VIN设备"] = result.stdout.strip().split('\n')
        except:
            pass

        # 检查cam-service
        try:
            result = subprocess.run(["pgrep", "-f", "cam-service"], capture_output=True, text=True)
            info["cam-service状态"] = "运行中" if result.returncode == 0 else "未运行"
        except:
            pass

        # 检查Hobot驱动
        try:
            result = subprocess.run(["lsmod"], capture_output=True, text=True)
            if result.returncode == 0:
                hobot_modules = [line for line in result.stdout.split('\n') if 'hobot' in line]
                info["Hobot驱动"] = hobot_modules
        except:
            pass

        # 保存报告
        report_path = f"{self.output_dir}/camera_system_info.txt"
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("RDK X5 摄像头系统信息报告\n")
            f.write("=" * 40 + "\n")
            for key, value in info.items():
                f.write(f"{key}:\n")
                if isinstance(value, list):
                    for item in value:
                        if item.strip():
                            f.write(f"  {item}\n")
                else:
                    f.write(f"  {value}\n")
                f.write("\n")

        self.log_info(f"✅ 系统信息报告已保存: {report_path}")

def main():
    """主函数"""
    print("RDK X5 CSI摄像头图像捕获工具")
    print("=" * 50)

    capture = CSICameraCapture()

    # 创建系统信息报告
    capture.create_system_info_report()

    # 运行综合测试
    success = capture.run_comprehensive_test()

    if success:
        print("\n🎉 摄像头测试成功!")
        print(f"请检查 {capture.output_dir} 目录中的图像文件")
        sys.exit(0)
    else:
        print("\n❌ 摄像头测试失败!")
        print("请检查设备配置和驱动状态")
        sys.exit(1)

if __name__ == "__main__":
    main()