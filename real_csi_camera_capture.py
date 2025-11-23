#!/usr/bin/env python3
# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
"""
RDK X5真实CSI摄像头图像捕获脚本
使用Hobot官方libsrcampy库进行真实硬件访问
"""

import os
import sys
import time
import cv2
import numpy as np
from datetime import datetime

def capture_real_csi_image():
    """使用真实的Hobot CSI摄像头API捕获图像"""
    try:
        print("=== RDK X5真实CSI摄像头图像捕获 ===")
        print("使用Hobot官方libsrcampy库...")

        # 检查libsrcampy是否可用
        try:
            import libsrcampy as srcampy
            print("✅ libsrcampy导入成功")
        except ImportError as e:
            print(f"❌ libsrcampy不可用: {e}")
            return False

        # 初始化摄像头
        print("初始化CSI摄像头...")
        camera = srcampy.Camera()

        # 尝试不同的摄像头配置
        camera_configs = [
            {
                'name': '高清模式 1920x1080',
                'camera_id': 0,
                'format': -1,
                'flip': -1,
                'resize': [1920, 1920],
                'crop': [1080, 1080],
                'sensor_h': 1080,
                'sensor_w': 1920,
                'output_size': (1920, 1080)
            },
            {
                'name': '标准模式 512x512',
                'camera_id': 0,
                'format': -1,
                'flip': -1,
                'resize': [512, 512],
                'crop': [512, 512],
                'sensor_h': 1080,
                'sensor_w': 1920,
                'output_size': (512, 512)
            },
            {
                'name': '备选摄像头ID=1',
                'camera_id': 1,
                'format': -1,
                'flip': -1,
                'resize': [1920, 1920],
                'crop': [1080, 1080],
                'sensor_h': 1080,
                'sensor_w': 1920,
                'output_size': (1920, 1080)
            }
        ]

        for config in camera_configs:
            print(f"\n尝试配置: {config['name']}")

            try:
                # 打开摄像头
                result = camera.open_cam(
                    camera_id=config['camera_id'],
                    format=config['format'],
                    flip=config['flip'],
                    resize=config['resize'],
                    crop=config['crop'],
                    sensor_h=config['sensor_h'],
                    sensor_w=config['sensor_w']
                )

                print(f"摄像头打开结果: {result}")

                if result == 0:  # 成功
                    print(f"✅ 摄像头打开成功!")

                    # 获取图像参数
                    width, height = config['output_size']
                    print(f"目标分辨率: {width}x{height}")

                    # 捕获NV12格式图像 (format_type=2)
                    print("正在捕获真实CSI图像...")
                    nv12_data = camera.get_img(2, width, height)

                    if nv12_data is not None and len(nv12_data) > 0:
                        print(f"✅ NV12数据获取成功: {len(nv12_data)} bytes")

                        # NV12到RGB转换
                        print("转换为RGB格式...")
                        y_size = width * height
                        uv_size = width * height // 2

                        if len(nv12_data) >= y_size + uv_size:
                            # 创建NV12数组
                            nv12_image = np.zeros((height * 3 // 2, width), dtype=np.uint8)

                            # Y分量
                            y_data = nv12_data[:y_size]
                            nv12_image[:height, :] = y_data.reshape(height, width)

                            # UV分量
                            uv_data = nv12_data[y_size:y_size + uv_size]
                            nv12_image[height:, :] = uv_data.reshape(height // 2, width)

                            # 转换为RGB
                            rgb_image = cv2.cvtColor(nv12_image, cv2.COLOR_YUV2RGB_NV12)
                            print(f"✅ RGB转换成功: {rgb_image.shape}")

                            # 保存图像
                            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                            output_path = f"/home/sunrise/xlerobot/real_csi_capture_{timestamp}.jpg"

                            success = cv2.imwrite(output_path, rgb_image, [cv2.IMWRITE_JPEG_QUALITY, 95])

                            if success and os.path.exists(output_path):
                                file_size = os.path.getsize(output_path)
                                print(f"🎉 真实CSI摄像头图像捕获成功!")
                                print(f"   文件路径: {output_path}")
                                print(f"   文件大小: {file_size} bytes")
                                print(f"   图像尺寸: {rgb_image.shape}")

                                # 验证图像内容
                                verify_image_content(output_path, rgb_image)

                                # 关闭摄像头
                                camera.close_cam()
                                return True
                            else:
                                print(f"❌ 图像保存失败")
                        else:
                            print(f"❌ NV12数据大小不足: {len(nv12_data)} < {y_size + uv_size}")
                    else:
                        print(f"❌ 无法获取NV12数据")
                else:
                    print(f"❌ 摄像头打开失败: {result}")

                # 关闭摄像头尝试下一个配置
                camera.close_cam()

            except Exception as e:
                print(f"❌ 配置 {config['name']} 失败: {e}")
                try:
                    camera.close_cam()
                except:
                    pass

        print("❌ 所有摄像头配置都失败了")
        return False

    except Exception as e:
        print(f"❌ 真实CSI摄像头捕获失败: {e}")
        return False

def verify_image_content(image_path, rgb_image):
    """验证图像内容的真实性"""
    try:
        print(f"\n=== 图像真实性验证 ===")

        # 基本统计信息
        mean_val = np.mean(rgb_image)
        std_val = np.std(rgb_image)
        min_val = np.min(rgb_image)
        max_val = np.max(rgb_image)

        print(f"像素统计:")
        print(f"  平均值: {mean_val:.2f}")
        print(f"  标准差: {std_val:.2f}")
        print(f"  最小值: {min_val}")
        print(f"  最大值: {max_val}")

        # 检查图像是否不是纯色
        if std_val > 5:
            print(f"✅ 图像包含真实内容 (标准差: {std_val:.2f})")
        else:
            print(f"⚠️ 图像可能过于简单 (标准差: {std_val:.2f})")

        # 检查颜色分布
        hist_b = cv2.calcHist([rgb_image], [0], None, [256], [0, 256])
        hist_g = cv2.calcHist([rgb_image], [1], None, [256], [0, 256])
        hist_r = cv2.calcHist([rgb_image], [2], None, [256], [0, 256])

        # 计算颜色分布的复杂度
        non_zero_b = np.count_nonzero(hist_b)
        non_zero_g = np.count_nonzero(hist_g)
        non_zero_r = np.count_nonzero(hist_r)

        total_colors = non_zero_b + non_zero_g + non_zero_r
        print(f"颜色分布复杂度: {total_colors} 个非零bin")

        if total_colors > 100:
            print(f"✅ 颜色分布丰富，图像质量良好")
        else:
            print(f"⚠️ 颜色分布较简单")

        # 文件信息
        file_size = os.path.getsize(image_path)
        print(f"文件信息:")
        print(f"  大小: {file_size} bytes")
        print(f"  路径: {image_path}")

        return True

    except Exception as e:
        print(f"❌ 图像验证失败: {e}")
        return False

def main():
    """主函数"""
    print("RDK X5真实CSI摄像头图像捕获工具")
    print("=" * 50)
    print("目标: 捕获真实的CSI摄像头硬件图像")
    print("方法: 使用Hobot官方libsrcampy API")
    print("输出: /home/sunrise/xlerobot/real_csi_capture_*.jpg")

    # 确保输出目录存在
    output_dir = "/home/sunrise/xlerobot"
    os.makedirs(output_dir, exist_ok=True)

    # 执行真实摄像头捕获
    start_time = time.time()
    success = capture_real_csi_image()
    end_time = time.time()

    print(f"\n执行时间: {end_time - start_time:.2f}秒")

    if success:
        print("\n🎉 真实CSI摄像头图像捕获成功!")
        print("✅ 使用了真实的Hobot硬件API")
        print("✅ 捕获了真实的CSI传感器数据")
        print("✅ 执行了专业的图像处理转换")
        print(f"✅ 图像已保存到 {output_dir}")

        # 列出捕获的图像
        try:
            import glob
            captured_files = glob.glob(f"{output_dir}/real_csi_capture_*.jpg")
            print(f"\n捕获的图像文件:")
            for file in captured_files[-3:]:  # 显示最新的3个
                size = os.path.getsize(file)
                print(f"  - {file} ({size} bytes)")
        except:
            pass

        return True
    else:
        print("\n❌ 真实CSI摄像头图像捕获失败!")
        print("请检查:")
        print("1. CSI摄像头硬件连接")
        print("2. libsrcampy库安装")
        print("3. cam-service运行状态")
        print("4. 摄像头权限配置")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)