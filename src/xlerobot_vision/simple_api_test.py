#!/usr/bin/env python3.10
"""
简单的API连接测试 - 验证Qwen3-VL-Plus API可用性
Story 1.6: 视觉理解集成开发
"""

import requests
import base64
import json
import os
import tempfile
from PIL import Image, ImageDraw


def create_simple_test_image():
    """创建简单的测试图像"""
    # 创建一个200x200的红色图像，中间有一个白色圆形
    img = Image.new('RGB', (200, 200), 'red')
    draw = ImageDraw.Draw(img)
    draw.ellipse([50, 50, 150, 150], fill='white')

    # 保存到临时文件
    with tempfile.NamedTemporaryFile(suffix='.jpg', delete=False) as tmp:
        img.save(tmp.name)
        return tmp.name


def test_api_connection():
    """测试阿里云API连接"""
    print("🔗 测试Qwen3-VL-Plus API连接")
    print("-" * 40)

    # API配置
    api_key = "sk-600a739fb3f54f338616254c1c69c1f6"
    base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"

    # 创建测试图像
    try:
        image_path = create_simple_test_image()
        print(f"✅ 测试图像创建: {image_path}")

        # 转换为base64
        with open(image_path, 'rb') as f:
            image_data = f.read()
            base64_str = base64.b64encode(image_data).decode('utf-8')

        data_url = f"data:image/jpeg;base64,{base64_str}"
        print("✅ 图像编码完成")

        # 准备API请求
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": "qwen-vl-plus",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "請用廣東話描述呢張圖片嘅內容"
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": data_url
                            }
                        }
                    ]
                }
            ],
            "max_tokens": 200,
            "temperature": 0.7,
            "stream": False
        }

        print("🌐 发送API请求...")

        # 发送请求
        response = requests.post(
            f"{base_url}/chat/completions",
            headers=headers,
            json=data,
            timeout=30
        )

        print(f"📊 响应状态码: {response.status_code}")

        if response.status_code == 200:
            try:
                result = response.json()
                print("✅ API调用成功!")

                if "choices" in result and len(result["choices"]) > 0:
                    content = result["choices"][0]["message"]["content"]
                    print(f"📝 视觉理解结果: {content}")

                if "usage" in result:
                    usage = result["usage"]
                    print(f"📈 Token使用: {usage.get('total_tokens', 0)} (输入: {usage.get('prompt_tokens', 0)}, 输出: {usage.get('completion_tokens', 0)})")

                return True

            except json.JSONDecodeError as e:
                print(f"❌ 响应解析失败: {e}")
                print(f"原始响应: {response.text[:500]}...")
                return False

        else:
            print(f"❌ API调用失败: {response.status_code}")
            try:
                error_info = response.json()
                print(f"错误信息: {error_info}")
            except:
                print(f"错误响应: {response.text}")
            return False

    except requests.exceptions.Timeout:
        print("❌ 请求超时")
        return False
    except requests.exceptions.RequestException as e:
        print(f"❌ 网络请求失败: {e}")
        return False
    except Exception as e:
        print(f"❌ 未知错误: {e}")
        return False
    finally:
        # 清理临时文件
        if 'image_path' in locals() and os.path.exists(image_path):
            os.remove(image_path)
            print("✅ 临时文件已清理")


def test_stream_api():
    """测试流式API"""
    print("\n🌊 测试流式API连接")
    print("-" * 40)

    # API配置
    api_key = "sk-600a739fb3f54f338616254c1c69c1f6"
    base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1"

    # 创建测试图像
    try:
        image_path = create_simple_test_image()

        # 转换为base64
        with open(image_path, 'rb') as f:
            image_data = f.read()
            base64_str = base64.b64encode(image_data).decode('utf-8')

        data_url = f"data:image/jpeg;base64,{base64_str}"

        # 准备API请求
        headers = {
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json"
        }

        data = {
            "model": "qwen-vl-plus",
            "messages": [
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": "簡要用粵語講下呢張圖有乜"
                        },
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": data_url
                            }
                        }
                    ]
                }
            ],
            "max_tokens": 100,
            "temperature": 0.7,
            "stream": True,
            "stream_options": {"include_usage": True}
        }

        print("🌊 发送流式API请求...")
        print("流式输出:", end=" ")

        response = requests.post(
            f"{base_url}/chat/completions",
            headers=headers,
            json=data,
            timeout=30,
            stream=True
        )

        if response.status_code == 200:
            content_received = False
            for line in response.iter_lines():
                if line:
                    line = line.decode('utf-8')
                    if line.startswith('data: '):
                        data_str = line[6:]
                        if data_str == '[DONE]':
                            break
                        try:
                            chunk = json.loads(data_str)
                            if "choices" in chunk and len(chunk["choices"]) > 0:
                                delta = chunk["choices"][0].get("delta", {})
                                if "content" in delta:
                                    print(delta["content"], end='', flush=True)
                                    content_received = True
                            elif "usage" in chunk:
                                usage = chunk["usage"]
                                print(f"\n📈 Token使用: {usage.get('total_tokens', 0)}")
                        except json.JSONDecodeError:
                            continue

            if content_received:
                print("\n✅ 流式API调用成功!")
                return True
            else:
                print("\n❌ 未收到内容")
                return False
        else:
            print(f"\n❌ 流式API调用失败: {response.status_code}")
            return False

    except Exception as e:
        print(f"\n❌ 流式测试失败: {e}")
        return False
    finally:
        # 清理临时文件
        if 'image_path' in locals() and os.path.exists(image_path):
            os.remove(image_path)


def main():
    """主测试函数"""
    print("🧪 Qwen3-VL-Plus API连接测试")
    print("=" * 50)
    print("Story 1.6: 视觉理解集成开发")
    print("=" * 50)

    success_count = 0
    total_tests = 2

    # 测试基本API连接
    if test_api_connection():
        success_count += 1

    # 测试流式API
    if test_stream_api():
        success_count += 1

    # 测试总结
    print("\n" + "=" * 50)
    print("📊 测试总结")
    print("=" * 50)
    print(f"✅ 成功测试: {success_count}/{total_tests}")
    print(f"📈 成功率: {success_count/total_tests*100:.1f}%")

    if success_count == total_tests:
        print("🎉 所有API测试通过!")
        print("💡 Qwen3-VL-Plus API已就绪，可以开始Story 1.6开发")
    else:
        print("⚠️ 部分测试失败，请检查网络和API配置")

    return success_count == total_tests


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)