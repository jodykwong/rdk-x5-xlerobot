#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版阿里云TTS客户端（不使用Token）
====================================

根据用户的环境变量，可能TTS只需要Access Key和App Key，不需要单独的Token。

作者: Dev Agent
日期: 2025-11-06
"""

import os
import sys
import json
import base64
import time
import hmac
import hashlib
import requests
import logging

logger = logging.getLogger(__name__)


class AliyunTTSSimple:
    """简化版阿里云TTS客户端"""

    def __init__(self):
        # 从环境变量读取配置
        self.access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', '')
        self.access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')
        self.app_key = 'YOUR_NLS_APPKEY'

        # 端点
        self.endpoint = "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts"

        logger.info("✓ 简化TTS客户端初始化完成")
        logger.info(f"  - App Key: {self.app_key}")
        logger.info(f"  - Access Key: {'✓' if self.access_key_id else '✗'}")

    def synthesize(self, text: str, voice: str = 'xiaoxiao') -> bool:
        """测试TTS语音合成"""
        logger.info(f"🔊 测试TTS: {text}")

        # 构建请求参数（尝试多种格式）
        formats_to_try = [
            # 格式1: 完全扁平化（类似ASR）
            {
                'appkey': self.app_key,
                'voice': voice,
                'text': text,
                'format': 'wav',
                'sample_rate': 22050,
                'language': 'zh'
            },
            # 格式2: 嵌套audio对象
            {
                'appkey': self.app_key,
                'audio': {
                    'voice': voice,
                    'text': text,
                    'format': 'wav',
                    'sample_rate': 22050
                }
            },
            # 格式3: 添加Basic Auth和签名
            {
                'appkey': self.app_key,
                'voice': voice,
                'text': text,
                'format': 'wav',
                'sample_rate': 22050,
                'timestamp': int(time.time() * 1000)
            }
        ]

        for i, request_data in enumerate(formats_to_try, 1):
            logger.info(f"\n尝试格式 {i}...")
            logger.info(f"  请求参数: {json.dumps(request_data, ensure_ascii=False)}")

            # 添加认证头
            auth_string = base64.b64encode(
                f'{self.access_key_id}:{self.access_key_secret}'.encode()
            ).decode()

            headers = {
                'Authorization': f'Basic {auth_string}',
                'Content-Type': 'application/json; charset=UTF-8'
            }

            try:
                response = requests.post(
                    self.endpoint,
                    json=request_data,
                    headers=headers,
                    timeout=10
                )

                logger.info(f"  响应: {response.status_code}")
                logger.info(f"  内容: {response.text[:200]}")

                if response.status_code == 200:
                    result = response.json()
                    if 'audio_file' in result:
                        audio = base64.b64decode(result['audio_file'])
                        logger.info(f"  ✅ 成功! 音频: {len(audio)} bytes")
                        return True

            except Exception as e:
                logger.error(f"  ❌ 异常: {e}")

        logger.error(f"❌ 所有格式都失败")
        return False


def main():
    """主函数"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    logger.info("=" * 70)
    logger.info("🔊 简化版TTS测试（不使用Token）")
    logger.info("=" * 70)

    # 加载.ros2_env文件
    ros2_env_path = os.path.expanduser('~/.ros2_env')
    if os.path.exists(ros2_env_path):
        logger.info(f"✅ 加载ROS2环境配置: {ros2_env_path}")
        with open(ros2_env_path, 'r') as f:
            for line in f:
                line = line.strip()
                if line.startswith('export ') and '=' in line:
                    key_value = line.replace('export ', '')
                    key, value = key_value.split('=', 1)
                    if key in ['ALIBABA_CLOUD_ACCESS_KEY_ID', 'ALIBABA_CLOUD_ACCESS_KEY_SECRET']:
                        os.environ[key] = value.strip('"').strip("'")

    # 创建TTS客户端
    tts = AliyunTTSSimple()

    # 测试TTS
    success = tts.synthesize("你好，我是傻强！")

    if success:
        logger.info("\n" + "=" * 70)
        logger.info("✅ TTS测试成功！")
        logger.info("=" * 70)
    else:
        logger.info("\n" + "=" * 70)
        logger.info("❌ TTS测试失败")
        logger.info("=" * 70)


if __name__ == '__main__':
    main()
