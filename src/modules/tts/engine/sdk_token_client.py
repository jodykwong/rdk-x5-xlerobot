#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
使用阿里云官方SDK获取Token
=========================

根据官方技术文档：https://help.aliyun.com/zh/isi/getting-started/obtain-an-access-token

作者: Dev Agent
日期: 2025-11-06
"""

import os
import sys
import json
import logging
from pathlib import Path

# 加载.ros2_env环境变量
ros2_env_path = os.path.expanduser('~/.ros2_env')
if os.path.exists(ros2_env_path):
    with open(ros2_env_path, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('export ') and '=' in line:
                key_value = line.replace('export ', '')
                key, value = key_value.split('=', 1)
                if key in ['ALIBABA_CLOUD_ACCESS_KEY_ID', 'ALIBABA_CLOUD_ACCESS_KEY_SECRET',
                          'ALIBABA_CLOUD_REGION', 'ALIBABA_CLOUD_APP_KEY']:
                    os.environ[key] = value.strip('"').strip("'")

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


def get_token_with_sdk():
    """使用阿里云SDK获取Token"""
    try:
        from alibabacloud_nls_python_sdk import NlsCppSdk

        logger.info("=" * 70)
        logger.info("🔑 使用阿里云官方SDK获取Token")
        logger.info("=" * 70)

        # 创建NLS客户端
        client = NlsCppSdk.NlsClient()

        # 配置Access Key
        access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', '')
        access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')
        app_key = os.getenv('ALIBABA_CLOUD_APP_KEY', '4G5BCMccTCW8nC8w')

        logger.info(f"✓ Access Key ID: {access_key_id[:10]}...")
        logger.info(f"✓ App Key: {app_key}")

        # 获取Token
        logger.info("🔑 正在使用SDK获取Token...")
        token = client.getToken(access_key_id, access_key_secret)

        if token:
            logger.info(f"✅ Token获取成功: {token}")
            # 保存Token
            with open('/tmp/aliyun_token_sdk.txt', 'w') as f:
                f.write(token)
            logger.info(f"✓ Token已保存到: /tmp/aliyun_token_sdk.txt")
            return token
        else:
            logger.error("❌ Token获取失败")
            return None

    except Exception as e:
        logger.error(f"❌ SDK获取Token异常: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_tts_with_token(token):
    """使用Token测试TTS"""
    try:
        import base64
        import requests

        app_key = os.getenv('ALIBABA_CLOUD_APP_KEY', '4G5BCMccTCW8nC8w')

        # 构建TTS请求
        tts_data = {
            'appkey': app_key,
            'token': token,
            'audio': {
                'voice': 'xiaoxiao',
                'text': '你好，我是傻强！',
                'audio_format': 'wav',
                'audio_sample_rate': 22050
            }
        }

        headers = {'Content-Type': 'application/json; charset=UTF-8'}

        logger.info("🔊 测试TTS...")
        logger.info(f"✓ 使用Token: {token}")

        response = requests.post(
            "https://nls-gateway.cn-shanghai.aliyuncs.com/stream/v1/tts",
            json=tts_data,
            headers=headers,
            timeout=10
        )

        logger.info(f"✓ TTS响应: {response.status_code}")
        logger.info(f"✓ TTS内容: {response.text[:200]}")

        if response.status_code == 200:
            result = response.json()
            if 'audio_file' in result:
                audio = base64.b64decode(result['audio_file'])
                logger.info(f"✅ TTS成功! 音频: {len(audio)} bytes")

                # 保存音频
                with open('/tmp/tts_sdk_success.wav', 'wb') as f:
                    f.write(audio)
                logger.info(f"✓ 音频已保存: /tmp/tts_sdk_success.wav")
                return True
        else:
            logger.error(f"❌ TTS失败")
            return False

    except Exception as e:
        logger.error(f"❌ TTS测试异常: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    """主函数"""
    # 1. 获取Token
    token = get_token_with_sdk()

    if token:
        # 2. 测试TTS
        success = test_tts_with_token(token)

        if success:
            logger.info("\n" + "=" * 70)
            logger.info("✅ 完整TTS测试成功!")
            logger.info("=" * 70)
        else:
            logger.error("\n❌ TTS测试失败")
    else:
        logger.error("❌ Token获取失败")


if __name__ == '__main__':
    main()
