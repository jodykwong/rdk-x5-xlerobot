#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿里云NLS Gateway Token获取客户端
===============================

根据官方文档：通过SDK获取Access Token
https://help.aliyun.com/zh/isi/getting-started/obtain-an-access-token

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
from typing import Optional, Dict, Any

logger = logging.getLogger(__name__)


class AliyunTokenClient:
    """阿里云NLS Gateway Token获取客户端"""

    def __init__(self):
        """初始化Token客户端"""
        self.access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID', '')
        self.access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET', '')
        self.region = os.getenv('ALIBABA_CLOUD_REGION', 'cn-shanghai')

        # Token获取端点（尝试多个可能的端点）
        self.token_endpoints = [
            f"https://nls-gateway.{self.region}.aliyuncs.com/stream/v1/token",
            f"https://nls-gateway.{self.region}.aliyuncs.com/stream/v1/authentication/token",
            f"https://nls-gateway.{self.region}.aliyuncs.com/iot/token"
        ]

        # 当前Token
        self._current_token: Optional[str] = None
        self._token_expires_at: float = 0

        logger.info("✓ Token客户端初始化完成")
        logger.info(f"  - 区域: {self.region}")
        logger.info(f"  - 端点: {len(self.token_endpoints)}个待尝试")

    def get_token(self, force_refresh: bool = False) -> Optional[str]:
        """
        获取Access Token

        Args:
            force_refresh: 强制刷新Token

        Returns:
            Token字符串或None
        """
        # 检查Token是否有效
        if not force_refresh and self._current_token and time.time() < self._token_expires_at:
            logger.info("✓ 使用缓存的Token")
            return self._current_token

        logger.info("🔑 正在获取新的Access Token...")

        try:
            # 获取新Token
            token = self._fetch_token()
            if token:
                self._current_token = token
                # Token有效期通常是24小时，提前5分钟刷新
                self._token_expires_at = time.time() + (24 * 3600 - 300)
                logger.info(f"✓ Token获取成功: {token[:20]}...")
                return token
            else:
                logger.error("❌ Token获取失败")
                return None

        except Exception as e:
            logger.error(f"❌ Token获取异常: {e}")
            return None

    def _fetch_token(self) -> Optional[str]:
        """实际调用API获取Token"""
        # 生成签名
        timestamp = int(time.time() * 1000)

        # 构建签名字符串
        string_to_sign = f"GET&/&access_key_id={self.access_key_id}&method=GET&timestamp={timestamp}"

        # 生成HMAC-SHA1签名
        signature = base64.b64encode(
            hmac.new(
                self.access_key_secret.encode(),
                string_to_sign.encode(),
                hashlib.sha1
            ).digest()
        ).decode()

        # 构建请求参数
        params = {
            'method': 'GET',
            'timestamp': timestamp,
            'access_key_id': self.access_key_id,
            'signature': signature,
            'signature_type': 'HMAC-SHA1',
            'signature_version': '1.0'
        }

        # 添加Basic Auth头
        auth_string = base64.b64encode(
            f'{self.access_key_id}:{self.access_key_secret}'.encode()
        ).decode()

        headers = {
            'Authorization': f'Basic {auth_string}',
            'Content-Type': 'application/json; charset=UTF-8'
        }

        # 尝试多个端点
        for endpoint in self.token_endpoints:
            logger.info(f"🔑 尝试端点: {endpoint}")
            logger.info(f"🔑 Token请求参数: {json.dumps(params, ensure_ascii=False)}")

            try:
                # 发送请求
                response = requests.get(
                    endpoint,
                    params=params,
                    headers=headers,
                    timeout=10
                )

                logger.info(f"🔑 Token响应状态: {response.status_code}")
                logger.info(f"🔑 Token响应内容: {response.text[:200]}")

                if response.status_code == 200:
                    result = response.json()
                    if 'token' in result:
                        token = result['token']
                        expires_in = result.get('expires_in', 86400)
                        logger.info(f"✓ Token获取成功!")
                        logger.info(f"  - 端点: {endpoint}")
                        logger.info(f"  - Token: {token[:20]}...")
                        logger.info(f"  - 有效期: {expires_in}秒")
                        return token
                    else:
                        logger.error(f"❌ 响应中缺少token字段: {result}")
                        continue
                else:
                    logger.error(f"❌ 端点 {endpoint} 失败: {response.status_code}, {response.text}")
                    continue

            except Exception as e:
                logger.error(f"❌ 端点 {endpoint} 异常: {e}")
                continue

        logger.error("❌ 所有端点都失败了")
        return None

    def test_token(self, token: str) -> bool:
        """测试Token是否有效"""
        logger.info(f"🧪 测试Token: {token[:20]}...")

        # 使用Token调用TTS API进行测试
        test_data = {
            'appkey': '4G5BCMccTCW8nC8w',
            'token': token,
            'text': '测试',
            'voice': 'xiaoxiao',
            'format': 'wav'
        }

        try:
            headers = {'Content-Type': 'application/json; charset=UTF-8'}
            response = requests.post(
                f"https://nls-gateway.{self.region}.aliyuncs.com/stream/v1/tts",
                json=test_data,
                headers=headers,
                timeout=10
            )

            if response.status_code == 200:
                logger.info(f"✓ Token有效!")
                return True
            else:
                error_msg = response.json().get('message', response.text)
                logger.error(f"❌ Token无效: {error_msg}")
                return False

        except Exception as e:
            logger.error(f"❌ Token测试异常: {e}")
            return False


def main():
    """主函数"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    logger.info("=" * 70)
    logger.info("🔑 阿里云NLS Gateway Token获取测试")
    logger.info("=" * 70)

    # 加载.ros2_env文件中的环境变量
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
    else:
        logger.error(f"❌ ROS2环境配置不存在: {ros2_env_path}")
        return

    # 检查环境变量
    if not os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID'):
        logger.error("❌ 未配置ALIBABA_CLOUD_ACCESS_KEY_ID")
        return

    if not os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET'):
        logger.error("❌ 未配置ALIBABA_CLOUD_ACCESS_KEY_SECRET")
        return

    logger.info(f"✓ Access Key ID: {os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID')[:10]}...")

    # 创建Token客户端
    token_client = AliyunTokenClient()

    # 获取Token
    token = token_client.get_token(force_refresh=True)

    if token:
        # 测试Token
        if token_client.test_token(token):
            # 保存Token到文件
            with open('/tmp/aliyun_token.txt', 'w') as f:
                f.write(token)
            logger.info(f"✅ Token已保存到: /tmp/aliyun_token.txt")

            # 设置环境变量
            os.environ['ALIBABA_CLOUD_TOKEN'] = token
            logger.info(f"✅ Token已设置到环境变量")

            logger.info("=" * 70)
            logger.info("✅ Token获取完成!")
            logger.info("=" * 70)
        else:
            logger.error("❌ Token测试失败")
    else:
        logger.error("❌ Token获取失败")


if __name__ == '__main__':
    main()
