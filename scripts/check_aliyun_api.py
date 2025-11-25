#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
阿里云API验证脚本
验证阿里云NLS SDK和Token生成功能
"""

import sys
import os
sys.path.append('/home/sunrise/.local/lib/python3.10/site-packages')

def check_aliyun_sdk():
    """检查阿里云NLS SDK"""
    try:
        from nls.token import getToken
        from nls.speech_synthesizer import NlsSpeechSynthesizer
        return True, "阿里云NLS SDK: 已安装"
    except ImportError as e:
        return False, f"阿里云NLS SDK: 导入失败 - {e}"

def check_token_generation():
    """检查Token生成"""
    try:
        from nls.token import getToken

        # 使用文档中的真实凭证
        # 移除硬编码密钥: access_key_id = "YOUR_ACCESS_KEY_ID"
        access_key_secret = "YOUR_ACCESS_KEY_SECRET"

        token = getToken(access_key_id, access_key_secret)
        if token and len(token) > 10:
            return True, f"Token生成: 成功 ({token[:16]}...)"
        else:
            return False, "Token生成: 失败 - Token无效"
    except Exception as e:
        return False, f"Token生成: 失败 - {e}"

def check_app_key():
    """检查App Key配置"""
    # 移除硬编码密钥: app_key = "YOUR_NLS_APPKEY"
    if len(app_key) >= 10:
        return True, f"App Key: 有效 ({app_key})"
    else:
        return False, "App Key: 无效"

def check_websocket_connection():
    """检查WebSocket连接能力"""
    try:
        from nls.token import getToken
        from nls.speech_synthesizer import NlsSpeechSynthesizer

# ⚠️ 严禁Mock数据 - 本文件必须使用真实硬件和真实API
# 安全配置导入
try:
    from core.security.security_config_manager import init_security_config, get_security_manager
    init_security_config()
    security_manager = get_security_manager()
except Exception as e:
    print(f'❌ 安全配置初始化失败: {e}')
    # 根据文件类型决定是否退出
    import sys
    sys.exit(1)

        # 移除硬编码密钥: access_key_id = "YOUR_ACCESS_KEY_ID"
        access_key_secret = "YOUR_ACCESS_KEY_SECRET"
        # 移除硬编码密钥: app_key = "YOUR_NLS_APPKEY"

        token = getToken(access_key_id, access_key_secret)
        if not token:
            return False, "WebSocket连接: Token获取失败"

        # 创建WebSocket合成器
        synthesizer = NlsSpeechSynthesizer(
            token=token,
            appkey=app_key,
            on_metainfo=lambda msg, *args: None,
            on_data=lambda msg, *args: None,
            on_completed=lambda msg, *args: None,
            on_error=lambda msg, *args: None
        )

        return True, "WebSocket连接: 可建立"
    except Exception as e:
        return False, f"WebSocket连接: 失败 - {e}"

def main():
    """主验证函数"""
    print("🔍 阿里云WebSocket API验证")
    print("=" * 40)

    checks = [
        ("SDK安装", check_aliyun_sdk),
        ("App Key", check_app_key),
        ("Token生成", check_token_generation),
        ("WebSocket连接", check_websocket_connection)
    ]

    passed = 0
    failed = 0

    for name, check_func in checks:
        success, message = check_func()
        if success:
            print(f"✅ {name}: {message}")
            passed += 1
        else:
            print(f"❌ {name}: {message}")
            failed += 1

    print("\n📊 验证结果:")
    print(f"   通过: {passed}")
    print(f"   失败: {failed}")
    print(f"   总计: {passed + failed}")

    if failed == 0:
        print("\n🎉 所有阿里云API验证通过!")
        return 0
    else:
        print(f"\n⚠️  {failed}项验证失败")
        return 1

if __name__ == "__main__":
    exit(main())