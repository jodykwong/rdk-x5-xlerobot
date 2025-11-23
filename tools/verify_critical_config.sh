#!/bin/bash
# 关键配置验证脚本 - 在每次重大修改后运行

echo "🔍 验证关键配置..."

errors=0

# 检查1: 重采样逻辑存在
if grep -q "audio_processor\|resample.*44100.*16000\|音频重采样.*44100.*16000" src/modules/asr/simple_aliyun_asr_service.py; then
    echo "✅ 重采样逻辑存在"
else
    echo "❌ 重采样逻辑缺失 - 会导致400错误！"
    ((errors++))
fi

# 检查2: 官方SDK Token获取
if grep -q "from nls.token import getToken\|alibabacloud.*nls" src/modules/asr/simple_aliyun_asr_service.py; then
    echo "✅ 使用官方SDK获取Token"
else
    echo "⚠️ 未使用官方SDK - Token获取可能不稳定"
fi

# 检查3: AppKey配置 (检查环境变量获取方式)
if grep -q "ALIYUN_NLS_APPKEY\|os.environ.get.*APPKEY" src/modules/asr/simple_aliyun_asr_service.py; then
    echo "✅ AppKey环境变量配置正确"
else
    echo "❌ AppKey配置方式缺失"
    ((errors++))
fi

# 检查4: 无HTTP API误用
if grep -q "requests.post.*asr\|http.*nls-gateway" src/modules/asr/simple_aliyun_asr_service.py; then
    echo "⚠️ 发现HTTP API调用 - 确认是否正确"
fi

if [ $errors -gt 0 ]; then
    echo "❌ 发现 $errors 个严重问题"
    exit 1
else
    echo "✅ 配置验证通过"
    exit 0
fi