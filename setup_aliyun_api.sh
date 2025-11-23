# Aliyun API环境配置指南
# Story 1.3 MVP - 真实API验证

echo "🔧 开始配置阿里云ASR API环境..."

# 加载Story 1.1环境变量配置
if [ -f "/home/sunrise/xlerobot/config/.env.sprint1" ]; then
    source /home/sunrise/xlerobot/config/.env.sprint1
    echo "✅ 已加载Story 1.1环境配置"
fi

# 检查当前环境变量状态
echo "📊 当前环境变量状态:"
echo "ALIBABA_CLOUD_APPKEY: ${ALIBABA_CLOUD_APPKEY:-未设置}"
echo "ALIBABA_CLOUD_ACCESS_KEY_SECRET: ${ALIBABA_CLOUD_ACCESS_KEY_SECRET:-未设置}"
echo "ALIYUN_NLS_APP_KEY: ${ALIYUN_NLS_APP_KEY:-未设置}"
echo "ALIYUN_NLS_APP_SECRET: ${ALIYUN_NLS_APP_SECRET:-未设置}"

# 设置API密钥的示例命令
echo ""
echo "💡 请按以下步骤配置API环境:"
echo ""
echo "1. 登录阿里云控制台 (https://nls-portal.console.aliyun.com/)"
echo "2. 开通智能语音交互服务"
echo "3. 创建项目并获取AppKey和AppSecret或Token"
echo "4. 运行以下命令设置环境变量:"
echo ""
echo "# 方式1: 使用AppSecret (推荐)"
echo 'export ALIYUN_NLS_APP_KEY="your_app_key_here"'
echo 'export ALIYUN_NLS_APP_SECRET="your_app_secret_here"'
echo ""
echo "# 方式2: 使用Token (如果已生成)"
echo 'export ALIYUN_NLS_APP_KEY="your_app_key_here"'
echo 'export ALIYUN_NLS_TOKEN="your_token_here"'
echo ""
echo "5. 验证配置:"
echo "source setup_aliyun_api.sh"
echo ""

# 验证配置是否有效 (优先检查Story 1.1配置)
if [ -n "$ALIBABA_CLOUD_APPKEY" ] && [ -n "$ALIBABA_CLOUD_ACCESS_KEY_SECRET" ]; then
    echo "✅ API环境配置有效 (使用Story 1.1配置)"
    echo "   App Key: ${ALIBABA_CLOUD_APPKEY:0:8}..."
    echo "   App Secret: ${ALIBABA_CLOUD_ACCESS_KEY_SECRET:0:8}..."
    echo ""
    echo "🎯 下一步: 运行真实API验证测试"
    echo "python3 real_api_validation.py"
elif [ -n "$ALIYUN_NLS_APP_KEY" ] && ([ -n "$ALIYUN_NLS_APP_SECRET" ] || [ -n "$ALIYUN_NLS_TOKEN" ]); then
    echo "✅ API环境配置有效 (使用Story 1.3配置)"
    echo "   App Key: ${ALIYUN_NLS_APP_KEY:0:8}..."
    if [ -n "$ALIYUN_NLS_APP_SECRET" ]; then
        echo "   App Secret: ${ALIYUN_NLS_APP_SECRET:0:8}..."
    else
        echo "   Token: ${ALIYUN_NLS_TOKEN:0:8}..."
    fi
    echo ""
    echo "🎯 下一步: 运行真实API验证测试"
    echo "python3 real_api_validation.py"
else
    echo "❌ API环境未配置"
    echo "   检测到Story 1.1配置文件，但环境变量未加载"
    echo "   请检查配置文件中的API密钥是否正确设置"
    echo "   或者设置ALIYUN_NLS_*环境变量"
fi

echo ""
echo "🔗 相关链接:"
echo "- 阿里云语音服务控制台: https://nls-portal.console.aliyun.com/"
echo "- paraformer-v1粤语模型文档: https://help.aliyun.com/zh/nls/developer-reference/api-details"
echo "- 计费说明: https://help.aliyun.com/zh/nls/product-overview/pricing"