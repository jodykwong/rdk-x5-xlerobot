#!/bin/bash
# 阿里云NLS连接测试脚本
# XleRobot Story 1.1 - 语音唤醒和基础识别
# Brownfield Level 4 企业级测试

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "🔌 阿里云NLS连接测试工具"
echo "=========================="
echo "测试时间: $(date)"
echo "配置文件: /home/sunrise/xlerobot/config/aliyun_nls_config.yaml"
echo ""

# 检查结果变量
TESTS_PASSED=0
TESTS_FAILED=0

# 测试函数
test_result() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✅ $2${NC}"
        ((TESTS_PASSED++))
        return 0
    else
        echo -e "${RED}❌ $2${NC}"
        ((TESTS_FAILED++))
        return 1
    fi
}

info_result() {
    echo -e "${BLUE}ℹ️ $1${NC}"
}

# 解析配置文件
parse_config() {
    python3 << 'EOF'
import yaml
import sys

try:
    with open('/home/sunrise/xlerobot/config/aliyun_nls_config.yaml', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # 获取配置
    appkey = config.get('authentication', {}).get('appkey', '')
    access_key_id = config.get('authentication', {}).get('token', {}).get('access_key_id', '')
    access_key_secret = config.get('authentication', {}).get('token', {}).get('access_key_secret', '')
    token_endpoint = config.get('authentication', {}).get('token', {}).get('endpoint', '')
    websocket_endpoint = config.get('service_endpoints', {}).get('intelligent', {}).get('websocket', '')

    print(f"APPKEY:{appkey}")
    print(f"ACCESS_KEY_ID:{access_key_id}")
    print(f"ACCESS_KEY_SECRET:{access_key_secret}")
    print(f"TOKEN_ENDPOINT:{token_endpoint}")
    print(f"WEBSOCKET_ENDPOINT:{websocket_endpoint}")

except Exception as e:
    print(f"ERROR:{str(e)}")
    sys.exit(1)
EOF
}

echo "🔧 第一步：加载配置"
echo "=================="

CONFIG_OUTPUT=$(parse_config)
APPKEY=$(echo "$CONFIG_OUTPUT" | grep "^APPKEY:" | cut -d: -f2)
ACCESS_KEY_ID=$(echo "$CONFIG_OUTPUT" | grep "^ACCESS_KEY_ID:" | cut -d: -f2)
ACCESS_KEY_SECRET=$(echo "$CONFIG_OUTPUT" | grep "^ACCESS_KEY_SECRET:" | cut -d: -f2)
TOKEN_ENDPOINT=$(echo "$CONFIG_OUTPUT" | grep "^TOKEN_ENDPOINT:" | cut -d: -f2)
WEBSOCKET_ENDPOINT=$(echo "$CONFIG_OUTPUT" | grep "^WEBSOCKET_ENDPOINT:" | cut -d: -f2)

# 检查配置完整性
if [ -z "$APPKEY" ] || [ "$APPKEY" = "YOUR_APPKEY_HERE" ]; then
    echo -e "${RED}❌ 配置不完整，请先运行验证脚本${NC}"
    exit 1
fi

test_result 0 "配置文件加载成功"

echo ""
echo "🔐 第二步：Token获取测试"
echo "===================="

# 测试Token获取
test_result() {
    local appkey="$1"
    local access_key_id="$2"
    local access_key_secret="$3"
    local endpoint="$4"

    # 生成时间戳
    local timestamp=$(date -u +%s)

    # 构建认证字符串
    local auth_string="POST /oauth/v1/token&${timestamp}"
    local signature=$(echo -n "$auth_string" | openssl dgst -sha256 -hmac "$access_key_secret" -binary | base64)

    # 构建请求
    local request_body="{
        \"grant_type\": \"client_credentials\",
        \"client_id\": \"$appkey\",
        \"client_secret\": \"$signature\"
    }"

    # 发送请求
    local response=$(curl -s -X POST "$endpoint" \
        -H "Content-Type: application/json" \
        -H "X-NLS-LOG-DATE: $(date -u +%Y%m%dT%H%M%SZ)" \
        -d "$request_body" \
        --connect-timeout 10 \
        --max-time 30)

    # 解析响应
    if echo "$response" | python3 -c "
import sys, json
try:
    data = json.load(sys.stdin)
    if 'token' in data:
        print('SUCCESS:' + data['token'])
        print('EXPIRES:' + str(data.get('expires_in', 'unknown')))
        sys.exit(0)
    else:
        print('ERROR:' + json.dumps(data, ensure_ascii=False))
        sys.exit(1)
except:
    print('ERROR:Invalid response')
    sys.exit(1)
"; then
        return 0
    else
        return 1
    fi
}

if test_result "$APPKEY" "$ACCESS_KEY_ID" "$ACCESS_KEY_SECRET" "$TOKEN_ENDPOINT"; then
    test_result 0 "Token获取测试成功"
else
    test_result 1 "Token获取测试失败"
    echo -e "${YELLOW}请检查：${NC}"
    echo "1. Access Key ID是否正确"
    echo "2. Access Key Secret是否正确"
    echo "3. 网络连接是否正常"
    echo "4. 阿里云NLS服务是否可用"
fi

echo ""
echo "🌐 第三步：网络连接测试"
echo "===================="

# 测试DNS解析
if [[ $WEBSOCKET_ENDPOINT =~ wss://([^/]+)/ ]]; then
    DOMAIN="${BASH_REMATCH[1]}"
    if nslookup "$DOMAIN" &> /dev/null; then
        test_result 0 "DNS解析成功: $DOMAIN"

        # 测试HTTP连接
        if curl -s --connect-timeout 5 "https://$DOMAIN" > /dev/null; then
            test_result 0 "HTTPS连接测试成功"
        else
            test_result 1 "HTTPS连接测试失败"
        fi
    else
        test_result 1 "DNS解析失败: $DOMAIN"
    fi
else
    test_result 1 "WebSocket端点格式无效"
fi

echo ""
echo "🎤 第四步：音频服务配置测试"
echo "========================"

# 检查音频设备
if arecord -l &> /dev/null; then
    test_result 0 "音频输入设备可用"

    # 检查采样率
    SAMPLE_RATE=$(python3 -c "
import yaml
with open('/home/sunrise/xlerobot/config/aliyun_nls_config.yaml') as f:
    config = yaml.safe_load(f)
    print(config.get('asr', {}).get('sample_rate', 16000))
")
    echo "配置的ASR采样率: ${SAMPLE_RATE}Hz"
else
    test_result 1 "音频输入设备不可用"
fi

# 检查Python依赖
echo "检查Python依赖..."
DEPENDENCIES=("requests" "websocket-client" "pyyaml")
for dep in "${DEPENDENCIES[@]}"; do
    if python3 -c "import $dep" 2>/dev/null; then
        test_result 0 "Python依赖 $dep 可用"
    else
        test_result 1 "Python依赖 $dep 不可用"
    fi
done

echo ""
echo "🔍 第五步：配置完整性检查"
echo "========================"

# 检查关键配置项
echo "关键配置项检查:"

# ASR配置检查
if python3 -c "
import yaml
with open('/home/sunrise/xlerobot/config/aliyun_nls_config.yaml') as f:
    config = yaml.safe_load(f)
    asr = config.get('asr', {})
    tts = config.get('tts', {})
    print('ASR_ENABLED:', asr.get('enabled', False))
    print('TTS_ENABLED:', tts.get('enabled', False))
    print('CANTONESE:', 'cantonese' in str(asr))
    print('MULTILINGUAL:', asr.get('multilingual', {}).get('enabled', False))
" > /dev/null; then
    info_result "配置项解析成功"
else
    test_result 1 "配置项解析失败"
fi

echo ""
echo "📊 测试结果汇总"
echo "================"
echo -e "${GREEN}✅ 通过测试: $TESTS_PASSED 项${NC}"
echo -e "${RED}❌ 失败测试: $TESTS_FAILED 项${NC}"

TOTAL_TESTS=$((TESTS_PASSED + TESTS_FAILED))
SUCCESS_RATE=$((TESTS_PASSED * 100 / TOTAL_TESTS))

echo "总体通过率: $SUCCESS_RATE%"

if [ $TESTS_FAILED -eq 0 ]; then
    echo ""
    echo -e "${GREEN}🎉 所有连接测试都通过了！${NC}"
    echo -e "${GREEN}✅ 阿里云NLS连接正常，可以开始集成工作${NC}"
    echo ""
    echo -e "${BLUE}建议的下一步：${NC}"
    echo "1. 运行ASR功能测试: bash /home/sunrise/xlerobot/scripts/test_asr_functionality.sh"
    echo "2. 开始Story 1.1开发: *develop"
    echo ""
    echo -e "${BLUE}配置文件位置：${NC}"
    echo "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml"
    exit 0
else
    echo ""
    echo -e "${RED}🚨 连接测试失败！${NC}"
    echo -e "${RED}❌ 有 $TESTS_FAILED 项测试未通过${NC}"
    echo ""
    echo -e "${YELLOW}故障排除建议：${NC}"
    echo "1. 检查阿里云账号余额和权限"
    echo "2. 验证Access Key配置"
    echo "3. 检查网络连接和防火墙设置"
    echo "4. 确认阿里云NLS服务状态"
    echo "5. 运行配置验证: bash /home/sunrise/xlerobot/scripts/validate_aliyun_config.sh"
    exit 1
fi