#!/bin/bash
# 阿里云NLS配置验证脚本
# XleRobot Story 1.1 - 语音唤醒和基础识别
# Brownfield Level 4 企业级验证

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "🔍 阿里云NLS配置验证工具"
echo "=========================="
echo "检查时间: $(date)"
echo "配置文件: /home/sunrise/xlerobot/config/aliyun_nls_config.yaml"
echo ""

# 检查结果变量
CHECKS_PASSED=0
CHECKS_FAILED=0

# 检查函数
check_result() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✅ $2${NC}"
        ((CHECKS_PASSED++))
        return 0
    else
        echo -e "${RED}❌ $2${NC}"
        ((CHECKS_FAILED++))
        return 1
    fi
}

warning_result() {
    echo -e "${YELLOW}⚠️ $1${NC}"
}

info_result() {
    echo -e "${BLUE}ℹ️ $1${NC}"
}

# 检查YAML解析工具
check_yaml_tool() {
    if command -v python3 &> /dev/null; then
        python3 -c "import yaml" 2>/dev/null && echo "PyYAML available" || {
            echo "Installing PyYAML..."
            pip3 install PyYAML > /dev/null 2>&1
        }
    else
        echo -e "${RED}❌ Python3 not available${NC}"
        exit 1
    fi
}

# 解析YAML配置
parse_config() {
    python3 << 'EOF'
import yaml
import sys

try:
    with open('/home/sunrise/xlerobot/config/aliyun_nls_config.yaml', 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # 检查关键配置项
    appkey = config.get('authentication', {}).get('appkey', '')
    access_key_id = config.get('authentication', {}).get('token', {}).get('access_key_id', '')
    access_key_secret = config.get('authentication', {}).get('token', {}).get('access_key_secret', '')

    print(f"APPKEY:{appkey}")
    print(f"ACCESS_KEY_ID:{access_key_id}")
    print(f"ACCESS_KEY_SECRET:{access_key_secret}")

    # 检查服务端点
    websocket_endpoint = config.get('service_endpoints', {}).get('intelligent', {}).get('websocket', '')
    print(f"WEBSOCKET_ENDPOINT:{websocket_endpoint}")

    # 检查ASR配置
    asr_enabled = config.get('asr', {}).get('enabled', False)
    asr_sample_rate = config.get('asr', {}).get('sample_rate', 0)
    print(f"ASR_ENABLED:{asr_enabled}")
    print(f"ASR_SAMPLE_RATE:{asr_sample_rate}")

    # 检查TTS配置
    tts_enabled = config.get('tts', {}).get('enabled', False)
    tts_voice = config.get('tts', {}).get('voice', '')
    print(f"TTS_ENABLED:{tts_enabled}")
    print(f"TTS_VOICE:{tts_voice}")

except Exception as e:
    print(f"ERROR:{str(e)}")
    sys.exit(1)
EOF
}

echo "🔧 第一步：基础环境检查"
echo "===================="

# 检查配置文件存在性
if [ -f "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml" ]; then
    check_result 0 "配置文件存在"
else
    check_result 1 "配置文件不存在"
    exit 1
fi

# 检查YAML解析工具
check_yaml_tool

# 检查ROS2环境
if command -v ros2 &> /dev/null; then
    check_result 0 "ROS2命令可用"
else
    check_result 1 "ROS2命令不可用"
fi

echo ""
echo "🔐 第二步：认证配置检查"
echo "======================"

# 解析配置文件
CONFIG_OUTPUT=$(parse_config)
APPKEY=$(echo "$CONFIG_OUTPUT" | grep "^APPKEY:" | cut -d: -f2)
ACCESS_KEY_ID=$(echo "$CONFIG_OUTPUT" | grep "^ACCESS_KEY_ID:" | cut -d: -f2)
ACCESS_KEY_SECRET=$(echo "$CONFIG_OUTPUT" | grep "^ACCESS_KEY_SECRET:" | cut -d: -f2)

# 检查AppKey
if [ "$APPKEY" != "YOUR_APPKEY_HERE" ] && [ -n "$APPKEY" ]; then
    check_result 0 "AppKey已配置"
else
    check_result 1 "AppKey未配置或使用默认值"
    warning_result "请在配置文件中填写你的阿里云AppKey"
fi

# 检查Access Key ID
if [ "$ACCESS_KEY_ID" != "YOUR_ACCESS_KEY_ID_HERE" ] && [ -n "$ACCESS_KEY_ID" ]; then
    check_result 0 "Access Key ID已配置"
else
    check_result 1 "Access Key ID未配置或使用默认值"
    warning_result "请在配置文件中填写你的阿里云Access Key ID"
fi

# 检查Access Key Secret
if [ "$ACCESS_KEY_SECRET" != "YOUR_ACCESS_KEY_SECRET_HERE" ] && [ -n "$ACCESS_KEY_SECRET" ]; then
    check_result 0 "Access Key Secret已配置"
else
    check_result 1 "Access Key Secret未配置或使用默认值"
    warning_result "请在配置文件中填写你的阿里云Access Key Secret"
fi

echo ""
echo "🌐 第三步：服务端点配置检查"
echo "========================"

WEBSOCKET_ENDPOINT=$(echo "$CONFIG_OUTPUT" | grep "^WEBSOCKET_ENDPOINT:" | cut -d: -f2)

if [ -n "$WEBSOCKET_ENDPOINT" ]; then
    check_result 0 "WebSocket端点已配置: $WEBSOCKET_ENDPOINT"

    # 测试DNS解析
    if [[ $WEBSOCKET_ENDPOINT =~ wss://([^/]+)/ ]]; then
        DOMAIN="${BASH_REMATCH[1]}"
        if nslookup "$DOMAIN" &> /dev/null; then
            check_result 0 "DNS解析正常: $DOMAIN"
        else
            check_result 1 "DNS解析失败: $DOMAIN"
        fi
    fi
else
    check_result 1 "WebSocket端点未配置"
fi

echo ""
echo "🎤 第四步：ASR配置检查"
echo "=================="

ASR_ENABLED=$(echo "$CONFIG_OUTPUT" | grep "^ASR_ENABLED:" | cut -d: -f2)
ASR_SAMPLE_RATE=$(echo "$CONFIG_OUTPUT" | grep "^ASR_SAMPLE_RATE:" | cut -d: -f2)

if [ "$ASR_ENABLED" = "True" ]; then
    check_result 0 "ASR服务已启用"

    if [ "$ASR_SAMPLE_RATE" = "16000" ]; then
        check_result 0 "ASR采样率配置正确: ${ASR_SAMPLE_RATE}Hz"
    else
        warning_result "ASR采样率可能需要调整: ${ASR_SAMPLE_RATE}Hz (建议16000Hz)"
    fi

    # 检查粤语配置
    if grep -q "cantonese.*enabled.*true" /home/sunrise/xlerobot/config/aliyun_nls_config.yaml; then
        check_result 0 "粤语识别已启用"
    else
        warning_result "粤语识别未启用，可能影响Story 1.1验收标准"
    fi
else
    check_result 1 "ASR服务未启用"
fi

echo ""
echo "🔊 第五步：TTS配置检查"
echo "=================="

TTS_ENABLED=$(echo "$CONFIG_OUTPUT" | grep "^TTS_ENABLED:" | cut -d: -f2)
TTS_VOICE=$(echo "$CONFIG_OUTPUT" | grep "^TTS_VOICE:" | cut -d: -f2)

if [ "$TTS_ENABLED" = "True" ]; then
    check_result 0 "TTS服务已启用"

    if [ -n "$TTS_VOICE" ]; then
        check_result 0 "默认发音人已配置: $TTS_VOICE"
    else
        warning_result "默认发音人未配置，将使用xiaoyun"
    fi

    # 检查粤语发音人
    if grep -q "shanshan\|jiajia\|taozi" /home/sunrise/xlerobot/config/aliyun_nls_config.yaml; then
        check_result 0 "粤语发音人已配置"
    else
        warning_result "粤语发音人未配置，建议配置shanshan或jiajia"
    fi
else
    warning_result "TTS服务未启用（Story 1.1可选功能）"
fi

echo ""
echo "🛡️ 第六步：安全检查"
echo "=================="

# 检查文件权限
if [ -r "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml" ]; then
    check_result 0 "配置文件可读"
else
    check_result 1 "配置文件不可读"
fi

# 检查环境变量
if [ -n "$ALIYUN_NLS_ACCESS_KEY_ID" ]; then
    info_result "检测到环境变量 ALIYUN_NLS_ACCESS_KEY_ID"
fi

if [ -n "$ALIYUN_NLS_ACCESS_KEY_SECRET" ]; then
    info_result "检测到环境变量 ALIYUN_NLS_ACCESS_KEY_SECRET"
fi

# 检查是否在版本控制中
if git rev-parse --git-dir > /dev/null 2>&1; then
    if git check-ignore /home/sunrise/xlerobot/config/aliyun_nls_config.yaml > /dev/null 2>&1; then
        check_result 0 "配置文件已在.gitignore中"
    else
        warning_result "配置文件可能已提交到版本控制"
        warning_result "建议将配置文件添加到.gitignore"
    fi
fi

echo ""
echo "📊 验证结果汇总"
echo "================"
echo -e "${GREEN}✅ 通过检查: $CHECKS_PASSED 项${NC}"
echo -e "${RED}❌ 失败检查: $CHECKS_FAILED 项${NC}"

TOTAL_CHECKS=$((CHECKS_PASSED + CHECKS_FAILED))
SUCCESS_RATE=$((CHECKS_PASSED * 100 / TOTAL_CHECKS))

echo "总体通过率: $SUCCESS_RATE%"

if [ $CHECKS_FAILED -eq 0 ]; then
    echo ""
    echo -e "${GREEN}🎉 恭喜！所有配置检查都通过了！${NC}"
    echo -e "${GREEN}✅ 阿里云NLS配置正确，可以开始集成工作${NC}"
    echo ""
    echo -e "${BLUE}下一步操作：${NC}"
    echo "1. 运行环境测试: bash /home/sunrise/xlerobot/scripts/test_aliyun_connection.sh"
    echo "2. 开始Story 1.1实施: *develop"
    exit 0
else
    echo ""
    echo -e "${RED}🚨 配置验证失败！${NC}"
    echo -e "${RED}❌ 有 $CHECKS_FAILED 项检查未通过，请修复配置${NC}"
    echo ""
    echo -e "${YELLOW}请解决以下问题：${NC}"

    if [ "$APPKEY" = "YOUR_APPKEY_HERE" ] || [ -z "$APPKEY" ]; then
        echo "1. 填写AppKey: 从阿里云控制台获取"
    fi

    if [ "$ACCESS_KEY_ID" = "YOUR_ACCESS_KEY_ID_HERE" ] || [ -z "$ACCESS_KEY_ID" ]; then
        echo "2. 填写Access Key ID: 从RAM用户管理获取"
    fi

    if [ "$ACCESS_KEY_SECRET" = "YOUR_ACCESS_KEY_SECRET_HERE" ] || [ -z "$ACCESS_KEY_SECRET" ]; then
        echo "3. 填写Access Key Secret: 从RAM用户管理获取"
    fi

    echo "4. 重新运行验证脚本"
    echo "5. 确保网络连接正常"
    exit 1
fi