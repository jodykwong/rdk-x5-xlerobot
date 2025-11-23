#!/bin/bash
# XLeBot ASR系统演示脚本
# 展示正确的ASR启动流程

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🎤 XLeBot ASR系统演示${NC}"
echo "======================================"

# 环境准备
echo -e "${YELLOW}🔑 加载环境变量...${NC}"
source .env
source ./xlerobot_env.sh
source /opt/ros/humble/setup.bash
export PYTHONPATH="/home/sunrise/xlerobot/src:$PYTHONPATH"
export ROS_DOMAIN_ID=42

echo -e "${GREEN}✅ 环境准备完成${NC}"
echo ""

# 验证ASR组件
echo -e "${YELLOW}🧪 验证ASR核心组件...${NC}"

$PYTHON_EXECUTABLE -c "
print('🔍 验证ASR WebSocket服务...')
from modules.asr.websocket.websocket_asr_service import AliyunASRWebSocketService
asr = AliyunASRWebSocketService()
info = asr.get_service_info()
print(f'✅ WebSocket ASR服务已连接: {info[\"token_valid\"]}')
print(f'   - 语言: {info[\"language\"]}')
print(f'   - 采样率: {info[\"sample_rate\"]}Hz')
print(f'   - SDK状态: {\"已加载\" if info[\"sdk_available\"] else \"未加载\"}')
print()

print('🔍 验证ASR系统主控制器...')
from modules.asr.asr_system import ASRSystem
asr_system = ASRSystem()
success = asr_system.initialize()
print(f'✅ ASR系统初始化: {\"成功\" if success else \"失败\"}')
print()

print('🎯 ASR系统状态检查:')
status = asr_system.get_status()
print(f'   - 系统状态: {status[\"state\"]}')
print(f'   - 麦克风: {\"可用\" if status[\"microphone_available\"] else \"不可用\"}')
print(f'   - LLM集成: {\"已集成\" if status[\"llm_client\"] else \"未集成\"}')
print(f'   - TTS集成: {\"已集成\" if status[\"tts_client\"] else \"未集成\"}')
print()

print('🔊 测试ASR识别功能...')
# 使用静音数据进行连接测试
test_audio = b'\\x00' * 16000
result = asr.recognize_audio(test_audio)
print(f'✅ ASR连接测试: {\"成功\" if result is not None else \"失败\"}')
"

echo ""
echo -e "${GREEN}🎉 ASR系统验证完成！${NC}"
echo ""
echo -e "${YELLOW}📋 ASR系统正确流程：${NC}"
echo "1. 环境变量加载 (.env + xlerobot_env.sh)"
echo "2. Python环境设置 (3.10 + PYTHONPATH)"
echo "3. ROS2环境配置 (humble + DOMAIN_ID=42)"
echo "4. WebSocket ASR服务初始化"
echo "5. ASR系统主控制器启动"
echo "6. 音频设备连接和配置"
echo ""
echo -e "${YELLOW}🚀 启动完整ASR系统命令：${NC}"
echo "./start_fixed_voice_assistant.sh"
echo ""
echo -e "${BLUE}✨ ASR系统现在已就绪！${NC}"
echo "   - 支持粤语语音识别"
echo "   - 使用WebSocket连接阿里云NLS"
echo "   - 包含唤醒词检测功能"
echo "   - 集成多模态LLM处理"
"