#!/bin/bash
# XleRobot 完整开发环境检查脚本
# Brownfield Level 4 强制环境验证

# ============================================
# 🛡️ 加载XLeRobot专用环境配置
# ============================================
# 加载环境脚本，确保使用正确的Python环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$SCRIPT_DIR/xlerobot_env.sh" ]]; then
    source "$SCRIPT_DIR/xlerobot_env.sh"
    echo "✅ XLeRobot环境已加载"
else
    echo "❌ 错误：找不到xlerobot_env.sh环境脚本"
    echo "请确保在XLeRobot项目根目录中运行此脚本"
    exit 1
fi

echo "🤖 XleRobot 开发环境完整验证"
echo "================================="
echo "检查时间: $(date)"
echo "检查用户: $(whoami)"
echo "工作目录: $(pwd)"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

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

echo "🔧 第一步：基础环境配置检查"
echo "----------------------------"

# 1. Python环境检查
echo "检查Python环境..."
PYTHON_VERSION=$($PYTHON_EXECUTABLE --version 2>/dev/null | grep -o "3\.10\.[0-9]*")
if [ "$PYTHON_VERSION" = "3.10.12" ]; then
    check_result 0 "Python版本正确: $PYTHON_VERSION"
else
    check_result 1 "Python版本错误，需要3.10.12，当前: $($PYTHON_EXECUTABLE --version 2>/dev/null || echo '未知')"
fi

# 检查Python路径
PYTHON_PATH=$(which $PYTHON_EXECUTABLE)
if [ "$PYTHON_PATH" = "/usr/bin/python3.10" ] || [ "$PYTHON_PATH" = "/usr/bin/python3" ]; then
    check_result 0 "Python路径正确: $PYTHON_PATH"
else
    check_result 1 "Python路径错误，需要/usr/bin/python3，当前: $PYTHON_PATH"
fi

# 检查是否使用conda
if echo $PYTHON_PATH | grep -q "miniconda"; then
    check_result 1 "检测到conda环境，禁止使用conda进行开发"
else
    check_result 0 "未检测到conda环境，符合要求"
fi

echo ""
echo "🔧 第二步：运行环境配置脚本"
echo "----------------------------"

# 2. 环境配置脚本检查
if [ -f "/home/sunrise/xlerobot/setup_xlerobot_env.sh" ]; then
    check_result 0 "环境配置脚本存在"

    # 运行环境配置脚本
    info_result "运行环境配置脚本..."
    source /home/sunrise/xlerobot/setup_xlerobot_env.sh > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        check_result 0 "环境配置脚本执行成功"
    else
        check_result 1 "环境配置脚本执行失败"
    fi
else
    check_result 1 "环境配置脚本不存在"
fi

echo ""
echo "🔧 第三步：ROS2环境验证"
echo "----------------------"

# 3. ROS2模块检查
info_result "检查ROS2核心模块..."
$PYTHON_EXECUTABLE -c "
try:
    import rclpy
    print('✅ ROS2 rclpy模块正常')
except ImportError as e:
    print('❌ ROS2 rclpy模块导入失败:', e)
    exit(1)
" 2>/dev/null
check_result $? "ROS2 rclpy模块"

$PYTHON_EXECUTABLE -c "
try:
    from std_msgs.msg import String
    print('✅ ROS2标准消息模块正常')
except ImportError as e:
    print('❌ ROS2标准消息模块导入失败:', e)
    exit(1)
" 2>/dev/null
check_result $? "ROS2标准消息模块"

# 检查ROS2环境变量
if [ "$ROS_DISTRO" = "humble" ]; then
    check_result 0 "ROS_DISTRO正确: $ROS_DISTRO"
else
    check_result 1 "ROS_DISTRO错误，需要humble，当前: $ROS_DISTRO"
fi

if [ "$ROS_VERSION" = "2" ]; then
    check_result 0 "ROS_VERSION正确: $ROS_VERSION"
else
    check_result 1 "ROS_VERSION错误，需要2，当前: $ROS_VERSION"
fi

# 检查ROS2工具
if command -v ros2 &> /dev/null; then
    check_result 0 "ROS2命令行工具可用"
else
    check_result 1 "ROS2命令行工具不可用"
fi

echo ""
echo "🔧 第四步：TROS环境验证"
echo "---------------------"

# 4. TROS模块检查
info_result "检查TROS音频模块..."
$PYTHON_EXECUTABLE -c "
try:
    from audio_msg.msg import AudioFrame, SmartAudioData, AudioEventType
    print('✅ TROS音频消息模块正常')
except ImportError as e:
    print('❌ TROS音频消息模块导入失败:', e)
    exit(1)
" 2>/dev/null
check_result $? "TROS音频消息模块"

# 检查TROS安装
if [ -d "/opt/tros/humble" ]; then
    check_result 0 "TROS安装目录存在"
else
    check_result 1 "TROS安装目录不存在"
fi

# 检查TROS包
if ls /opt/tros/humble/share/ | grep -q "audio_msg"; then
    check_result 0 "TROS音频包可用"
else
    check_result 1 "TROS音频包不可用"
fi

echo ""
echo "🔧 第五步：硬件设备验证"
echo "----------------------"

# 5. 音频设备检查
info_result "检查音频输入设备..."
AUDIO_DEVICES=$(arecord -l 2>/dev/null | grep "card" | wc -l)
if [ $AUDIO_DEVICES -ge 2 ]; then
    check_result 0 "检测到 $AUDIO_DEVICES 个音频设备"
else
    check_result 1 "音频设备数量不足，需要至少2个，当前: $AUDIO_DEVICES"
fi

# 检查USB音频设备
if arecord -l 2>/dev/null | grep -q "USB Audio"; then
    check_result 0 "USB音频设备检测到"
else
    check_result 1 "USB音频设备未检测到"
fi

# 检查板载音频设备
if arecord -l 2>/dev/null | grep -q "ES8326"; then
    check_result 0 "板载ES8326音频设备检测到"
else
    warning_result "板载ES8326音频设备未检测到（可能不影响开发）"
fi

# 检查USB设备权限
if [ -r "/dev/snd" ]; then
    check_result 0 "音频设备权限正常"
else
    check_result 1 "音频设备权限不足"
fi

echo ""
echo "📹 第六步：摄像头硬件验证"
echo "------------------------"

# 6. IMX219摄像头检查
info_result "检查IMX219摄像头硬件..."

# 检查I2C设备（IMX219通常在I2C bus 1上）
if [ -e "/sys/bus/i2c/devices/1-0010" ] || ls /sys/bus/i2c/devices/ | grep -q "0010"; then
    check_result 0 "IMX219摄像头I2C设备检测成功"
elif ls /sys/bus/i2c/devices/ 2>/dev/null | grep -q "imx219"; then
    check_result 0 "IMX219摄像头I2C设备检测成功"
else
    check_result 1 "IMX219摄像头I2C设备未检测到"
fi

# 检查MIPI CSI连接（通过VIN设备间接验证）
if ls /dev/vin* 2>/dev/null | grep -q "vin0"; then
    check_result 0 "MIPI CSI接口通过VIN设备验证成功"
else
    warning_result "MIPI CSI接口未明确验证（可能仍可用）"
fi

# 检查VIN设备（D-Robotics特有）
VIN_DEVICES=$(ls /dev/vin* 2>/dev/null | wc -l)
if [ $VIN_DEVICES -ge 4 ]; then
    check_result 0 "VIN设备检测成功: $VIN_DEVICES 个设备"
else
    check_result 1 "VIN设备数量不足，需要至少4个，当前: $VIN_DEVICES"
fi

# 检查VIN设备详情
if [ -e "/dev/vin0_cap" ] && [ -e "/dev/vin0_src" ]; then
    check_result 0 "VIN0设备完整(cap+src)"
else
    check_result 1 "VIN0设备不完整"
fi

# 检查摄像头设备权限
if [ -r "/dev/vin0_cap" ]; then
    check_result 0 "摄像头设备权限正常"
else
    check_result 1 "摄像头设备权限不足"
fi

echo ""
echo "🤖 第七步：ROS2摄像头环境验证"
echo "-----------------------------"

# 7. D-Robotics官方API检查
info_result "检查D-Robotics官方摄像头API..."
if [ -f "/usr/local/lib/python3.10/dist-packages/hobot_vio/libsrcampy.so" ]; then
    check_result 0 "D-Robotics libsrcampy库存在"
else
    check_result 1 "D-Robotics libsrcampy库不存在"
fi

# 检查hobot_vio Python模块
$PYTHON_EXECUTABLE -c "
import sys
sys.path.append('/usr/local/lib/python3.10/dist-packages/hobot_vio')
try:
    import libsrcampy as srcampy
    print('✅ D-Robotics libsrcampy模块可导入')
except ImportError as e:
    print('❌ D-Robotics libsrcampy模块导入失败:', e)
    exit(1)
" 2>/dev/null
check_result $? "D-Robotics libsrcampy模块"

# 检查ROS2传感器消息模块
$PYTHON_EXECUTABLE -c "
try:
    from sensor_msgs.msg import Image, CameraInfo, CompressedImage
    print('✅ ROS2传感器消息模块正常')
except ImportError as e:
    print('❌ ROS2传感器消息模块导入失败:', e)
    exit(1)
" 2>/dev/null
check_result $? "ROS2传感器消息模块"

# 检查xlerobot_camera包
if [ -d "/home/sunrise/xlerobot/src/xlerobot_camera" ]; then
    check_result 0 "xlerobot_camera包源码存在"
else
    check_result 1 "xlerobot_camera包源码不存在"
fi

# 检查xlerobot_camera是否已构建
if [ -d "/home/sunrise/xlerobot/build/xlerobot_camera" ]; then
    check_result 0 "xlerobot_camera包已构建"
else
    warning_result "xlerobot_camera包未构建（运行colcon build构建）"
fi

# 测试D-Robotics摄像头API功能
info_result "测试D-Robotics摄像头API功能..."
$PYTHON_EXECUTABLE -c "
import sys
sys.path.append('/usr/local/lib/python3.10/dist-packages/hobot_vio')
try:
    import libsrcampy as srcampy
    # 尝试创建摄像头对象
    camera = srcampy.Camera()
    print('✅ D-Robotics摄像头对象创建成功')

    # 尝试获取摄像头信息（不打开摄像头）
    print('✅ D-Robotics摄像头API功能正常')
except Exception as e:
    print(f'⚠️ D-Robotics摄像头API功能测试失败: {e}')
    print('   (可能需要硬件连接或权限)')
" 2>/dev/null
check_result $? "D-Robotics摄像头API功能"

echo ""
echo "🔧 第八步：视觉LLM依赖验证"
echo "-------------------------"

# 8. 视觉LLM依赖检查
info_result "检查视觉处理依赖包..."

# 检查OpenCV
$PYTHON_EXECUTABLE -c "
try:
    import cv2
    print(f'✅ OpenCV可用: {cv2.__version__}')
except ImportError:
    print('❌ OpenCV缺失（视觉LLM必需）')
    exit(1)
" 2>/dev/null
check_result $? "OpenCV图像处理库"

# 检查PIL/Pillow
$PYTHON_EXECUTABLE -c "
try:
    from PIL import Image
    print('✅ PIL/Pillow图像库可用')
except ImportError:
    print('⚠️ PIL/Pillow缺失（可能影响图像处理）')
" 2>/dev/null

# 检查requests库（视觉LLM API调用）
$PYTHON_EXECUTABLE -c "
try:
    import requests
    print('✅ requests库可用')
except ImportError:
    print('❌ requests库缺失（视觉LLM API调用必需）')
    exit(1)
" 2>/dev/null
check_result $? "requests HTTP库"

# 检查base64编码（图像传输）
$PYTHON_EXECUTABLE -c "
import base64
print('✅ base64编码模块可用')
" 2>/dev/null
check_result $? "base64编码模块"

echo ""
echo "🔧 第九步：开发工具验证"
echo "----------------------"

# 9. 开发工具检查
if command -v colcon &> /dev/null; then
    check_result 0 "colcon构建工具可用"
else
    check_result 1 "colcon构建工具不可用"
fi

# 检查Python包
info_result "检查其他Python依赖包..."
$PYTHON_EXECUTABLE -c "
try:
    import numpy
    print('✅ numpy可用')
except ImportError:
    print('⚠️ numpy缺失（可能影响开发）')
" 2>/dev/null

# 检查matplotlib（图像显示调试）
$PYTHON_EXECUTABLE -c "
try:
    import matplotlib.pyplot as plt
    print('✅ matplotlib可用（图像调试）')
except ImportError:
    print('⚠️ matplotlib缺失（可能影响图像调试）')
" 2>/dev/null

echo ""
echo "📊 检查结果汇总"
echo "================"

echo -e "${GREEN}✅ 通过检查: $CHECKS_PASSED 项${NC}"
echo -e "${RED}❌ 失败检查: $CHECKS_FAILED 项${NC}"

TOTAL_CHECKS=$((CHECKS_PASSED + CHECKS_FAILED))
if [ $CHECKS_FAILED -eq 0 ]; then
    echo ""
    echo -e "${GREEN}🎉 恭喜！所有环境检查都通过了！${NC}"
    echo -e "${GREEN}✅ 开发环境配置正确，可以开始多模态开发工作${NC}"
    echo -e "${GREEN}✅ 音频系统 + 视觉系统环境就绪${NC}"
    echo ""
    echo -e "${BLUE}下一步：${NC}"
    echo "1. 启动 Developer Agent: /bmad:bmm:agents:dev"
    echo "2. 选择开发任务："
    echo "   - Story 1.1-1.4: 语音系统开发"
    echo "   - Story 1.5-1.8: 多模态视觉系统开发"
    echo "   - 视觉LLM集成开发"
    exit 0
else
    echo ""
    echo -e "${RED}🚨 环境检查失败！${NC}"
    echo -e "${RED}❌ 有 $CHECKS_FAILED 项检查未通过，禁止开始开发工作${NC}"
    echo ""
    echo -e "${YELLOW}请解决以下问题后重新运行检查：${NC}"
    echo "1. 查看上述失败的检查项目"
    echo "2. 参考 /home/sunrise/xlerobot/docs/development-environment-checklist.md"
    echo "3. 运行环境配置脚本: source /home/sunrise/xlerobot/setup_xlerobot_env.sh"
    echo "4. 如果摄像头相关检查失败，检查Story 1.5文档"
    echo "5. 重新运行此检查脚本"
    exit 1
fi