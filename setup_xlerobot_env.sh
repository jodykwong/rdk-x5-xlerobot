#!/bin/bash
# XleRobot 开发环境配置脚本 (已废弃)
# ⚠️ 注意：此脚本已被 xlerobot_env.sh 替代
# 严格按照 Brownfield Level 4 要求配置
# 项目: xlerobot - 家用机器人控制系统

echo "🤖 XleRobot 开发环境配置 (已废弃)"
echo "================================="
echo "⚠️ 此脚本已被弃用，请使用新版本："
echo "   source ./xlerobot_env.sh"
echo ""

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

echo "✅ Python环境: $(/usr/bin/python3 --version)"
echo "✅ ROS2版本: $ROS_DISTRO"
echo "✅ TROS状态: 已加载"
echo "✅ 音频设备: $(arecord -l 2>/dev/null | grep 'card 0' | wc -l)个设备"

# 验证关键模块
/usr/bin/python3 -c "
try:
    import rclpy
    from audio_msg.msg import AudioFrame
    from std_msgs.msg import String
    print('✅ 所有核心模块加载成功')
except ImportError as e:
    print(f'❌ 模块加载失败: {e}')
    exit(1)
"

echo "🎯 开发环境配置完成！"
echo "使用方法: source setup_xlerobot_env.sh"