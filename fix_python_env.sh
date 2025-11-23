#!/bin/bash
# Python环境修复脚本 (已废弃)
# ⚠️ 注意：此脚本已被 xlerobot_env.sh 替代

echo "🔧 Python环境修复 (已废弃)"
echo "============================"
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

echo "🔍 验证修复结果..."
echo "Python路径: $(which python3)"
echo "Python版本: $(python3 --version 2>/dev/null || echo 'python3命令不可用')"
echo "✅ 系统Python验证:"
/usr/bin/python3 --version

# 6. 测试关键模块
echo "🧪 测试关键模块..."
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

/usr/bin/python3 -c "
try:
    import rclpy
    from audio_msg.msg import AudioFrame
    from std_msgs.msg import String
    print('✅ 所有关键模块导入成功')
except ImportError as e:
    print(f'❌ 模块导入失败: {e}')
    exit(1)
"

echo "✅ Python环境修复完成！"
echo "💡 提示: 在新shell中运行此脚本，或运行: source fix_python_env.sh"