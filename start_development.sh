#!/bin/bash
# XleRobot 开发启动脚本
# 在新shell中启动开发环境，确保Python环境正确

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

echo "🚀 启动XleRobot开发环境"
echo "========================="

# 启动新的bash shell，确保Python环境正确
exec bash --rcfile <(echo "
# 1. 立即修复Python环境
export PATH=\$(echo \$PATH | tr ':' '\n' | grep -v miniconda | tr '\n' ':' | sed 's/:\$//')
export PATH=\"/usr/bin:/usr/local/bin:/opt/ros/humble/bin:/usr/hobot/bin:/home/sunrise/.local/bin:/snap/bin\"

# 2. 强制python3指向系统版本
alias python3='$PYTHON_EXECUTABLE'
function python3() { $PYTHON_EXECUTABLE \"\$@\"; }

# 3. 加载ROS2/TROS环境
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

# 4. 设置环境变量
export ROS_DISTRO=humble
export ROS_VERSION=2
export PYTHONPATH=\"/opt/hobot/lib/python3.10/site-packages:/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:\$PYTHONPATH\"

# 5. 显示环境状态
echo ''
echo '🤖 XleRobot开发环境已准备就绪'
echo '============================='
echo \"✅ Python版本: \$($PYTHON_EXECUTABLE --version)\"
echo \"✅ Python路径: \$(which python3 2>/dev/null || echo '已强制使用系统Python')\"
echo \"✅ ROS2版本: \$ROS_DISTRO\"
echo \"✅ TROS状态: 已加载\"
echo \"✅ 音频设备: \$(arecord -l 2>/dev/null | grep 'card 0' | wc -l)个设备\"

# 6. 验证关键模块
echo '🧪 验证关键模块...'
$PYTHON_EXECUTABLE -c \"
try:
    import rclpy
    from audio_msg.msg import AudioFrame
    from std_msgs.msg import String
    print('✅ 所有关键模块导入成功')
except ImportError as e:
    print(f'❌ 模块导入失败: {e}')
\"

echo ''
echo '🎯 开发环境就绪！'
echo '💡 可用的命令:'
echo '   - python3 --version  (查看Python版本)'
echo '   - which python3      (查看Python路径)'
echo '   - arecord -l         (查看音频设备)'
echo '   - ros2 topic list    (ROS2主题列表)'
echo '   - bash complete_env_check.sh  (运行完整环境检查)'
echo ''
echo '🚀 现在可以开始Story 1.1开发了！'
echo '   使用: /bmad:bmm:agents:dev 启动Developer Agent'
")