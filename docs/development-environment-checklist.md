# XleRobot 开发环境检查清单
## Brownfield Level 4 强制要求

**文档编号**: XLR-ENV-20251108-001
**项目名称**: XleRobot 家用机器人控制系统
**适用范围**: 所有 Developer Agent 启动前必须检查
**最后更新**: 2025-11-08
**版本**: 1.0

---

## 🚨 **强制执行声明**

**本文档包含 Brownfield Level 4 企业级开发的强制环境要求。每次启动 Developer Agent 进行任何开发工作之前，必须完成以下所有检查项目。任何检查失败都禁止开始开发工作。**

---

## 📋 **环境配置检查清单**

### ✅ **第一阶段：基础环境配置**

#### 1.1 Python环境验证
```bash
# 检查Python版本 (必须是系统Python 3.10.12)
python3 --version
# 预期输出: Python 3.10.12

# 检查Python路径 (必须是/usr/bin/python3)
which python3
# 预期输出: /usr/bin/python3

# 禁止使用Python 3.13或conda环境
python3 --version | grep -v "3.13" || echo "❌ 错误：使用了错误的Python版本"
```

**✅ 检查标准:**
- [ ] Python版本必须是 3.10.12
- [ ] Python路径必须是 /usr/bin/python3
- [ ] 禁止使用 conda 或 Python 3.13

#### 1.2 运行环境配置脚本
```bash
# 运行官方环境配置脚本
source /home/sunrise/xlerobot/setup_xlerobot_env.sh
# 预期输出: 🎯 开发环境配置完成！所有核心模块加载成功
```

**✅ 检查标准:**
- [ ] 脚本执行无错误
- [ ] 显示"所有核心模块加载成功"
- [ ] 环境变量正确设置

### ✅ **第二阶段：ROS2环境验证**

#### 2.1 ROS2核心模块测试
```bash
# 测试rclpy模块
python3 -c "import rclpy; print('✅ ROS2 rclpy模块正常')"

# 测试标准消息模块
python3 -c "from std_msgs.msg import String; print('✅ ROS2标准消息模块正常')"

# 检查ROS2环境变量
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
# 预期: ROS_DISTRO=humble, ROS_VERSION=2
```

**✅ 检查标准:**
- [ ] rclpy模块导入成功
- [ ] std_msgs模块导入成功
- [ ] ROS_DISTRO设置为humble
- [ ] ROS_VERSION设置为2

#### 2.2 ROS2工具验证
```bash
# 检查ROS2命令行工具
which ros2
# 预期输出: /opt/ros/humble/bin/ros2

# 检查ros2功能
timeout 3 ros2 topic list 2>/dev/null || echo "ROS2 daemon需要启动"
```

**✅ 检查标准:**
- [ ] ros2命令可用
- [ ] ROS2基础功能正常

### ✅ **第三阶段：TROS环境验证**

#### 3.1 TROS音频模块测试
```bash
# 测试TROS音频消息模块
python3 -c "
from audio_msg.msg import AudioFrame, SmartAudioData, AudioEventType
print('✅ TROS音频消息模块正常')
"

# 检查可用的音频消息类型
python3 -c "
import audio_msg.msg
print('可用音频消息类型:', [x for x in dir(audio_msg.msg) if not x.startswith('_')])
"
```

**✅ 检查标准:**
- [ ] AudioFrame消息导入成功
- [ ] SmartAudioData消息导入成功
- [ ] AudioEventType消息导入成功

#### 3.2 TROS安装验证
```bash
# 检查TROS安装目录
ls /opt/tros/humble/ | head -5
# 预期: bin, include, lib, local, share等目录

# 检查TROS包
ls /opt/tros/humble/share/ | grep -E "(audio|ai_msgs)" | head -3
```

**✅ 检查标准:**
- [ ] TROS安装目录存在
- [ ] 音频相关包可用
- [ ] AI消息包可用

### ✅ **第四阶段：硬件设备验证**

#### 4.1 音频输入设备检查
```bash
# 检查音频捕获设备
arecord -l | grep "card 0"
# 预期输出: USB Audio Device相关信息

# 检查板载音频设备
arecord -l | grep "ES8326"
# 预期输出: ES8326 HiFi相关信息

# 统计音频设备数量
arecord -l | grep "card" | wc -l
# 预期: 至少2个音频设备
```

**✅ 检查标准:**
- [ ] USB音频设备检测到
- [ ] 板载ES8326音频设备检测到
- [ ] 音频设备数量≥2个

#### 4.2 USB设备验证
```bash
# 检查USB音频设备
lsusb | grep -i audio
# 预期: C-Media Electronics, Inc. USB Audio Device

# 检查USB设备权限
ls -la /dev/snd/ | grep "card"
# 预期: 当前用户有访问权限
```

**✅ 检查标准:**
- [ ] USB音频设备已识别
- [ ] 音频设备权限正确

### ✅ **第五阶段：开发工具验证**

#### 5.1 构建工具检查
```bash
# 检查colcon构建工具
which colcon
# 预期输出: /usr/bin/colcon

# 检查colcon版本
colcon --version
# 预期: 显示版本信息
```

**✅ 检查标准:**
- [ ] colcon工具可用
- [ ] colcon版本信息正常

#### 5.2 开发依赖检查
```bash
# 检查常用Python包
python3 -c "
try:
    import numpy
    print('✅ numpy可用')
except ImportError:
    print('❌ numpy缺失')

try:
    import librosa
    print('✅ librosa可用')
except ImportError:
    print('⚠️ librosa缺失（可选）')
"
```

**✅ 检查标准:**
- [ ] numpy包可用
- [ ] 核心依赖包正常

---

## 🔍 **综合验证测试**

### 完整环境测试脚本
```bash
#!/bin/bash
# 完整的环境验证脚本
# 使用方法: bash /home/sunrise/xlerobot/docs/complete_env_check.sh

echo "🤖 XleRobot 开发环境完整验证"
echo "================================="

# 设置环境
source /home/sunrise/xlerobot/setup_xlerobot_env.sh

# 综合测试
python3 -c "
import sys
print(f'Python版本: {sys.version}')

try:
    import rclpy
    from audio_msg.msg import AudioFrame
    from std_msgs.msg import String
    print('✅ 所有核心模块导入成功')
except ImportError as e:
    print(f'❌ 模块导入失败: {e}')
    sys.exit(1)

print('✅ 开发环境验证通过')
"

echo "🎯 环境验证完成"
```

---

## 📊 **检查结果记录表**

| 检查项目 | 状态 | 检查时间 | 执行人 | 备注 |
|---------|------|----------|--------|------|
| Python 3.10.12环境 | | | | |
| 环境配置脚本 | | | | |
| ROS2 rclpy模块 | | | | |
| TROS音频模块 | | | | |
| USB音频设备 | | | | |
| 板载音频设备 | | | | |
| colcon构建工具 | | | | |
| 综合环境测试 | | | | |

**✅ 签名确认:**
- 开发环境检查完成: _________________ 日期: _________
- 环境配置验证通过: _________________ 日期: _________
- 准许开始开发工作: _________________ 日期: _________

---

## 🚨 **故障排除指南**

### 常见问题及解决方案

1. **Python版本错误**
   - 问题: 显示Python 3.13
   - 解决: `export PATH="/usr/bin:$PATH"`

2. **rclpy模块导入失败**
   - 问题: ModuleNotFoundError
   - 解决: 重新运行 `source /opt/ros/humble/setup.bash`

3. **音频设备检测不到**
   - 问题: arecord -l 无输出
   - 解决: 检查USB连接，重新插拔设备

4. **TROS模块导入失败**
   - 问题: No module named 'audio_msg'
   - 解决: 重新运行 `source /opt/tros/humble/setup.bash`

5. **权限问题**
   - 问题: 设备访问被拒绝
   - 解决: `sudo usermod -a -G audio $USER` 并重新登录

---

## 📞 **支持和联系**

**环境配置支持:**
- 环境配置脚本: `/home/sunrise/xlerobot/setup_xlerobot_env.sh`
- 完整检查脚本: `/home/sunrise/xlerobot/docs/complete_env_check.sh`
- 项目状态文件: `/home/sunrise/xlerobot/docs/bmm-workflow-status.md`

**紧急联系:**
- 技术负责人: System Architect
- 环境配置专家: DevOps Engineer

---

**文档维护:**
- 创建者: Claude (BMad Environment Configuration Agent)
- 审核状态: 待审核
- 下次更新: 根据环境变化及时更新
- 版本历史: v1.0 (2025-11-08)

---

*本文档是 Brownfield Level 4 企业级开发标准的重要组成部分，违反环境配置要求可能导致开发失败或系统不稳定。严格按照本清单执行是确保项目成功的关键保障。*