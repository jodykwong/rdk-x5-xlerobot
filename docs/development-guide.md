# XleRobot 开发指南

**文档编号**: XLR-DEV-P0-20251107-001
**项目名称**: XleRobot 家用机器人控制系统
**文档类型**: 开发指南文档
**生成日期**: 2025-11-07
**工作流**: Phase 0 Documentation - document-project

---

## 📋 概述

本开发指南为XleRobot项目提供完整的开发环境配置、构建流程、测试方法、调试技巧等开发相关指导。严格遵循Brownfield Level 4企业级标准，确保开发过程的一致性和代码质量。

### 目标读者
- 新加入项目的开发人员
- 参与项目重构的技术人员
- AI辅助开发系统
- 项目维护和运维人员

---

## 🔧 开发环境配置

### 🚨 重要环境要求
**必须严格按照以下配置，否则系统无法正常工作**

#### 1. 硬件环境要求
```yaml
硬件平台: D-Robotics RDK X5 V1.0
CPU: ARM Cortex-A55, 8核
内存: 8GB RAM (实际可用7GB)
存储: 128GB SSD
音频: USB麦克风 + 扬声器
视觉: IMX219摄像头 (可选)
网络: 千兆以太网 + WiFi
```

#### 2. 操作系统要求
```bash
# 确认系统版本
lsb_release -a
# 输出应该显示: Ubuntu 22.04.x LTS

# 确认内核版本
uname -r
# 输出应该显示: 6.x.x-generic (支持ARM64)
```

#### 3. ROS2环境配置 (强制要求)
```bash
# 1. 安装ROS2 Humble (如果尚未安装)
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete

# 2. 设置ROS2环境 (每次开发前必须执行)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 3. 验证ROS2安装
ros2 --version
# 应该输出: ros2 2.x.x

# 4. 测试ROS2节点
ros2 run demo_nodes_cpp talker
# 新终端测试:
ros2 run demo_nodes_py listener
```

#### 4. TROS环境配置 (强制要求)
```bash
# 1. 安装TROS 2.4.3 (地平线机器人开发套件)
# 下载TROS安装包并按照官方文档安装

# 2. 设置TROS环境 (每次开发前必须执行)
source /opt/tros/humble/setup.bash
echo "source /opt/tros/humble/setup.bash" >> ~/.bashrc

# 3. 验证TROS安装
# 检查TROS算法包数量
ls /opt/tros/humble/lib/ | wc -l
# 应该显示: 66 (66个算法包)

# 4. 测试TROS音频功能
python3 -c "import hobot_audio; print('TROS Audio loaded successfully')"
```

#### 5. Python环境配置 (重要)
```bash
# ⚠️ 关键要求: 必须使用系统Python 3.10
# 禁止使用Python 3.13或conda环境

# 1. 确认Python版本
/usr/bin/python3 --version
# 应该输出: Python 3.10.x

# 2. 验证Python路径
which python3
# 应该输出: /usr/bin/python3

# 3. 安装Python依赖
/usr/bin/python3 -m pip install -r requirements.txt

# 4. 验证关键包
/usr/bin/python3 -c "import rclpy; print('ROS2 Python OK')"
/usr/bin/python3 -c "import numpy; print('NumPy OK')"
/usr/bin/python3 -c "import requests; print('Requests OK')"
```

### 📦 项目依赖安装

#### 1. 系统依赖
```bash
# 安装系统包
sudo apt update
sudo apt install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    portaudio19-dev \
    python3-pyaudio \
    ffmpeg \
    libasound2-dev \
    libsndfile1-dev

# 安装音频处理库
sudo apt install -y \
    libportaudio2 \
    libportaudiocpp0 \
    portaudio19-dev
```

#### 2. Python依赖
```bash
# 使用系统Python 3.10安装
/usr/bin/python3 -m pip install --upgrade pip

# 安装核心依赖
/usr/bin/python3 -m pip install \
    rclpy>=3.3.0 \
    sensor-msgs>=4.2.0 \
    audio-common-msgs>=2.4.0 \
    numpy>=1.21.0 \
    requests>=2.28.0 \
    websockets>=10.4 \
    pyaudio>=0.2.11 \
    pygame>=2.1.0 \
    librosa>=0.9.2 \
    soundfile>=0.10.3

# 安装开发和测试工具
/usr/bin/python3 -m pip install \
    pytest>=7.0.0 \
    pytest-cov>=4.0.0 \
    black>=22.0.0 \
    flake8>=5.0.0 \
    mypy>=0.991 \
    pre-commit>=2.20.0
```

### 🔄 环境激活脚本

创建每日开发环境激活脚本：

```bash
#!/bin/bash
# dev_env.sh - 开发环境激活脚本

echo "🤖 XleRobot 开发环境激活中..."

# 激活ROS2环境
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ ROS2 Humble 环境已激活"
else
    echo "❌ ROS2 Humble 未安装"
    exit 1
fi

# 激活TROS环境
if [ -f "/opt/tros/humble/setup.bash" ]; then
    source /opt/tros/humble/setup.bash
    echo "✅ TROS 2.4.3 环境已激活"
else
    echo "❌ TROS 未安装"
    exit 1
fi

# 验证Python版本
PYTHON_VERSION=$(/usr/bin/python3 --version 2>&1 | grep -o '3\.10')
if [ -n "$PYTHON_VERSION" ]; then
    echo "✅ Python 3.10 环境正确"
else
    echo "❌ Python版本错误，必须使用Python 3.10"
    exit 1
fi

# 设置项目环境变量
export PYTHONPATH="/home/sunrise/xlerobot/src:$PYTHONPATH"
export ROS_DOMAIN_ID=42
export RCUTILS_LOGGING_SEVERITY=INFO

echo "🚀 开发环境激活完成!"
echo "当前工作目录: $(pwd)"
echo "Python路径: $(which python3)"
echo "ROS2版本: $(ros2 --version)"
```

---

## 🏗️ 构建流程

### 1. 项目结构验证
```bash
# 验证项目结构完整性
python3 scripts/verify_structure.py
```

### 2. 清理构建环境
```bash
# 清理之前的构建
./scripts/clean.sh

# 手动清理 (如果脚本失败)
rm -rf build/ install/ log/
```

### 3. 构建ROS2工作空间
```bash
# 确保环境激活
source /opt/ros/humble/setup.bash
source /opt/tros/humble/setup.bash

# 构建项目
colcon build --symlink-install --parallel-workers 4

# 验证构建结果
source install/setup.bash
ros2 pkg list | grep xlerobot
```

### 4. Python模块构建
```bash
# 构建Python模块
/usr/bin/python3 setup.py build

# 安装开发模式
/usr/bin/python3 -m pip install -e .

# 验证安装
/usr/bin/python3 -c "import xlerobot_llm; print('XleRobot模块安装成功')"
```

### 5. 自动化构建脚本
```bash
#!/bin/bash
# build.sh - 项目构建脚本

set -e  # 遇到错误立即退出

echo "🔨 开始构建XleRobot项目..."

# 验证环境
./scripts/verify_env.sh

# 清理旧构建
echo "🧹 清理构建环境..."
rm -rf build/ install/ log/

# 构建ROS2包
echo "📦 构建ROS2包..."
colcon build \
    --symlink-install \
    --parallel-workers $(nproc) \
    --event-handlers console_direct+

# 构建Python模块
echo "🐍 构建Python模块..."
/usr/bin/python3 setup.py build
/usr/bin/python3 -m pip install -e .

# 运行基础测试
echo "🧪 运行基础测试..."
colcon test --packages-select xlerobot_llm

echo "✅ 构建完成!"
```

---

## 🧪 测试流程

### 1. 测试环境准备
```bash
# 激活测试环境
source install/setup.bash
export RCUTILS_LOGGING_SEVERITY=DEBUG

# 检查测试依赖
/usr/bin/python3 -m pytest --version
```

### 2. 单元测试
```bash
# 运行所有单元测试
/usr/bin/python3 -m pytest tests/unit/ -v

# 运行特定模块测试
/usr/bin/python3 -m pytest tests/unit/test_asr/ -v

# 生成覆盖率报告
/usr/bin/python3 -m pytest tests/unit/ --cov=src --cov-report=html
```

### 3. 集成测试
```bash
# 运行集成测试
/usr/bin/python3 -m pytest tests/integration/ -v

# 运行语音管道集成测试
/usr/bin/python3 -m pytest tests/integration/test_voice_pipeline.py -v -s
```

### 4. 硬件测试 (真实硬件)
```bash
# ⚠️ 硬件测试需要真实硬件环境，严禁Mock数据

# 测试音频硬件
/usr/bin/python3 tests/hardware/test_audio_hardware.py

# 测试摄像头硬件 (如果可用)
/usr/bin/python3 tests/hardware/test_camera_hardware.py

# 完整硬件测试套件
./scripts/test_hardware.sh
```

### 5. 性能测试
```bash
# ASR性能测试
/usr/bin/python3 tests/performance/test_asr_performance.py

# TTS延迟测试
/usr/bin/python3 tests/performance/test_tts_latency.py

# 系统负载测试
/usr/bin/python3 tests/performance/test_system_load.py
```

### 6. 测试自动化脚本
```bash
#!/bin/bash
# test.sh - 完整测试套件

echo "🧪 开始运行XleRobot测试套件..."

# 代码质量检查
echo "📊 代码质量检查..."
black --check src/
flake8 src/
mypy src/

# 单元测试
echo "🔬 单元测试..."
/usr/bin/python3 -m pytest tests/unit/ --cov=src --cov-fail-under=80

# 集成测试
echo "🔗 集成测试..."
/usr/bin/python3 -m pytest tests/integration/ -v

# 性能测试
echo "⚡ 性能测试..."
/usr/bin/python3 tests/performance/test_asr_performance.py

# 硬件测试 (如果硬件可用)
if [ -e "/dev/dri/card0" ]; then
    echo "🎯 硬件测试..."
    ./scripts/test_hardware.sh
fi

echo "✅ 测试完成!"
```

---

## 🐛 调试技巧

### 1. 日志配置
```yaml
# config/logging.yaml
loggers:
  xlerobot:
    level: DEBUG
    handlers: [console, file]
  xlerobot.asr:
    level: DEBUG
    handlers: [console, file]
  xlerobot.llm:
    level: INFO
    handlers: [console, file]
  xlerobot.tts:
    level: DEBUG
    handlers: [console, file]

handlers:
  console:
    class: logging.StreamHandler
    level: INFO
    formatter: standard
  file:
    class: logging.FileHandler
    level: DEBUG
    filename: /tmp/xlerobot.log
    formatter: detailed
```

### 2. ROS2调试工具
```bash
# 查看ROS2节点
ros2 node list

# 查看话题
ros2 topic list

# 监听话题
ros2 topic echo /audio_input

# 查看节点信息
ros2 node info /xlerobot_asr_node

# 查看参数
ros2 param list /xlerobot_llm_node
ros2 param get /xlerobot_llm_node model_name
```

### 3. 性能调试
```bash
# 系统资源监控
top -p $(pgrep -f "xlerobot")

# 内存使用情况
ps aux | grep xlerobot

# 网络连接状态
netstat -an | grep :8080

# 音频设备状态
arecord -l
aplay -l
```

### 4. Python调试
```python
# 使用pdb调试器
import pdb; pdb.set_trace()

# 使用ROS2日志
import rclpy
from rclpy.logging import get_logger

logger = get_logger('xlerobot_debug')
logger.info('调试信息')
logger.error('错误信息')
```

---

## 📝 代码规范

### 1. Python代码规范
```python
# 使用Black格式化
black src/ tests/

# 使用flake8检查
flake8 src/ tests/ --max-line-length=100

# 使用mypy类型检查
mypy src/ --ignore-missing-imports
```

### 2. ROS2代码规范
```bash
# 使用ament_lint
colcon test --packages-select xlerobot_llm --lint-only

# 检查package.xml格式
xmllint --schema schema/package.xsd package.xml
```

### 3. Git提交规范
```bash
# 提交消息格式
<type>(<scope>): <description>

[optional body]

[optional footer]

# 示例
feat(asr): 添加唤醒词检测功能
fix(tts): 修复音频播放延迟问题
docs(readme): 更新安装说明
```

---

## 🔄 CI/CD流程

### 1. 持续集成配置
```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
    - uses: actions/checkout@v3
    - name: Setup ROS2
      run: |
        sudo apt update
        sudo apt install ros-humble-desktop python3-pip
        source /opt/ros/humble/setup.bash
    - name: Install dependencies
      run: |
        /usr/bin/python3 -m pip install -r requirements.txt
    - name: Run tests
      run: |
        source /opt/ros/humble/setup.bash
        /usr/bin/python3 -m pytest
```

### 2. 代码质量检查
```bash
# Pre-commit配置
pre-commit install

# 手动运行pre-commit
pre-commit run --all-files
```

---

## 📚 开发资源

### 1. 文档资源
- [项目主索引](./index.md)
- [源码树分析](./source-tree-analysis.md)
- [架构分析](./architecture-analysis.md)
- [API契约](./api-contracts.md)
- [组件清单](./component-inventory.md)

### 2. 外部资源
- [ROS2官方文档](https://docs.ros.org/en/humble/)
- [TROS开发指南](https://developer.horizon.ai/)
- [Python开发指南](https://docs.python.org/3.10/)

### 3. 开发工具推荐
- **IDE**: VS Code + Python + ROS2扩展
- **调试**: VS Code调试器 + ROS2调试工具
- **版本控制**: Git + GitHub
- **文档**: Markdown + 静态站点生成器

---

## ❓ 常见问题

### 1. 环境问题
**Q: Python版本错误怎么办？**
A: 必须使用系统Python 3.10，不要使用conda或其他Python版本。

**Q: ROS2命令找不到？**
A: 确保执行了 `source /opt/ros/humble/setup.bash`

**Q: TROS库导入失败？**
A: 确保执行了 `source /opt/tros/humble/setup.bash`

### 2. 构建问题
**Q: 构建失败怎么办？**
A: 检查环境配置，运行 `./scripts/clean.sh` 清理后重新构建

**Q: 依赖安装失败？**
A: 使用系统Python 3.10，检查网络连接，更新pip

### 3. 运行时问题
**Q: 音频设备无法访问？**
A: 检查音频设备权限，确保用户在audio组中

**Q: ROS2节点启动失败？**
A: 检查ROS2环境，查看日志文件 `/tmp/xlerobot.log`

---

## 🚨 重要提醒

### 开发环境强制要求
1. **必须使用ROS2 Humble环境**
2. **必须使用系统Python 3.10**
3. **必须激活TROS 2.4.3环境**
4. **禁止环境混用**

### Brownfield Level 4要求
1. **保持向后兼容性**
2. **完整的测试覆盖**
3. **详细的文档记录**
4. **严格的代码规范**

### 硬件测试要求
1. **严禁使用Mock数据**
2. **必须在真实硬件上测试**
3. **性能基准必须达标**
4. **错误处理必须完整**

---

*本开发指南遵循Brownfield Level 4企业级标准，为XleRobot项目提供完整的开发环境指导。如有疑问，请参考相关文档或联系项目维护团队。*