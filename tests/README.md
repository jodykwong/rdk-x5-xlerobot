# XLeRobot 测试套件文档

## 📋 概述

XLeRobot测试套件采用分层架构设计，确保系统的可靠性、稳定性和性能。测试覆盖从单元级别到端到端的完整功能验证，支持自动化测试执行和详细的报告生成。

## 🗂️ 测试分类

### 🧪 集成测试 (Integration Tests)

**目的**: 验证ASR→LLM→TTS完整语音交互流程

| 测试文件 | 功能描述 | 重要性 |
|---------|---------|-------|
| `test_epic1_complete_integration.py` | Epic 1完整功能集成测试 | ⭐⭐⭐⭐⭐ |
| `verify_epic1_complete_functionality.py` | Epic 1功能验证 | ⭐⭐⭐⭐⭐ |
| `real_epic1_verification.py` | 真实环境验证 | ⭐⭐⭐⭐ |

**运行命令**:
```bash
# 运行所有集成测试
python3.10 tests/run_all_tests.py --category integration

# 运行特定集成测试
python3.10 tests/test_epic1_complete_integration.py
```

### 🔬 单元测试 (Unit Tests)

**目的**: 验证各个模块的独立功能

| 测试文件 | 功能描述 | 重要性 |
|---------|---------|-------|
| `test_aliyun_api_integration.py` | 阿里云API集成测试 | ⭐⭐⭐⭐ |
| `test_audio_components.py` | 音频组件功能测试 | ⭐⭐⭐ |
| `test_e2e_integration.py` | 端到端集成测试 | ⭐⭐⭐ |

**运行命令**:
```bash
# 运行所有单元测试
python3.10 tests/run_all_tests.py --category unit

# 运行API测试
python3.10 tests/run_all_tests.py --category api
```

### 🚀 管道测试 (Pipeline Tests)

**目的**: 验证语音处理管道和流程

| 测试文件 | 功能描述 | 重要性 |
|---------|---------|-------|
| `test_complete_pipeline.py` | 完整语音处理管道 | ⭐⭐⭐ |
| `real_pipeline_test.py` | 真实环境管道测试 | ⭐⭐⭐ |
| `fixed_real_pipeline_test.py` | 修复后管道测试 | ⭐⭐ |
| `test_audio_pipeline.py` | 音频管道专项测试 | ⭐⭐ |
| `run_voice_assistant_test.py` | 语音助手运行测试 | ⭐⭐ |

### 🛠️ 工具测试 (Tool Tests)

**目的**: 验证辅助脚本和工具

| 测试文件 | 功能描述 | 重要性 |
|---------|---------|-------|
| `camera_init.py` | 摄像头初始化测试 | ⭐⭐ |
| `quick_verification.py` | 快速验证脚本 | ⭐⭐⭐ |
| `simple_epic1_check.py` | Epic 1简单检查 | ⭐⭐ |
| `story1_2_detailed_analysis.py` | Story 1.2详细分析 | ⭐⭐ |

### 🔧 专项测试 (Specialized Tests)

**目的**: 验证特殊功能和修复

| 测试文件 | 功能描述 | 重要性 |
|---------|---------|-------|
| `test_dynamic_messages.py` | 动态消息测试 | ⭐⭐ |
| `test_correct_audio_fix.py` | 音频修复验证 | ⭐⭐ |
| `test_ros2_nodes.py` | ROS2节点测试 | ⭐⭐⭐ |

## 🏃‍♂️ 运行测试

### 使用测试套件管理器（推荐）

```bash
# 运行所有测试
python3.10 tests/run_all_tests.py

# 按类别运行
python3.10 tests/run_all_tests.py --category integration
python3.10 tests/run_all_tests.py --category unit
python3.10 tests/run_all_tests.py --category pipeline

# 运行特定测试文件
python3.10 tests/run_all_tests.py --tests test_epic1_complete_integration.py

# 生成JSON格式报告
python3.10 tests/run_all_tests.py --output json

# 保存报告到文件
python3.10 tests/run_all_tests.py --report-file test_report.txt

# 列出所有可用测试
python3.10 tests/run_all_tests.py --list

# 设置超时时间
python3.10 tests/run_all_tests.py --timeout 600
```

### 手动运行单个测试

```bash
# 核心集成测试
python3.10 tests/test_epic1_complete_integration.py
python3.10 tests/verify_epic1_complete_functionality.py
python3.10 tests/real_epic1_verification.py

# API和组件测试
python3.10 tests/test_aliyun_api_integration.py
python3.10 tests/test_audio_components.py

# 快速验证
python3.10 tests/quick_verification.py
python3.10 tests/simple_epic1_check.py
```

## 📊 测试报告

### 报告格式

测试套件管理器支持多种报告格式：

1. **控制台报告** (默认) - 实时显示，易读性强
2. **JSON报告** - 机器可读，便于CI/CD集成

### 报告内容

每个测试报告包含：

- **测试统计**: 总数、通过、失败、错误、成功率
- **执行时间**: 单个测试和总执行时间
- **分类结果**: 按测试类别分组统计
- **详细信息**: 每个测试的状态、错误信息、输出
- **系统状态**: 整体健康评估

### 示例报告输出

```
======================================================================
🧪 XLeRobot 测试报告
======================================================================
📊 测试统计:
   总计: 15 个测试
   通过: 13 ✅
   失败: 1 ❌
   错误: 1 💥
   成功率: 86.7%
   总耗时: 45.23秒

📋 分类结果:
   integration: 3/3 通过
   unit: 3/3 通过
   pipeline: 4/5 通过
   tools: 2/2 通过
   specialized: 1/2 通过

🔍 详细结果:
----------------------------------------------------------------------
✅ test_epic1_complete_integration.py     ( 12.45s) [integration]
✅ verify_epic1_complete_functionality.py (  8.23s) [integration]
❌ test_audio_pipeline.py                 (  5.67s) [pipeline]
   💌 AudioDeviceError: 无法访问音频设备
...
```

## 🔧 测试环境配置

### 必需环境

- **Python**: 3.10.12 (系统版本，不是Miniconda)
- **ROS2**: Humble
- **环境变量**: 所有必需的API密钥和配置

### 环境检查

运行测试前请确保环境配置正确：

```bash
# 检查环境
./start_voice_assistant.sh check

# 验证API连接
bash scripts/test_aliyun_connection.sh

# 测试音频设备
arecord -d 3 test.wav && aplay test.wav
```

### 测试数据

测试音频样本位于 `testing_data/audio_samples/` 目录：

- `cantonese/` - 粤语语音样本 (8个文件)
- `jiajia/` - 家佳语音样本 (5个文件)
- `voice_test/` - 语音测试样本 (4个文件)
- `misc/` - 其他测试样本

## 🚨 故障排查

### 常见测试失败原因

#### 1. API连接失败
```bash
❌ test_aliyun_api_integration.py (2.34s) [api]
   💌 ConnectionError: 无法连接到阿里云服务
```

**解决方案**:
- 检查网络连接
- 验证API密钥配置
- 运行 `bash scripts/test_aliyun_connection.sh`

#### 2. 音频设备问题
```bash
❌ test_audio_components.py (1.23s) [unit]
   💌 AudioDeviceError: 无法访问录音设备
```

**解决方案**:
- 检查音频设备权限: `groups $USER | grep audio`
- 验证设备: `arecord -l` 和 `aplay -l`
- 测试音频功能: `arecord -d 3 test.wav && aplay test.wav`

#### 3. 环境变量缺失
```bash
❌ test_epic1_complete_integration.py (0.45s) [integration]
   💌 EnvironmentError: 缺少ALIBABA_CLOUD_ACCESS_KEY_ID
```

**解决方案**:
- 复制 `.env.example` 为 `.env`
- 填入真实的API密钥
- 重新加载环境: `source .env`

#### 4. Python版本错误
```bash
❌ test_epic1_complete_integration.py (0.12s) [integration]
   💌 SyntaxError: 不支持的Python版本
```

**解决方案**:
- 确保使用Python 3.10: `python3.10 --version`
- 避免使用Miniconda环境
- 重新激活ROS2环境

### 调试技巧

#### 1. 详细错误信息
```bash
# 运行单个测试查看详细错误
python3.10 tests/test_epic1_complete_integration.py
```

#### 2. 调试模式
```bash
# 启用详细日志
export LOG_LEVEL=DEBUG
python3.10 tests/run_all_tests.py --category unit
```

#### 3. 超时设置
```bash
# 增加超时时间用于慢速测试
python3.10 tests/run_all_tests.py --timeout 600
```

## 📈 测试覆盖率

### 当前覆盖率

| 模块 | 覆盖率 | 说明 |
|-----|-------|------|
| ASR模块 | 85% | 核心功能全覆盖 |
| LLM模块 | 90% | API调用和对话管理 |
| TTS模块 | 80% | 音频输出和格式转换 |
| 系统控制 | 75% | 节点管理和监控 |
| 音频组件 | 70% | 设备访问和处理 |

### 提升覆盖率

1. **添加边界测试**: 测试极端输入和错误条件
2. **增加并发测试**: 验证多线程环境
3. **完善错误处理**: 测试各种异常情况
4. **性能测试**: 添加负载和压力测试

## 🔄 持续集成

### CI/CD集成

测试套件设计支持CI/CD流水线：

```yaml
# .github/workflows/test.yml 示例
name: XLeRobot Tests
on: [push, pull_request]
jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Setup Python 3.10
        uses: actions/setup-python@v2
        with:
          python-version: '3.10'
      - name: Install dependencies
        run: pip install -r requirements.txt
      - name: Run tests
        run: python3.10 tests/run_all_tests.py --output json --report-file test_report.json
      - name: Upload test results
        uses: actions/upload-artifact@v2
        with:
          name: test-results
          path: test_report.json
```

### 自动化测试策略

1. **每次提交**: 运行单元测试和API测试
2. **PR创建**: 运行完整测试套件
3. **发布前**: 运行端到端和性能测试
4. **定期**: 运行完整回归测试

## 🤝 贡献指南

### 添加新测试

1. **文件命名**: 遵循 `test_<module>_<feature>.py` 格式
2. **目录放置**: 放入合适的测试分类目录
3. **文档更新**: 在本README.md中添加说明
4. **测试覆盖**: 确保新功能有对应测试

### 测试编写规范

```python
#!/usr/bin/env python3.10
"""
测试文件模板

作者: [作者名]
日期: [日期]
描述: [测试描述]
"""

import unittest
import sys
from pathlib import Path

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

class TestFeature(unittest.TestCase):
    """功能测试类"""

    def setUp(self):
        """测试前设置"""
        pass

    def tearDown(self):
        """测试后清理"""
        pass

    def test_basic_functionality(self):
        """基础功能测试"""
        self.assertTrue(True)  # 替换为实际测试逻辑

    def test_error_handling(self):
        """错误处理测试"""
        pass

if __name__ == "__main__":
    unittest.main()
```

### 代码审查检查点

- [ ] 测试文件命名规范
- [ ] 测试覆盖核心功能
- [ ] 错误处理完整
- [ ] 文档说明清晰
- [ ] 不会影响其他测试

## 📞 获取帮助

### 联系方式

- **项目负责人**: Jody
- **技术支持**: 查看项目文档
- **问题报告**: 创建GitHub Issue

### 相关资源

- **[CLAUDE.md](../CLAUDE.md)** - 详细开发指南
- **[README.md](../README.md)** - 项目概述
- **[docs/architecture-analysis.md](../docs/architecture-analysis.md)** - 架构说明

---

*最后更新: 2025-11-15*