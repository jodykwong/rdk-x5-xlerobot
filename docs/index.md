# XleRobot 智能语音机器人 - 项目文档索引

**文档编号**: XLR-INDEX-P0-20251107-001
**项目名称**: XleRobot 家用机器人控制系统
**项目级别**: Brownfield Level 4 (企业级变更)
**文档类型**: 项目主索引文档
**生成日期**: 2025-11-07
**工作流**: Phase 0 Documentation - document-project

---

## 📋 项目概览

### 基本信息
- **项目名称**: XleRobot 智能语音机器人系统
- **项目类型**: Software - 家用机器人控制系统
- **项目级别**: Level 4 - Brownfield Enterprise Scale
- **技术栈**: D-Robotics RDK X5 + ROS2 Humble + TROS 2.4.3 + Python 3.10
- **当前阶段**: Phase 3 Solutioning 已完成，准备进入 Phase 4 Implementation
- **文档版本**: v1.0 (Brownfield Level 4 标准版)

### 项目愿景
构建基于粤语语音交互的智能家用机器人控制系统，采用三阶段架构演进策略，从在线服务验证开始，逐步实现本地化处理，最终完成完整的机器人系统集成。

### 三阶段架构演进
- **第一阶段**: 全在线服务 (阿里云ASR + Qwen3-VL-Plus + 阿里云TTS) ✅ 当前执行阶段
- **第二阶段**: 离线服务 (TROS hobot_audio + hobot_llamacpp + FastSpeech2) ⏳ 后续阶段
- **第三阶段**: xlerobot集成 (TROS + ROS2深度集成) ⏳ 最终阶段

---

## 🗺️ 文档导航

### 📚 [项目概览文档](./project-overview.md)
项目的总体介绍，包括基本信息、核心特性、项目目标、系统架构、当前状态等。

### 🏗️ [系统架构文档](./architecture-analysis.md)
系统架构的深入分析，包括整体架构设计、核心组件、性能优化、安全架构、部署架构等。

### 📁 [源码树分析文档](./source-tree-analysis.md)
完整的源代码结构分析，包括目录结构、模块组织、设计模式、通信机制等。

### 🔧 [开发指南文档](./development-guide.md)
开发环境配置、构建流程、测试方法、调试技巧等开发相关指导。

### 🧩 [组件清单文档](./component-inventory.md)
系统中所有可重用组件的详细清单，包括组件功能、接口定义、使用方法等。

### 📡 [API契约文档](./api-contracts.md)
系统API接口的详细定义，包括端点、请求/响应格式、错误处理等。

### 📊 [数据模型文档](./data-models.md)
系统数据结构和模型的详细说明，包括数据流、存储格式、验证规则等。

### 🚀 [部署指南文档](./deployment-guide.md)
系统部署的详细指导，包括环境要求、部署步骤、配置管理、监控等。

### 📖 [扩展文档集](./project-docs/index.md)
更详细的项目扩展文档，包括技术栈文档、重构规划、工作流程执行报告等。

---

## 🎯 快速参考

### 环境要求
- **操作系统**: Ubuntu 22.04 LTS
- **ROS2版本**: Humble (强制要求)
- **Python版本**: 系统Python 3.10 (/usr/bin/python3.10，严禁使用Python 3.13)
- **硬件平台**: D-Robotics RDK X5 V1.0 (10Tops算力，7GB可用内存)
- **TROS版本**: 2.4.3 (66个算法包支持)

### 核心技术栈
- **语音识别**: 阿里云ASR (第一阶段) → TROS hobot_audio (第二阶段)
- **语言理解**: 通义千问Qwen3-VL-Plus (第一阶段) → TROS hobot_llamacpp (第二阶段)
- **语音合成**: 阿里云TTS (第一阶段) → FastSpeech2 (第二阶段)
- **视觉系统**: IMX219摄像头，1920x1080@30fps，零拷贝优化
- **机器人控制**: ROS2 Humble + TROS 2.4.3深度集成

### 开发环境配置
```bash
# 正确的开发环境激活方式
source /opt/ros/humble/setup.bash    # ROS2 Humble + Python3.10
source /opt/tros/humble/setup.bash   # TROS 2.4.3
# 使用 /usr/bin/python3.10 进行开发
```

### 当前状态
- **Phase 0**: Documentation ✅ 已完成 (本文档集)
- **Phase 1**: Analysis ✅ 已完成 (技术和商业分析)
- **Phase 2**: Planning ✅ 已完成 (PRD和UX设计)
- **Phase 3**: Solutioning ✅ 已完成 (架构设计和Gate-Check验证)
- **Phase 4**: Implementation ⏳ 准备开始 (Sprint Planning)

### 重要提醒
- **严格环境要求**: 所有后续开发必须在ROS2 Humble环境中完成
- **Python版本强制**: 必须使用系统Python3.10，严禁使用Python3.13
- **Brownfield合规**: 必须保持与现有系统的完全兼容性
- **质量标准**: 符合Level 4企业级变更管理要求

---

## 🔍 AI辅助开发指导

### 文档使用建议
1. **新开发者**: 从项目概览开始，然后阅读开发指南
2. **架构决策**: 参考架构文档和组件清单
3. **API集成**: 查阅API契约和数据模型文档
4. **部署运维**: 参考部署指南和监控配置

### 重构和扩展指导
1. **理解现有架构**: 详细阅读架构分析和源码树分析
2. **识别重用组件**: 查阅组件清单，优先使用现有组件
3. **遵循设计模式**: 参考架构文档中的设计模式指导
4. **保持向后兼容**: 确保新功能不破坏现有接口

---

## 📞 支持和联系

### 文档维护
- **文档版本**: v1.0
- **最后更新**: 2025-11-07
- **维护流程**: 随项目进展持续更新
- **反馈渠道**: 通过项目管理系统提交文档改进建议

### 相关资源
- **项目仓库**: /home/sunrise/xlerobot
- **构建输出**: /home/sunrise/xlerobot/build/
- **测试报告**: /home/sunrise/xlerobot/tests/
- **配置文件**: /home/sunrise/xlerobot/config/

---

*本文档遵循Brownfield Level 4企业级标准生成，为AI辅助开发和人工开发提供完整的项目上下文指导。*