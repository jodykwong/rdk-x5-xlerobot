"""
系统控制模块测试套件
===================

测试系统控制模块的所有组件：
- 系统架构
- 模块协调器
- 故障容错
- 可扩展性管理器
- 架构验证器

Author: BMad System Architect
Created: 2025-11-05
"""

import unittest
import sys
import os

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))

__all__ = [
    'TestSystemArchitecture',
    'TestModuleCoordinator',
    'TestFaultTolerance',
    'TestScalabilityManager',
    'TestArchitectureValidator',
    'TestSystemControlIntegration'
]
