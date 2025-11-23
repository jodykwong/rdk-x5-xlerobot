"""
系统控制模块

XLeRobot智能语音机器人系统的大脑和协调中心，
负责各模块间的协调通信、资源管理、状态监控和系统配置。
"""

# 导入实际存在的模块
from .architecture import SystemArchitecture, ModuleType, get_system_architecture
from .module_coordinator import ModuleCoordinator, MessageType, MessagePriority, get_module_coordinator
from .fault_tolerance import FaultTolerance, FailureType, Severity, get_fault_tolerance
from .scalability_manager import ScalabilityManager, ScalingStrategy, get_scalability_manager
from .config import SystemControlConfig, CONFIG

__version__ = "1.0.0"
__all__ = [
    "SystemArchitecture",
    "ModuleType",
    "ModuleCoordinator",
    "MessageType",
    "MessagePriority",
    "FaultTolerance",
    "FailureType",
    "Severity",
    "ScalabilityManager",
    "ScalingStrategy",
    "SystemControlConfig",
    "CONFIG",
    "get_system_architecture",
    "get_module_coordinator",
    "get_fault_tolerance",
    "get_scalability_manager"
]
