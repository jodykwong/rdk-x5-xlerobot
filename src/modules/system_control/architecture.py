"""
系统架构设计 - SystemArchitecture
=====================================

实现xlerobot系统的模块化架构设计，支持5个核心模块的高效协作。
采用ROS2分布式架构，确保系统可用性>99.9%，模块协调延迟<10ms。

Author: BMad System Architect
Created: 2025-11-05
"""

from enum import Enum
from typing import Dict, List, Optional, Any, Set
from dataclasses import dataclass, field
from datetime import datetime
import logging
import asyncio
from concurrent.futures import ThreadPoolExecutor

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logging.warning("ROS2 not available, running in standalone mode")


class ModuleState(Enum):
    """模块状态枚举"""
    UNINITIALIZED = "uninitialized"
    INITIALIZING = "initializing"
    READY = "ready"
    RUNNING = "running"
    PAUSED = "paused"
    STOPPING = "stopping"
    STOPPED = "stopped"
    ERROR = "error"
    MAINTENANCE = "maintenance"


class ModuleType(Enum):
    """模块类型枚举"""
    ASR = "asr"  # 语音识别
    LLM = "llm"  # 大语言模型
    TTS = "tts"  # 语音合成
    SYSTEM_CONTROL = "system_control"  # 系统控制
    SMART_HOME = "smart_home"  # 智能家居


@dataclass
class ModuleInfo:
    """模块信息数据类"""
    module_id: str
    module_type: ModuleType
    name: str
    version: str
    dependencies: List[str] = field(default_factory=list)
    state: ModuleState = ModuleState.UNINITIALIZED
    health_status: str = "unknown"
    performance_metrics: Dict[str, float] = field(default_factory=dict)
    last_heartbeat: Optional[datetime] = None
    node_handle: Optional[str] = None
    config: Dict[str, Any] = field(default_factory=dict)


@dataclass
class MessageFlow:
    """消息流定义"""
    source_module: str
    target_module: str
    message_type: str
    topic: str
    qos_profile: Optional[Dict[str, Any]] = None


class SystemArchitecture:
    """
    系统架构控制器

    负责：
    1. 管理5个核心模块的生命周期
    2. 定义模块间通信接口
    3. 监控模块健康状态
    4. 协调模块间的数据流
    5. 实现模块热插拔
    """

    def __init__(self, node_name: str = "system_architecture"):
        """初始化系统架构"""
        self.logger = logging.getLogger(f"SystemArchitecture.{node_name}")
        self.node_name = node_name

        # 核心模块注册表
        self.modules: Dict[str, ModuleInfo] = {}
        self.module_types: Dict[ModuleType, List[str]] = {
            ModuleType.ASR: [],
            ModuleType.LLM: [],
            ModuleType.TTS: [],
            ModuleType.SYSTEM_CONTROL: [],
            ModuleType.SMART_HOME: []
        }

        # 消息流配置
        self.message_flows: List[MessageFlow] = []

        # 依赖关系图
        self.dependency_graph: Dict[str, Set[str]] = {}

        # ROS2相关
        self.ros2_node = None
        self.ros2_context = None

        # 性能指标
        self.system_metrics = {
            "uptime": 0.0,
            "total_messages": 0,
            "failed_messages": 0,
            "average_latency": 0.0,
            "module_count": 0,
            "active_module_count": 0
        }

        # 执行器
        self.executor = ThreadPoolExecutor(max_workers=10)

        self.logger.info("SystemArchitecture initialized")

    def initialize(self) -> bool:
        """初始化系统架构"""
        try:
            self.logger.info("Initializing SystemArchitecture...")

            # 初始化ROS2
            if ROS2_AVAILABLE:
                self._init_ros2()

            # 注册5个核心模块
            self._register_core_modules()

            # 配置消息流
            self._setup_message_flows()

            # 构建依赖关系图
            self._build_dependency_graph()

            self.logger.info("SystemArchitecture initialized successfully")
            return True

        except Exception as e:
            self.logger.error(f"Failed to initialize SystemArchitecture: {e}")
            return False

    def _init_ros2(self):
        """初始化ROS2节点"""
        if not ROS2_AVAILABLE:
            return

        try:
            rclpy.init()
            self.ros2_context = rclpy.get_default_context()
            self.ros2_node = Node(self.node_name)

            # 配置QoS策略
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=1,  # Keep last
                depth=10
            )

            self.logger.info("ROS2 node initialized")

        except Exception as e:
            self.logger.error(f"ROS2 initialization failed: {e}")
            self.ros2_node = None

    def _register_core_modules(self):
        """注册5个核心模块"""
        core_modules = [
            ModuleInfo(
                module_id="asr_core",
                module_type=ModuleType.ASR,
                name="ASR Core Module",
                version="1.0.0",
                dependencies=["system_control"],
                config={"sample_rate": 16000, "channels": 1}
            ),
            ModuleInfo(
                module_id="llm_core",
                module_type=ModuleType.LLM,
                name="LLM Core Module",
                version="1.0.0",
                dependencies=["asr_core", "system_control"],
                config={"model": "qwen-plus", "max_tokens": 2000}
            ),
            ModuleInfo(
                module_id="tts_core",
                module_type=ModuleType.TTS,
                name="TTS Core Module",
                version="1.0.0",
                dependencies=["llm_core", "system_control"],
                config={"voice": "female_001", "format": "wav"}
            ),
            ModuleInfo(
                module_id="system_control",
                module_type=ModuleType.SYSTEM_CONTROL,
                name="System Control Module",
                version="1.0.0",
                dependencies=[],
                config={"coordinator_enabled": True}
            ),
            ModuleInfo(
                module_id="smart_home",
                module_type=ModuleType.SMART_HOME,
                name="Smart Home Module",
                version="1.0.0",
                dependencies=["system_control"],
                config={"protocol": "mqtt", "broker": "localhost"}
            )
        ]

        for module in core_modules:
            self.register_module(module)

        self.logger.info(f"Registered {len(core_modules)} core modules")

    def _setup_message_flows(self):
        """配置模块间消息流"""
        message_flows = [
            # ASR -> LLM: 语音转文本
            MessageFlow(
                source_module="asr_core",
                target_module="llm_core",
                message_type="voice_text",
                topic="/xlerobot/asr/text"
            ),
            # LLM -> TTS: 文本回复
            MessageFlow(
                source_module="llm_core",
                target_module="tts_core",
                message_type="text_reply",
                topic="/xlerobot/llm/reply"
            ),
            # 系统控制 -> 各模块: 控制指令
            MessageFlow(
                source_module="system_control",
                target_module="*",
                message_type="control_command",
                topic="/xlerobot/control/command"
            ),
            # 各模块 -> 系统控制: 状态报告
            MessageFlow(
                source_module="*",
                target_module="system_control",
                message_type="status_report",
                topic="/xlerobot/control/status"
            ),
            # LLM -> 智能家居: 设备控制
            MessageFlow(
                source_module="llm_core",
                target_module="smart_home",
                message_type="device_command",
                topic="/xlerobot/smart_home/command"
            )
        ]

        self.message_flows.extend(message_flows)
        self.logger.info(f"Setup {len(message_flows)} message flows")

    def _build_dependency_graph(self):
        """构建模块依赖关系图"""
        for module_id, module_info in self.modules.items():
            self.dependency_graph[module_id] = set(module_info.dependencies)

        self.logger.info("Dependency graph built")

    def register_module(self, module_info: ModuleInfo) -> bool:
        """注册模块"""
        try:
            if module_info.module_id in self.modules:
                self.logger.warning(f"Module {module_info.module_id} already registered")
                return False

            self.modules[module_info.module_id] = module_info
            self.module_types[module_info.module_type].append(module_info.module_id)
            self.dependency_graph[module_info.module_id] = set(module_info.dependencies)

            self.logger.info(f"Module registered: {module_info.module_id}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to register module: {e}")
            return False

    def unregister_module(self, module_id: str) -> bool:
        """注销模块"""
        try:
            if module_id not in self.modules:
                self.logger.warning(f"Module {module_id} not found")
                return False

            module_info = self.modules[module_id]
            del self.modules[module_id]
            self.module_types[module_info.module_type].remove(module_id)
            del self.dependency_graph[module_id]

            self.logger.info(f"Module unregistered: {module_id}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to unregister module: {e}")
            return False

    def start_module(self, module_id: str) -> bool:
        """启动模块"""
        try:
            if module_id not in self.modules:
                self.logger.error(f"Module {module_id} not found")
                return False

            module = self.modules[module_id]
            module.state = ModuleState.INITIALIZING

            # 检查依赖是否就绪
            for dep_id in module.dependencies:
                if dep_id not in self.modules:
                    self.logger.error(f"Dependency {dep_id} not found for module {module_id}")
                    module.state = ModuleState.ERROR
                    return False

                dep_module = self.modules[dep_id]
                if dep_module.state not in [ModuleState.READY, ModuleState.RUNNING]:
                    self.logger.error(f"Dependency {dep_id} not ready for module {module_id}")
                    module.state = ModuleState.ERROR
                    return False

            module.state = ModuleState.READY
            self.logger.info(f"Module started: {module_id}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to start module {module_id}: {e}")
            if module_id in self.modules:
                self.modules[module_id].state = ModuleState.ERROR
            return False

    def stop_module(self, module_id: str, force: bool = False) -> bool:
        """停止模块"""
        try:
            if module_id not in self.modules:
                self.logger.error(f"Module {module_id} not found")
                return False

            module = self.modules[module_id]

            if not force:
                # 检查是否有其他模块依赖此模块
                for mod_id, deps in self.dependency_graph.items():
                    if module_id in deps:
                        self.logger.error(f"Cannot stop {module_id}: other modules depend on it")
                        return False

            module.state = ModuleState.STOPPED
            self.logger.info(f"Module stopped: {module_id}")
            return True

        except Exception as e:
            self.logger.error(f"Failed to stop module {module_id}: {e}")
            return False

    def get_module_info(self, module_id: str) -> Optional[ModuleInfo]:
        """获取模块信息"""
        return self.modules.get(module_id)

    def list_modules(self, module_type: Optional[ModuleType] = None) -> List[ModuleInfo]:
        """列出模块"""
        if module_type is None:
            return list(self.modules.values())
        else:
            return [self.modules[mid] for mid in self.module_types.get(module_type, [])]

    def get_system_health(self) -> Dict[str, Any]:
        """获取系统健康状态"""
        total_modules = len(self.modules)
        ready_modules = sum(1 for m in self.modules.values() if m.state in [ModuleState.READY, ModuleState.RUNNING])
        error_modules = sum(1 for m in self.modules.values() if m.state == ModuleState.ERROR)

        health = {
            "total_modules": total_modules,
            "ready_modules": ready_modules,
            "error_modules": error_modules,
            "health_percentage": (ready_modules / total_modules * 100) if total_modules > 0 else 0,
            "availability": ((total_modules - error_modules) / total_modules * 100) if total_modules > 0 else 0,
            "timestamp": datetime.now().isoformat()
        }

        return health

    def update_performance_metrics(self, module_id: str, metrics: Dict[str, float]):
        """更新性能指标"""
        if module_id in self.modules:
            self.modules[module_id].performance_metrics.update(metrics)
            self.modules[module_id].last_heartbeat = datetime.now()

    def get_message_flows(self) -> List[MessageFlow]:
        """获取消息流配置"""
        return self.message_flows.copy()

    def get_dependency_graph(self) -> Dict[str, Set[str]]:
        """获取依赖关系图"""
        return {k: v.copy() for k, v in self.dependency_graph.items()}

    def is_module_dependent_on(self, module_id: str, target_id: str) -> bool:
        """检查模块是否依赖目标模块"""
        if module_id not in self.dependency_graph:
            return False
        return target_id in self.dependency_graph[module_id]

    def get_module_state(self, module_id: str) -> Optional[ModuleState]:
        """获取模块状态"""
        module = self.modules.get(module_id)
        return module.state if module else None

    def validate_architecture(self) -> Dict[str, Any]:
        """验证架构完整性"""
        validation_result = {
            "valid": True,
            "errors": [],
            "warnings": []
        }

        # 检查核心模块
        required_modules = ["asr_core", "llm_core", "tts_core", "system_control"]
        for req_module in required_modules:
            if req_module not in self.modules:
                validation_result["errors"].append(f"Missing required module: {req_module}")
                validation_result["valid"] = False

        # 检查依赖循环
        visited = set()
        rec_stack = set()

        def has_cycle(module_id: str) -> bool:
            visited.add(module_id)
            rec_stack.add(module_id)

            for dep in self.dependency_graph.get(module_id, set()):
                if dep not in visited:
                    if has_cycle(dep):
                        return True
                elif dep in rec_stack:
                    return True

            rec_stack.remove(module_id)
            return False

        for module_id in self.modules:
            if module_id not in visited:
                if has_cycle(module_id):
                    validation_result["errors"].append("Circular dependency detected")
                    validation_result["valid"] = False
                    break

        # 检查系统控制模块
        if "system_control" in self.modules:
            sys_ctrl = self.modules["system_control"]
            if sys_ctrl.dependencies:
                validation_result["warnings"].append("System control module should not have dependencies")

        return validation_result

    async def shutdown(self):
        """关闭系统架构"""
        self.logger.info("Shutting down SystemArchitecture...")

        # 停止所有模块
        for module_id in list(self.modules.keys()):
            self.stop_module(module_id, force=True)

        # 关闭线程池
        self.executor.shutdown(wait=True)

        # 关闭ROS2
        if self.ros2_node:
            self.ros2_node.destroy_node()
        if self.ros2_context:
            rclpy.shutdown()

        self.logger.info("SystemArchitecture shutdown complete")


# 单例模式
_system_architecture_instance: Optional[SystemArchitecture] = None


def get_system_architecture() -> SystemArchitecture:
    """获取系统架构单例"""
    global _system_architecture_instance
    if _system_architecture_instance is None:
        _system_architecture_instance = SystemArchitecture()
    return _system_architecture_instance


def initialize_system_architecture(node_name: str = "system_architecture") -> SystemArchitecture:
    """初始化系统架构单例"""
    global _system_architecture_instance
    _system_architecture_instance = SystemArchitecture(node_name)
    _system_architecture_instance.initialize()
    return _system_architecture_instance
