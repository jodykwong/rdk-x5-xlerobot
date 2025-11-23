#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
XLeRobot节点模块

包含所有核心ROS2节点的实现。
"""

from .asr_bridge_node import main as asr_bridge_main
from .llm_service_node import main as llm_service_main
from .tts_service_node import main as tts_service_main
from .voice_assistant_coordinator import main as coordinator_main

__all__ = [
    'asr_bridge_main',
    'llm_service_main',
    'tts_service_main',
    'coordinator_main'
]