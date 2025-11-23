#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-

"""
XLeRobot Phase 1 模块
语音助手第一阶段功能模块，包含唤醒词检测和基础语音交互功能

Epic: 1 - 语音唤醒和基础识别
作者: Claude Code
创建日期: 2025-11-19
"""

from .wake_word_detector import WakeWordDetector

__all__ = [
    'WakeWordDetector'
]