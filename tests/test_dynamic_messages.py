#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
动态消息类型定义 - 用于测试功能
在编译问题解决前，使用动态定义的消息类
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from dataclasses import dataclass


@dataclass
class ASRResult:
    """动态ASR结果消息"""
    header: Header
    text: str = ""
    confidence: float = 0.0
    begin_time: int = 0
    end_time: int = 0
    status_code: int = 0
    message: str = ""


@dataclass
class LLMResponse:
    """动态LLM响应消息"""
    header: Header
    text: str = ""
    session_id: str = ""
    confidence: float = 0.0
    status_code: int = 0
    error_message: str = ""
    user_input: str = ""
    response_time: float = 0.0
    model_name: str = ""


@dataclass
class LLMStatus:
    """动态LLM状态消息"""
    header: Header
    node_name: str = ""
    state: int = 0
    avg_response_time: float = 0.0
    total_requests: int = 0
    failed_requests: int = 0
    cpu_usage: float = 0.0
    memory_usage: float = 0.0
    last_error: str = ""


@dataclass
class TTSStatus:
    """动态TTS状态消息"""
    header: Header
    node_name: str = ""
    state: int = 0
    queue_length: int = 0
    avg_synthesis_time: float = 0.0
    avg_playback_time: float = 0.0
    total_syntheses: int = 0
    total_playbacks: int = 0
    last_error: str = ""


def test_message_creation():
    """测试消息创建"""
    import time
    from rclpy.clock import Clock

    clock = Clock()

    # 测试ASRResult
    header = Header()
    header.stamp = clock.now().to_msg()

    asr_result = ASRResult(
        header=header,
        text="测试语音识别",
        confidence=0.95,
        status_code=0
    )
    print(f"✅ ASRResult创建成功: {asr_result.text}")

    # 测试LLMResponse
    llm_response = LLMResponse(
        header=header,
        text="你好，我是XLeBot助手",
        session_id="test_123",
        confidence=0.9,
        status_code=0,
        response_time=1.5
    )
    print(f"✅ LLMResponse创建成功: {llm_response.text}")

    # 测试LLMStatus
    llm_status = LLMStatus(
        header=header,
        node_name="llm_service_node",
        state=1,
        total_requests=5,
        failed_requests=0
    )
    print(f"✅ LLMStatus创建成功: {llm_status.node_name}")

    # 测试TTSStatus
    tts_status = TTSStatus(
        header=header,
        node_name="tts_service_node",
        state=2,
        queue_length=1,
        total_syntheses=3
    )
    print(f"✅ TTSStatus创建成功: {tts_status.node_name}")

    return True


if __name__ == "__main__":
    rclpy.init()

    try:
        success = test_message_creation()
        if success:
            print("🎉 所有动态消息类型测试通过！")
        else:
            print("❌ 消息测试失败")
    except Exception as e:
        print(f"❌ 测试异常: {e}")
    finally:
        rclpy.shutdown()