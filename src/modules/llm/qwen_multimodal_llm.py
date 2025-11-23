#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Qwen3-VL-Plus 多模态LLM集成
支持文本、图像等多模态输入，提供智能对话能力
"""

import os
import json
import logging
import base64
import asyncio
import aiohttp
from typing import Dict, Any, Optional, List, Union
from dataclasses import dataclass

logger = logging.getLogger(__name__)

@dataclass
class MultimodalMessage:
    """多模态消息"""
    content: str
    message_type: str  # text, image, audio
    media_data: Optional[bytes] = None
    media_type: Optional[str] = None  # image/jpeg, image/png, etc.

class QwenMultimodalLLM:
    """Qwen3-VL-Plus 多模态LLM客户端"""

    def __init__(self, api_key: str = "", model: str = "qwen3-vl-plus"):
        self.api_key = api_key or os.getenv("DASHSCOPE_API_KEY", "")
        self.model = model
        self.base_url = "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation"
        self.session = None

        # 支持的输入类型
        self.supported_image_types = ["image/jpeg", "image/png", "image/webp"]
        self.supported_audio_types = ["audio/wav", "audio/mp3", "audio/flac"]

        # 默认参数
        self.default_temperature = 0.7
        self.default_max_tokens = 2000
        self.default_top_p = 0.8

    async def initialize(self):
        """异步初始化"""
        try:
            self.session = aiohttp.ClientSession(
                timeout=aiohttp.ClientTimeout(total=60),
                headers={
                    "Authorization": f"Bearer {self.api_key}",
                    "Content-Type": "application/json"
                }
            )
            logger.info("✅ Qwen多模态LLM客户端初始化成功")
            return True
        except Exception as e:
            logger.error(f"❌ Qwen多模态LLM初始化失败: {e}")
            return False

    async def close(self):
        """关闭连接"""
        if self.session:
            await self.session.close()
            self.session = None

    def _prepare_messages(self, messages: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """准备消息格式"""
        formatted_messages = []

        for msg in messages:
            if msg.get("role") in ["system", "user", "assistant"]:
                formatted_msg = {
                    "role": msg["role"],
                    "content": []
                }

                content = msg.get("content", "")
                if isinstance(content, str):
                    formatted_msg["content"].append({"text": content})
                elif isinstance(content, list):
                    for item in content:
                        if isinstance(item, str):
                            formatted_msg["content"].append({"text": item})
                        elif isinstance(item, dict):
                            if "text" in item:
                                formatted_msg["content"].append({"text": item["text"]})
                            elif "image" in item:
                                formatted_msg["content"].append({"image": item["image"]})
                            elif "audio" in item:
                                formatted_msg["content"].append({"audio": item["audio"]})

                formatted_messages.append(formatted_msg)

        return formatted_messages

    def _encode_image(self, image_data: bytes) -> str:
        """编码图像数据"""
        return base64.b64encode(image_data).decode('utf-8')

    def _encode_audio(self, audio_data: bytes) -> str:
        """编码音频数据"""
        return base64.b64encode(audio_data).decode('utf-8')

    async def generate_response(
        self,
        prompt: str,
        context: Optional[List[Dict[str, Any]]] = None,
        temperature: Optional[float] = None,
        max_tokens: Optional[int] = None,
        top_p: Optional[float] = None,
        images: Optional[List[bytes]] = None,
        audio_data: Optional[bytes] = None
    ) -> Optional[str]:
        """生成多模态响应"""
        try:
            if not self.session:
                logger.error("❌ LLM客户端未初始化")
                return None

            # 构建消息
            messages = []

            # 添加系统提示
            system_prompt = """你是傻强，一个友好、智能的粤语语音助手。你具备以下特点：
1. 用粤语回答问题
2. 回答简洁明了，适合语音播放
3. 具备多模态理解能力，可以处理图像、音频和文本
4. 乐于助人，对各种问题都有耐心
5. 支持天气查询、时间询问、生活帮助等功能"""

            messages.append({
                "role": "system",
                "content": system_prompt
            })

            # 添加上下文
            if context:
                messages.extend(context)

            # 构建用户消息
            user_content = [{"text": prompt}]

            # 添加图像
            if images:
                for i, img_data in enumerate(images):
                    user_content.append({
                        "image": f"data:image/jpeg;base64,{self._encode_image(img_data)}"
                    })

            # 添加音频
            if audio_data:
                user_content.append({
                    "audio": f"data:audio/wav;base64,{self._encode_audio(audio_data)}"
                })

            messages.append({
                "role": "user",
                "content": user_content
            })

            # 构建请求参数
            params = {
                "model": self.model,
                "input": {
                    "messages": self._prepare_messages(messages)
                },
                "parameters": {
                    "temperature": temperature or self.default_temperature,
                    "max_tokens": max_tokens or self.default_max_tokens,
                    "top_p": top_p or self.default_top_p,
                    "stream": False
                }
            }

            logger.info(f"🤖 调用Qwen多模态LLM: {prompt[:50]}...")

            # 发送请求
            async with self.session.post(self.base_url, json=params) as response:
                if response.status == 200:
                    result = await response.json()

                    if "output" in result and "text" in result["output"]:
                        response_text = result["output"]["text"].strip()
                        logger.info(f"✅ LLM响应: {response_text[:100]}...")
                        return response_text
                    else:
                        logger.error(f"❌ LLM响应格式错误: {result}")
                        return None
                else:
                    logger.error(f"❌ LLM请求失败: {response.status} - {await response.text()}")
                    return None

        except Exception as e:
            logger.error(f"❌ LLM生成响应失败: {e}")
            return None

    async def process_voice_command(
        self,
        text: str,
        previous_context: Optional[List[Dict[str, Any]]] = None
    ) -> Optional[str]:
        """处理语音命令"""
        try:
            # 专门为语音交互优化的提示
            prompt = f"用户说：{text}\n请用粤语简短回答："

            # 构建上下文
            context = []
            if previous_context:
                # 限制上下文长度，避免token超限
                context = previous_context[-3:] if len(previous_context) > 3 else previous_context

            response = await self.generate_response(
                prompt=prompt,
                context=context,
                temperature=0.7,
                max_tokens=200  # 语音回复限制在200字符内
            )

            if response:
                # 确保回复适合语音播放
                response = response.strip()
                if len(response) > 150:
                    response = response[:150] + "..."

                logger.info(f"🎤 语音命令处理: {text[:30]}... -> {response[:50]}...")
                return response
            else:
                return "抱歉，我现在无法处理这个问题，请稍后再试。"

        except Exception as e:
            logger.error(f"❌ 语音命令处理失败: {e}")
            return "系统遇到问题，请稍后再试。"

    async def analyze_image_query(
        self,
        image_data: bytes,
        query: str,
        context: Optional[List[Dict[str, Any]]] = None
    ) -> Optional[str]:
        """分析图像查询"""
        try:
            prompt = f"用户上传图片并询问：{query}\n请用粤语详细描述图片内容并回答问题。"

            response = await self.generate_response(
                prompt=prompt,
                context=context,
                images=[image_data],
                max_tokens=500
            )

            if response:
                logger.info(f"🖼️ 图像查询分析: {query[:30]}...")
                return response
            else:
                return "我无法分析这张图片，请确认图片清晰度。"

        except Exception as e:
            logger.error(f"❌ 图像查询分析失败: {e}")
            return "图像分析遇到问题，请稍后再试。"

    async def get_weather_response(self, location: str = "当前地点") -> Optional[str]:
        """获取天气回复"""
        try:
            # 这里可以集成真实的天气API
            # 目前使用模拟回复
            weather_responses = [
                f"{location}今日天气晴朗，温度适宜，适合出行",
                f"{location}今日多云，温度适中，建议携带外套",
                f"{location}今日有小雨，记得带伞出门",
                f"{location}今日阴天，温度偏凉，注意保暖"
            ]

            import random
            response = random.choice(weather_responses)

            logger.info(f"🌤️ 天气查询: {location}")
            return response

        except Exception as e:
            logger.error(f"❌ 天气查询失败: {e}")
            return "抱歉，暂时无法获取天气信息。"

    async def get_time_response(self) -> Optional[str]:
        """获取时间回复"""
        try:
            import datetime
            now = datetime.datetime.now()

            # 粤语时间表达
            hour = now.hour
            minute = now.minute

            if hour < 12:
                time_period = "上午"
            elif hour < 18:
                time_period = "下午"
            else:
                time_period = "晚上"

            response = f"现在系{time_period}{hour:02d}点{minute:02d}分"

            logger.info(f"⏰ 时间查询: {response}")
            return response

        except Exception as e:
            logger.error(f"❌ 时间查询失败: {e}")
            return "抱歉，无法获取当前时间。"

    async def handle_general_query(self, query: str) -> Optional[str]:
        """处理一般查询"""
        try:
            # 通用问题处理
            prompt = f"用户询问：{query}\n请用粤语友好、简洁地回答。"

            response = await self.generate_response(
                prompt=prompt,
                max_tokens=300
            )

            if response:
                logger.info(f"💬 一般查询: {query[:30]}...")
                return response
            else:
                return "抱歉，我暂时无法回答这个问题。"

        except Exception as e:
            logger.error(f"❌ 一般查询处理失败: {e}")
            return "系统遇到问题，请稍后再试。"

async def create_multimodal_llm(api_key: str = "", model: str = "qwen3-vl-plus") -> QwenMultimodalLLM:
    """创建多模态LLM实例"""
    llm = QwenMultimodalLLM(api_key=api_key, model=model)
    if await llm.initialize():
        return llm
    else:
        raise RuntimeError("多模态LLM初始化失败")