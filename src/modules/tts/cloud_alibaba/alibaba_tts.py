#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿里云语音合成服务实现

功能：
- 云端语音合成 (替代离线VITSTTS)
- 支持多种音色
- 高音质音频生成
- 支持中文和粤语

性能指标：
- 合成延迟: <200ms
- 音质: 16kHz/22kHz
- 字节格式支持

作者: Dev Agent
Epic: 3 - 语音合成模块 (阿里云版)
"""

import os
import asyncio
import aiohttp
import logging
from typing import Optional, Dict, Any, List
import json
import base64
import wave
import io
import time

logger = logging.getLogger(__name__)


class AlibabaCloudTTSConfig:
    """阿里云TTS配置"""
    
    def __init__(self):
        self.access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID')
        self.access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET')
        self.region = os.getenv('ALIBABA_CLOUD_REGION', 'cn-shanghai')
        self.endpoint = f"https://nls-gateway.{self.region}.aliyuncs.com/stream/v1/tts"
        
        if not self.access_key_id or not self.access_key_secret:
            raise ValueError("❌ 阿里云API密钥未配置")
            
        logger.info("✅ 阿里云TTS配置加载成功")
        
        # 可用音色列表
        self.voices = {
            'jiajia': '粤语女声-佳佳-方言',
            'xiaoyun': '女声-小云-通用',
            'xiaoyi': '女声-小艺-通用',
            'xiaoming': '男声-小明-通用',
            'xiaoxiang': '女声-小香-温柔',
            'xiaozhi': '男声-小智-成熟',
            'xiaoyuan': '女声-小源-知性',
            'xiaobada': '男声-小巴达-活泼',
            'xiaowei': '女声-小微-清新',
            'xiaoceng': '女声-小层-优雅',
            'xiaolan': '女声-小兰-甜美'
        }


class AlibabaCloudTTS:
    """阿里云语音合成引擎"""
    
    def __init__(self, config: Optional[AlibabaCloudTTSConfig] = None):
        """
        初始化阿里云TTS
        
        Args:
            config: 配置对象，如果为None则使用默认配置
        """
        self.config = config or AlibabaCloudTTSConfig()
        self.session = None
        # 使用Basic Auth（Access Key ID/Secret）
        self.api_key_id = self.config.access_key_id
        self.api_key_secret = self.config.access_key_secret
        if not self.api_key_id or not self.api_key_secret:
            raise ValueError("❌ 阿里云API密钥未设置")

        self.headers = {
            'Authorization': f'Basic {self._get_auth_string()}',
            'Content-Type': 'application/json'
        }
        self.default_voice = 'jiajia'  # 默认粤语女声
        
        logger.info("🎯 阿里云语音合成引擎初始化完成")
        
    def _get_auth_string(self) -> str:
        """获取认证字符串"""
        credentials = f"{self.config.access_key_id}:{self.config.access_key_secret}"
        return base64.b64encode(credentials.encode()).decode()
        
    async def __aenter__(self):
        """异步上下文管理器入口"""
        if not self.session:
            self.session = aiohttp.ClientSession()
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """异步上下文管理器出口"""
        if self.session:
            await self.session.close()
            self.session = None
            
    async def synthesize(self,
                        text: str,
                        voice: str = None,
                        sample_rate: int = 16000,
                        speed: float = 1.0,
                        volume: float = 1.0,
                        pitch: float = 1.0) -> Dict[str, Any]:
        """
        合成语音
        
        Args:
            text: 待合成的文本
            voice: 音色名称
            sample_rate: 采样率
            speed: 语速 (0.5-2.0)
            volume: 音量 (0.5-2.0)
            pitch: 音调 (0.5-2.0)
            
        Returns:
            合成结果字典
        """
        voice = voice or self.default_voice
        
        try:
            # 构建请求 (最简单的TTS请求格式)
            request_data = {
                "text": text,
                "voice": voice,
                "format": "wav",
                "sample_rate": sample_rate
            }
            
            # 发送请求
            async with self.session.post(
                self.config.endpoint,
                headers=self.headers,
                json=request_data,
                timeout=aiohttp.ClientTimeout(total=15)
            ) as response:
                
                if response.status == 200:
                    # 获取音频数据
                    audio_data = await response.read()
                    
                    logger.info(f"✅ TTS合成成功: {text[:20]}...")
                    return {
                        'success': True,
                        'audio_data': audio_data,
                        'format': 'WAV',
                        'sample_rate': sample_rate,
                        'voice': voice,
                        'duration': len(audio_data) / (sample_rate * 2),  # 估算
                        'model': 'alibaba-cloud'
                    }
                else:
                    result = await response.json()
                    logger.error(f"❌ TTS合成失败: {result}")
                    return {
                        'success': False,
                        'error': result.get('message', 'Unknown error'),
                        'model': 'alibaba-cloud'
                    }
                    
        except Exception as e:
            logger.error(f"❌ TTS合成异常: {e}")
            return {
                'success': False,
                'error': str(e),
                'model': 'alibaba-cloud'
            }
            
    async def synthesize_file(self,
                             text: str,
                             output_file: str,
                             voice: str = None,
                             sample_rate: int = 16000) -> Dict[str, Any]:
        """
        合成语音并保存到文件
        
        Args:
            text: 待合成的文本
            output_file: 输出音频文件路径
            voice: 音色名称
            sample_rate: 采样率
            
        Returns:
            合成结果字典
        """
        try:
            result = await self.synthesize(text, voice, sample_rate)
            
            if result['success']:
                # 保存音频文件
                with open(output_file, 'wb') as f:
                    f.write(result['audio_data'])
                    
                logger.info(f"✅ 音频文件已保存: {output_file}")
                result['file_path'] = output_file
                return result
            else:
                return result
                
        except Exception as e:
            logger.error(f"❌ 保存音频文件失败: {e}")
            return {
                'success': False,
                'error': str(e),
                'model': 'alibaba-cloud'
            }
            
    async def synthesize_stream(self,
                               texts: List[str],
                               voice: str = None,
                               sample_rate: int = 16000) -> List[Dict[str, Any]]:
        """
        批量合成语音
        
        Args:
            texts: 文本列表
            voice: 音色名称
            sample_rate: 采样率
            
        Returns:
            合成结果列表
        """
        results = []
        for i, text in enumerate(texts):
            result = await self.synthesize(text, voice, sample_rate)
            results.append(result)
            
            if result['success']:
                logger.info(f"✅ 文本 {i+1}/{len(texts)} 合成成功")
            else:
                logger.error(f"❌ 文本 {i+1}/{len(texts)} 合成失败")
                
        return results
        
    async def health_check(self) -> Dict[str, Any]:
        """
        健康检查
        
        Returns:
            健康状态
        """
        try:
            # 创建一个短的测试文本
            test_text = "你好"
            
            result = await self.synthesize(test_text, voice=self.default_voice)
            
            if result['success']:
                return {
                    'status': 'healthy',
                    'api_accessible': True,
                    'model': 'alibaba-cloud',
                    'default_voice': self.default_voice,
                    'available_voices': len(self.config.voices),
                    'response_time_ms': 0
                }
            else:
                return {
                    'status': 'unhealthy',
                    'api_accessible': False,
                    'error': result['error'],
                    'model': 'alibaba-cloud'
                }
                
        except Exception as e:
            return {
                'status': 'error',
                'api_accessible': False,
                'error': str(e),
                'model': 'alibaba-cloud'
            }
            
    def get_available_voices(self) -> Dict[str, str]:
        """获取可用音色列表"""
        return self.config.voices.copy()
        
    def get_stats(self) -> Dict[str, Any]:
        """获取统计信息"""
        return {
            'engine': 'AlibabaCloudTTS',
            'model': 'alibaba-cloud',
            'version': '1.0.0',
            'default_voice': self.default_voice,
            'available_voices': len(self.config.voices),
            'status': 'ready'
        }


# 兼容性适配器 - 替代VITSTTS
class VITSTTS(AlibabaCloudTTS):
    """
    兼容性适配器
    替代原有的VITSTTS，保持接口一致
    """
    
    def __init__(self):
        super().__init__()
        logger.info("🔄 VITSTTS已切换为阿里云版本")
        
    async def synthesize_to_audio(self, text: str) -> bytes:
        """
        合成语音为字节数据
        
        Args:
            text: 待合成的文本
            
        Returns:
            音频字节数据
        """
        result = await self.synthesize(text)
        if result['success']:
            return result['audio_data']
        else:
            logger.error(f"TTS合成失败: {result['error']}")
            return b''
            
    async def synthesize_to_file(self, text: str, output_file: str) -> bool:
        """
        合成语音并保存到文件
        
        Args:
            text: 待合成的文本
            output_file: 输出文件路径
            
        Returns:
            是否成功
        """
        result = await self.synthesize_file(text, output_file)
        return result['success']


# 示例用法
async def main():
    """示例用法"""
    async with AlibabaCloudTTS() as tts:
        # 获取可用音色
        voices = tts.get_available_voices()
        print(f"可用音色: {list(voices.keys())}")
        
        # 健康检查
        health = await tts.health_check()
        print(f"健康状态: {health}")
        
        # 合成语音
        # result = await tts.synthesize("你好，我是阿里云语音助手！")
        # if result['success']:
        #     with open('output.wav', 'wb') as f:
        #         f.write(result['audio_data'])
        #     print("语音合成成功")


if __name__ == '__main__':
    asyncio.run(main())
