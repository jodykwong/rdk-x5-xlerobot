"""
TTS RESTful API服务器

基于FastAPI实现的TTS RESTful API接口。
"""

import base64
import uuid
import time
from datetime import datetime
from typing import Dict, List, Optional, Any
from fastapi import FastAPI, HTTPException, BackgroundTasks
from fastapi.responses import JSONResponse
import logging

from ..models.request_models import (
    TTSRequest, TTSStreamRequest, VoiceListRequest, HealthCheckRequest
)
from ..models.response_models import (
    TTSResponse, VoiceListResponse, HealthCheckResponse, ErrorResponse
)
from ..utils.cache_manager import TTSCacheManager
from ..utils.performance_monitor import PerformanceMonitor
from ..utils.voice_loader import VoiceLoader

logger = logging.getLogger(__name__)


class TTSAPIServer:
    """TTS API服务器"""

    def __init__(self, tts_system, voice_manager):
        """
        初始化TTS API服务器

        Args:
            tts_system: TTS系统实例
            voice_manager: 音色管理器实例
        """
        self.tts_system = tts_system
        self.voice_manager = voice_manager
        self.app = FastAPI(
            title="TTS服务API",
            description="提供文本转语音服务的RESTful API",
            version="1.0.0"
        )
        self.cache_manager = TTSCacheManager(max_size=1000, ttl=3600)
        self.performance_monitor = PerformanceMonitor()
        self.voice_loader = VoiceLoader(max_workers=4)

        self._setup_routes()
        self._setup_exception_handlers()

    def _setup_routes(self) -> None:
        """设置API路由"""
        # 健康检查
        @self.app.get("/api/v1/tts/health", response_model=HealthCheckResponse)
        async def health_check(detailed: bool = False):
            """健康检查接口"""
            request_id = str(uuid.uuid4())
            start_time = time.time()

            try:
                # 获取基本状态
                health_status = self.performance_monitor.get_health_status()
                stats = self.performance_monitor.get_stats()

                response = HealthCheckResponse(
                    status=health_status['status'],
                    version="1.0.0",
                    timestamp=datetime.now(),
                    uptime=stats['uptime_seconds'],
                    request_id=request_id
                )

                if detailed:
                    response.services = {
                        "tts_engine": "healthy",
                        "cache": "healthy",
                        "voice_loader": "healthy"
                    }
                    response.performance = {
                        "avg_response_time": stats['avg_response_time'],
                        "requests_per_second": stats['qps'],
                        "cache_hit_rate": stats['cache_hit_rate_percent'],
                        "active_connections": stats['active_connections']
                    }

                # 记录性能
                processing_time = time.time() - start_time
                self.performance_monitor.record_request(processing_time, True, True)

                return response

            except Exception as e:
                logger.error(f"健康检查失败: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        # 获取音色列表
        @self.app.get("/api/v1/tts/voices", response_model=VoiceListResponse)
        async def get_voices(
            language: Optional[str] = None,
            gender: Optional[str] = None,
            emotion_support: Optional[bool] = None
        ):
            """获取音色列表"""
            request_id = str(uuid.uuid4())
            start_time = time.time()

            try:
                # 获取所有音色
                voices_data = self.voice_manager.list_voices()

                # 过滤音色
                filtered_voices = []
                for voice in voices_data:
                    if language and voice.get('language') != language:
                        continue
                    if gender and voice.get('gender') != gender:
                        continue
                    if emotion_support is not None and voice.get('emotion_support') != emotion_support:
                        continue
                    filtered_voices.append(voice)

                response = VoiceListResponse(
                    success=True,
                    voices=filtered_voices,
                    total=len(filtered_voices),
                    request_id=request_id
                )

                # 记录性能
                processing_time = time.time() - start_time
                self.performance_monitor.record_request(processing_time, True, True)

                return response

            except Exception as e:
                logger.error(f"获取音色列表失败: {e}")
                raise HTTPException(status_code=500, detail=str(e))

        # 文本转语音
        @self.app.post("/api/v1/tts/synthesize", response_model=TTSResponse)
        async def synthesize_tts(request: TTSRequest):
            """文本转语音接口"""
            request_id = str(uuid.uuid4())
            start_time = time.time()

            try:
                # 生成缓存键
                cache_key = self.cache_manager._generate_key(
                    request.text, request.voice_id,
                    language=request.language,
                    emotion=request.emotion,
                    emotion_intensity=request.emotion_intensity,
                    speed=request.speed,
                    pitch=request.pitch,
                    volume=request.volume,
                    audio_format=request.audio_format,
                    audio_quality=request.audio_quality,
                    sample_rate=request.sample_rate
                )

                # 检查缓存
                cache_result = None
                if request.cache_enabled:
                    cache_result = self.cache_manager.get_audio(
                        request.text, request.voice_id,
                        language=request.language,
                        emotion=request.emotion,
                        emotion_intensity=request.emotion_intensity,
                        speed=request.speed,
                        pitch=request.pitch,
                        volume=request.volume,
                        audio_format=request.audio_format,
                        audio_quality=request.audio_quality,
                        sample_rate=request.sample_rate
                    )

                if cache_result:
                    audio_data, metadata = cache_result
                    audio_base64 = base64.b64encode(audio_data).decode('utf-8')

                    response = TTSResponse(
                        success=True,
                        audio_data=audio_base64,
                        audio_format=request.audio_format,
                        audio_size=len(audio_data),
                        duration=metadata.get('duration'),
                        sample_rate=request.sample_rate,
                        request_id=request_id,
                        processing_time=time.time() - start_time,
                        cache_hit=True,
                        message="从缓存返回"
                    )

                    # 记录性能
                    processing_time = time.time() - start_time
                    self.performance_monitor.record_request(processing_time, True, True)

                    return response

                # 预加载音色
                self.voice_loader.load_voice(request.voice_id, self.voice_manager)

                # 调用TTS系统
                audio_data = await self.tts_system.synthesize(
                    text=request.text,
                    voice_id=request.voice_id,
                    language=request.language,
                    emotion=request.emotion,
                    emotion_intensity=request.emotion_intensity,
                    speed=request.speed,
                    pitch=request.pitch,
                    volume=request.volume,
                    output_format=request.audio_format.value,
                    quality=request.audio_quality.value,
                    sample_rate=request.sample_rate
                )

                # 转换为base64
                audio_base64 = base64.b64encode(audio_data).decode('utf-8')

                # 缓存结果
                if request.cache_enabled:
                    self.cache_manager.set_audio(
                        request.text, request.voice_id, audio_data,
                        language=request.language,
                        emotion=request.emotion,
                        emotion_intensity=request.emotion_intensity,
                        speed=request.speed,
                        pitch=request.pitch,
                        volume=request.volume,
                        audio_format=request.audio_format.value,
                        audio_quality=request.audio_quality.value,
                        sample_rate=request.sample_rate
                    )

                response = TTSResponse(
                    success=True,
                    audio_data=audio_base64,
                    audio_format=request.audio_format,
                    audio_size=len(audio_data),
                    duration=None,
                    sample_rate=request.sample_rate,
                    request_id=request_id,
                    processing_time=time.time() - start_time,
                    cache_hit=False,
                    message="音频合成成功"
                )

                # 记录性能
                processing_time = time.time() - start_time
                self.performance_monitor.record_request(processing_time, True, False)

                return response

            except Exception as e:
                logger.error(f"TTS合成失败: {e}")
                error_response = ErrorResponse(
                    success=False,
                    error_code="TTS_001",
                    error_message=str(e),
                    request_id=request_id
                )
                raise HTTPException(status_code=500, detail=error_response.dict())

        # 流式文本转语音
        @self.app.post("/api/v1/tts/synthesize-stream")
        async def synthesize_tts_stream(request: TTSStreamRequest):
            """流式文本转语音接口"""
            request_id = str(uuid.uuid4())
            start_time = time.time()

            try:
                # 预加载音色
                self.voice_loader.load_voice(request.voice_id, self.voice_manager)

                # 调用TTS系统流式合成
                audio_chunks = await self.tts_system.synthesize_stream(
                    text=request.text,
                    voice_id=request.voice_id,
                    language=request.language,
                    emotion=request.emotion,
                    emotion_intensity=request.emotion_intensity,
                    speed=request.speed,
                    chunk_size=request.chunk_size
                )

                # 记录性能
                processing_time = time.time() - start_time
                self.performance_monitor.record_request(processing_time, True, False)

                return {"request_id": request_id, "chunks": audio_chunks}

            except Exception as e:
                logger.error(f"流式TTS合成失败: {e}")
                error_response = ErrorResponse(
                    success=False,
                    error_code="TTS_002",
                    error_message=str(e),
                    request_id=request_id
                )
                raise HTTPException(status_code=500, detail=error_response.dict())

    def _setup_exception_handlers(self) -> None:
        """设置异常处理器"""
        @self.app.exception_handler(Exception)
        async def global_exception_handler(request, exc):
            """全局异常处理器"""
            logger.error(f"未处理的异常: {exc}", exc_info=True)
            return JSONResponse(
                status_code=500,
                content=ErrorResponse(
                    success=False,
                    error_code="INTERNAL_ERROR",
                    error_message="内部服务器错误",
                    request_id=str(uuid.uuid4())
                ).dict()
            )

    def get_app(self) -> FastAPI:
        """获取FastAPI应用实例"""
        return self.app

    async def start(self, host: str = "0.0.0.0", port: int = 8000, reload: bool = False):
        """
        启动API服务器

        Args:
            host: 主机地址
            port: 端口号
            reload: 是否自动重载
        """
        import uvicorn
        logger.info(f"启动TTS API服务器: http://{host}:{port}")
        await uvicorn.run(
            self.app,
            host=host,
            port=port,
            reload=reload,
            log_level="info"
        )
