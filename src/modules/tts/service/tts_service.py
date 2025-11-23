"""
TTS服务主入口

整合RESTful API和WebSocket接口的统一服务入口。
"""

import asyncio
import logging
from typing import Optional
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
import uvicorn

from .api.tts_api import TTSAPIServer
from .websocket.websocket_server import TTSWebSocketServer
from .utils.cache_manager import TTSCacheManager
from .utils.performance_monitor import PerformanceMonitor
from .utils.voice_loader import VoiceLoader

logger = logging.getLogger(__name__)


class TTSService:
    """TTS服务主类"""

    def __init__(self, tts_system, voice_manager, config: Optional[dict] = None):
        """
        初始化TTS服务

        Args:
            tts_system: TTS系统实例
            voice_manager: 音色管理器实例
            config: 配置参数
        """
        self.tts_system = tts_system
        self.voice_manager = voice_manager
        self.config = config or {}

        # 初始化组件
        self.cache_manager = TTSCacheManager(
            max_size=self.config.get('cache_max_size', 1000),
            ttl=self.config.get('cache_ttl', 3600)
        )
        self.performance_monitor = PerformanceMonitor()
        self.voice_loader = VoiceLoader(
            max_workers=self.config.get('voice_loader_workers', 4)
        )

        # 初始化API服务器
        self.api_server = TTSAPIServer(tts_system, voice_manager)
        self.app = self.api_server.get_app()

        # 添加CORS中间件
        self.app.add_middleware(
            CORSMiddleware,
            allow_origins=self.config.get('cors_allow_origins', ["*"]),
            allow_credentials=True,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        # 初始化WebSocket服务器
        self.websocket_server = TTSWebSocketServer(
            tts_system, voice_manager, self.cache_manager
        )

        # 设置WebSocket路由
        self._setup_websocket_routes()

        # 预加载常用音色
        self._preload_common_voices()

    def _setup_websocket_routes(self) -> None:
        """设置WebSocket路由"""
        @self.app.websocket("/ws/v1/tts/stream")
        async def websocket_endpoint(websocket: WebSocket):
            """WebSocket端点"""
            connection_id = f"conn-{id(websocket)}"
            await self.websocket_server.handle_connection(websocket, connection_id)

    def _preload_common_voices(self) -> None:
        """预加载常用音色"""
        common_voices = self.config.get('common_voices', ['default'])
        logger.info(f"预加载音色: {common_voices}")

        # 在后台任务中预加载
        asyncio.create_task(
            self._async_preload_voices(common_voices)
        )

    async def _async_preload_voices(self, voice_ids: list):
        """异步预加载音色"""
        results = self.voice_loader.preload_voices(voice_ids, self.voice_manager)
        logger.info(f"音色预加载结果: {results}")

    async def start(
        self,
        host: str = "0.0.0.0",
        port: int = 8000,
        reload: bool = False
    ):
        """
        启动TTS服务

        Args:
            host: 主机地址
            port: 端口号
            reload: 是否自动重载
        """
        logger.info(f"启动TTS服务: http://{host}:{port}")
        logger.info(f"WebSocket端点: ws://{host}:{port}/ws/v1/tts/stream")

        # 启动服务器
        config = uvicorn.Config(
            self.app,
            host=host,
            port=port,
            reload=reload,
            log_level="info"
        )
        server = uvicorn.Server(config)
        await server.serve()

    def get_stats(self) -> dict:
        """获取服务统计信息"""
        api_stats = self.performance_monitor.get_stats()
        ws_stats = self.websocket_server.get_performance_stats()
        cache_stats = self.cache_manager.get_cache_stats()

        return {
            'api': api_stats,
            'websocket': ws_stats,
            'cache': cache_stats,
            'loaded_voices': self.voice_loader.get_loaded_voices(),
            'active_websocket_connections': self.websocket_server.get_connection_count()
        }

    def shutdown(self):
        """关闭服务"""
        logger.info("关闭TTS服务...")
        self.voice_loader.shutdown()
