# XleRobot多模态在线对话API集成
# Story 1.7: 多模态在线对话API集成
# 严格遵循Epic 1纯在线架构原则

__version__ = "1.0.0"
__author__ = "XleRobot Developer Team"
__description__ = "纯在线多模态对话API集成，100%基于阿里云服务"

from .online_dialogue_api import OnlineDialogueAPI
from .simple_session_manager import SimpleSessionManager
from .cantonese_text_processor import CantoneseTextProcessor

__all__ = [
    'OnlineDialogueAPI',
    'SimpleSessionManager',
    'CantoneseTextProcessor'
]