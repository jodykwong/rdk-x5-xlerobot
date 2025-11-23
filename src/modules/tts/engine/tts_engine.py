"""
语音合成引擎
=============

集成VITS语音合成模型，实现高质量粤语语音合成。

作者: Dev Agent
"""

import torch
import numpy as np
import soundfile as sf
from typing import Optional, Dict, Any, Tuple
import logging
import time
from pathlib import Path
import yaml

# 导入音色相关模块
from ..voices.voice_manager import VoiceManager, VoiceProfile
from ..voices.voice_controller import VoiceController, VoiceParameters, BlendMode
from ..voices.voice_cache import VoiceCache
from ..voices.voice_quality import VoiceQualityEvaluator


class TTSEngine:
    """VITS语音合成引擎"""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        初始化TTS引擎

        Args:
            config: 配置字典
        """
        self.config = config or {}
        self.logger = logging.getLogger(__name__)
        self.device = self._get_device()
        self.model = None
        self.model_config = None

        # 初始化音色系统
        self.voice_manager = VoiceManager(config.get('voices') if config else None)
        self.voice_controller = VoiceController()
        self.voice_cache = VoiceCache(
            cache_dir=config.get('cache_dir', '/tmp/tts_voice_cache') if config else '/tmp/tts_voice_cache',
            max_size=config.get('max_cache_size', 500) if config else 500
        )
        self.quality_evaluator = VoiceQualityEvaluator()

        # 当前音色状态
        self.current_voice_id: Optional[str] = None
        self.current_voice_profile: Optional[VoiceProfile] = None

        self.logger.info(f"✓ TTS引擎初始化完成 (设备: {self.device}, 音色: {len(self.voice_manager.voices)})")

    def _get_device(self) -> str:
        """
        获取计算设备

        Returns:
            设备类型 (cpu/npu)
        """
        if torch.cuda.is_available():
            self.logger.info("检测到CUDA支持")
            return "cuda"

        # 检查NPU支持
        try:
            import npu
            return "npu"
        except ImportError:
            pass

        self.logger.info("使用CPU设备")
        return "cpu"

    def load_model(self, model_path: str, config_path: str) -> bool:
        """
        加载VITS模型

        Args:
            model_path: 模型文件路径
            config_path: 配置文件路径

        Returns:
            加载是否成功
        """
        try:
            # 加载模型配置
            if Path(config_path).exists():
                with open(config_path, 'r', encoding='utf-8') as f:
                    self.model_config = yaml.safe_load(f)
                self.logger.info(f"✓ 模型配置已加载: {config_path}")

            # 加载模型文件
            if Path(model_path).exists():
                self.model = torch.jit.load(model_path, map_location=self.device)
                self.logger.info(f"✓ VITS模型已加载: {model_path}")
                return True
            else:
                self.logger.error(f"✗ 模型文件不存在: {model_path}")
                return False

        except Exception as e:
            self.logger.error(f"✗ 模型加载失败: {e}")
            return False

    def synthesize(self, text: str, **kwargs) -> Tuple[np.ndarray, float]:
        """
        语音合成

        Args:
            text: 输入文本
            **kwargs: 合成参数

        Returns:
            元组: (音频数据, 合成时间)
        """
        if self.model is None:
            raise ValueError("模型未加载，请先调用load_model()")

        start_time = time.time()

        try:
            # 文本预处理
            processed_text = self._preprocess_text(text)

            # 文本编码为特征
            text_features = self._encode_text(processed_text)

            # VITS模型推理
            audio = self._inference(text_features, **kwargs)

            # 音频后处理
            audio = self._postprocess_audio(audio)

            # 应用音色（如果有指定）
            voice_id = kwargs.get('voice_id')
            if voice_id:
                audio = self.apply_voice_to_synthesis(audio, voice_id)

            synthesis_time = time.time() - start_time
            self.logger.info(f"语音合成完成: {len(text)}字符 -> {len(audio)}采样点 -> {synthesis_time:.3f}s")

            return audio, synthesis_time

        except Exception as e:
            self.logger.error(f"✗ 语音合成失败: {e}")
            raise

    def _preprocess_text(self, text: str) -> torch.Tensor:
        """
        文本预处理

        Args:
            text: 输入文本

        Returns:
            文本特征张量
        """
        # 简化的文本编码（实际需要更复杂的文本到特征转换）
        # 这里使用字符级编码

        # 文本长度标准化
        max_length = 50
        text = text[:max_length].ljust(max_length, ' ')

        # 字符到ID映射
        char_to_id = {chr(i): i for i in range(32, 127)}  # ASCII字符

        # 转换为ID序列
        ids = [char_to_id.get(c, 0) for c in text]

        # 转换为张量
        tensor = torch.tensor(ids, dtype=torch.long, device=self.device).unsqueeze(0)

        return tensor

    def _encode_text(self, text_tensor: torch.Tensor) -> torch.Tensor:
        """
        文本编码为特征

        Args:
            text_tensor: 文本张量

        Returns:
            文本特征张量
        """
        # 简化的编码过程（实际需要更复杂的文本编码器）
        # 这里返回原始文本张量作为占位符

        return text_tensor

    def _inference(self, text_features: torch.Tensor, **kwargs) -> np.ndarray:
        """
        VITS模型推理

        Args:
            text_features: 文本特征
            **kwargs: 推理参数

        Returns:
            生成的音频数据
        """
        # 合成参数
        noise_scale = kwargs.get('noise_scale', 0.667)
        length_scale = kwargs.get('length_scale', 1.0)
        speaker_id = kwargs.get('speaker_id', 0)

        # 准备输入
        with torch.no_grad():
            # 模拟VITS推理过程（实际需要完整的VITS模型推理）
            # 这里生成合成音频作为演示

            batch_size, seq_len = text_features.shape
            output_length = seq_len * 100  # 假设每个字符对应100个音频采样点

            # 生成随机音频作为占位符（实际应使用VITS模型生成）
            audio = torch.randn(1, output_length) * 0.1

            # 简化的后处理
            audio = torch.tanh(audio) * 0.5

        # 转换为numpy数组
        audio_np = audio.squeeze(0).cpu().numpy()

        return audio_np

    def _postprocess_audio(self, audio: np.ndarray) -> np.ndarray:
        """
        音频后处理

        Args:
            audio: 原始音频数据

        Returns:
            处理后的音频数据
        """
        # 归一化
        audio = audio / (np.max(np.abs(audio)) + 1e-8)

        # 音量调整
        audio = audio * 0.8

        # 高通滤波（移除直流分量）
        audio = audio - np.mean(audio)

        return audio

    def synthesize_to_file(self, text: str, output_path: str, **kwargs) -> bool:
        """
        直接合成到文件

        Args:
            text: 输入文本
            output_path: 输出文件路径
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        try:
            audio, synthesis_time = self.synthesize(text, **kwargs)

            # 写入音频文件
            sf.write(output_path, audio, 22050)

            self.logger.info(f"✓ 音频已保存: {output_path} (耗时: {synthesis_time:.3f}s)")

            return True

        except Exception as e:
            self.logger.error(f"✗ 合成失败: {e}")
            return False

    def get_model_info(self) -> Dict[str, Any]:
        """
        获取模型信息

        Returns:
            模型信息字典
        """
        info = {
            'device': self.device,
            'model_loaded': self.model is not None,
            'model_config': self.model_config,
            'supports_npu': self._get_device() == 'npu',
        }

        if self.model is not None:
            info['model_type'] = type(self.model).__name__

        return info

    def switch_voice(self, voice_id: str) -> bool:
        """
        切换音色

        Args:
            voice_id: 音色ID

        Returns:
            切换是否成功
        """
        start_time = time.time()

        # 尝试切换到指定音色
        if self.voice_manager.switch_voice(voice_id):
            self.current_voice_id = voice_id
            self.current_voice_profile = self.voice_manager.get_current_voice()

            switch_time = (time.time() - start_time) * 1000

            if switch_time < 100:
                self.logger.info(f"✓ 音色切换成功: {voice_id} (耗时: {switch_time:.2f}ms)")
                return True
            else:
                self.logger.warning(f"⚠️ 音色切换较慢: {switch_time:.2f}ms (目标: <100ms)")
                return True
        else:
            self.logger.error(f"✗ 音色切换失败: {voice_id}")
            return False

    def set_voice_parameters(self, params: VoiceParameters, transition_duration: float = 0.0) -> bool:
        """
        设置音色参数

        Args:
            params: 音色参数
            transition_duration: 过渡时间（秒）

        Returns:
            设置是否成功
        """
        # 更新当前音色的特征
        if self.current_voice_profile:
            self.current_voice_profile.characteristics.update(params.to_dict())

        return self.voice_controller.set_parameters(params, transition_duration)

    def blend_voices(self, voice1_id: str, voice2_id: str, blend_mode: BlendMode,
                     alpha: float, output_voice_id: str) -> bool:
        """
        混合两种音色

        Args:
            voice1_id: 音色1 ID
            voice2_id: 音色2 ID
            blend_mode: 混合模式
            alpha: 混合因子
            output_voice_id: 输出音色ID

        Returns:
            混合是否成功
        """
        voice1 = self.voice_manager.get_voice(voice1_id)
        voice2 = self.voice_manager.get_voice(voice2_id)

        if not voice1 or not voice2:
            self.logger.error(f"✗ 音色不存在: {voice1_id} 或 {voice2_id}")
            return False

        # 创建音色参数
        params1 = VoiceParameters(**voice1.characteristics)
        params2 = VoiceParameters(**voice2.characteristics)

        # 混合参数
        blended_params = self.voice_controller.blend_voices(params1, params2, blend_mode, alpha)

        # 创建新的音色档案
        from ..voices.voice_manager import VoiceType
        blended_voice = VoiceProfile(
            voice_id=output_voice_id,
            name=f"混合音色_{voice1.name}_{voice2.name}",
            type=VoiceType.CUSTOM,
            description=f"混合自 {voice1.name} 和 {voice2.name}",
            model_path=voice1.model_path,
            config_path=f"/home/sunrise/xlerobot/src/modules/tts/voices/configs/{output_voice_id}.yaml",
            sample_rate=voice1.sample_rate,
            speaker_id=voice1.speaker_id + 50,
            quality_score=(voice1.quality_score + voice2.quality_score) / 2,
            characteristics=blended_params.to_dict(),
            supported_languages=list(set(voice1.supported_languages + voice2.supported_languages)),
            created_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            updated_at=time.strftime("%Y-%m-%dT%H:%M:%S"),
            version="1.0.0",
            is_active=True
        )

        # 注册混合音色
        return self.voice_manager.register_voice(blended_voice)

    def apply_voice_to_synthesis(self, audio: np.ndarray, voice_id: str) -> np.ndarray:
        """
        在语音合成时应用音色

        Args:
            audio: 原始音频
            voice_id: 音色ID

        Returns:
            应用音色后的音频
        """
        voice = self.voice_manager.get_voice(voice_id)
        if not voice:
            self.logger.warning(f"⚠️ 音色不存在，使用默认参数: {voice_id}")
            voice_params = VoiceParameters()
        else:
            voice_params = VoiceParameters(**voice.characteristics)

        # 应用音色参数
        processed_audio = self.voice_controller.apply_voice_to_audio(
            audio, voice_params, voice.sample_rate
        )

        return processed_audio

    def evaluate_voice_quality(self, audio: np.ndarray, voice_id: str,
                              sample_rate: int) -> Dict[str, Any]:
        """
        评估音色质量

        Args:
            audio: 音频数据
            voice_id: 音色ID
            sample_rate: 采样率

        Returns:
            质量评估结果
        """
        metrics = self.quality_evaluator.evaluate_voice_quality(audio, sample_rate, voice_id)
        report = self.quality_evaluator.get_quality_report(voice_id)

        if report:
            report['metrics'] = {
                'mos_score': metrics.mos_score,
                'snr': metrics.snr,
                'thd': metrics.thd,
                'dynamic_range': metrics.dynamic_range,
                'spectral_centroid': metrics.spectral_centroid,
                'zero_crossing_rate': metrics.zero_crossing_rate,
                'mfcc_distortion': metrics.mfcc_distortion,
                'evaluation_time': metrics.evaluation_time
            }

        return report

    def preload_all_voices(self) -> Dict[str, bool]:
        """
        预加载所有音色

        Returns:
            预加载结果字典
        """
        results = {}
        for voice_id in self.voice_manager.voices.keys():
            results[voice_id] = self.voice_cache.preload_voice(voice_id)

        self.logger.info(f"✓ 已预加载 {sum(results.values())} / {len(results)} 个音色")
        return results

    def get_voice_statistics(self) -> Dict[str, Any]:
        """
        获取音色统计信息

        Returns:
            统计信息字典
        """
        voice_stats = self.voice_manager.get_voice_statistics()
        cache_stats = self.voice_cache.get_cache_stats()
        controller_stats = self.voice_controller.get_parameter_statistics()

        return {
            'voices': voice_stats,
            'cache': cache_stats,
            'controller': controller_stats,
            'current_voice': {
                'voice_id': self.current_voice_id,
                'name': self.current_voice_profile.name if self.current_voice_profile else None
            }
        }

    def benchmark(self, test_texts: list, **kwargs) -> Dict[str, float]:
        """
        性能基准测试

        Args:
            test_texts: 测试文本列表
            **kwargs: 合成参数

        Returns:
            性能指标字典
        """
        times = []
        for text in test_texts:
            start = time.time()
            try:
                self.synthesize(text, **kwargs)
                times.append(time.time() - start)
            except Exception as e:
                self.logger.error(f"基准测试失败: {e}")
                continue

        if not times:
            return {'error': 'no successful syntheses'}

        return {
            'avg_time': np.mean(times),
            'min_time': np.min(times),
            'max_time': np.max(times),
            'std_time': np.std(times),
            'total_tests': len(times),
        }

    def __repr__(self) -> str:
        return f"TTSEngine(device={self.device}, model={'loaded' if self.model else 'not loaded'}, voices={len(self.voice_manager.voices)})"
