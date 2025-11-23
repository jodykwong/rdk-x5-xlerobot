"""
文档语音播放器
==============

将文档内容转换为粤语音频并播放。

作者: Dev Agent
"""

import os
import re
import logging
import time
from typing import List, Optional
from pathlib import Path

# 尝试导入TTS模块
try:
    from .engine.universal_aliyun_client import UniversalAliyunTTSClient
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False


class DocumentPlayer:
    """文档语音播放器"""

    def __init__(self, tts_config: Optional[dict] = None):
        """
        初始化播放器

        Args:
            tts_config: TTS配置
        """
        self.logger = logging.getLogger(__name__)
        self.tts_config = tts_config or {}

        # 初始化TTS客户端
        self.tts_client = None
        if TTS_AVAILABLE:
            try:
                self.tts_client = UniversalAliyunTTSClient(tts_config)
                self.logger.info("✓ TTS客户端初始化成功")
            except Exception as e:
                self.logger.error(f"✗ TTS客户端初始化失败: {e}")
        else:
            self.logger.warning("⚠️ TTS模块不可用")

    def read_document(self, file_path: str) -> str:
        """
        读取文档内容

        Args:
            file_path: 文档路径

        Returns:
            文档文本内容
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # 清理markdown格式
            content = self._clean_markdown(content)

            self.logger.info(f"✓ 成功读取文档: {file_path} ({len(content)} 字符)")
            return content

        except Exception as e:
            self.logger.error(f"✗ 读取文档失败: {e}")
            return ""

    def _clean_markdown(self, text: str) -> str:
        """清理markdown格式"""
        # 移除标题标记
        text = re.sub(r'^#+\s*', '', text, flags=re.MULTILINE)

        # 移除加粗、斜体
        text = re.sub(r'\*\*(.*?)\*\*', r'\1', text)
        text = re.sub(r'\*(.*?)\*', r'\1', text)

        # 移除链接
        text = re.sub(r'\[(.*?)\]\(.*?\)', r'\1', text)

        # 移除表格标记
        text = re.sub(r'\|', ' ', text)
        text = re.sub(r'^[\|\-\:\s]+$', '', text, flags=re.MULTILINE)

        # 移除代码块
        text = re.sub(r'```.*?```', '', text, flags=re.DOTALL)

        # 移除HTML标签
        text = re.sub(r'<[^>]+>', '', text)

        # 移除多余空行
        text = re.sub(r'\n\s*\n', '\n', text)

        # 移除行首尾空格
        text = '\n'.join(line.strip() for line in text.split('\n'))

        return text.strip()

    def split_text(self, text: str, max_length: int = 100) -> List[str]:
        """
        将文本分割成段落

        Args:
            text: 输入文本
            max_length: 最大段落长度

        Returns:
            段落列表
        """
        # 按句子分割
        sentences = re.split(r'[。！？]', text)

        paragraphs = []
        current_paragraph = ""

        for sentence in sentences:
            sentence = sentence.strip()
            if not sentence:
                continue

            # 如果加上这句话不超过最大长度
            if len(current_paragraph + sentence) <= max_length:
                current_paragraph += sentence + "。"
            else:
                # 保存当前段落
                if current_paragraph:
                    paragraphs.append(current_paragraph.strip())

                # 开始新段落
                current_paragraph = sentence + "。"

        # 添加最后一个段落
        if current_paragraph:
            paragraphs.append(current_paragraph.strip())

        return paragraphs

    def synthesize_paragraph(self, paragraph: str, output_file: str, **kwargs) -> bool:
        """
        合成单个段落

        Args:
            paragraph: 段落文本
            output_file: 输出文件路径
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        if not self.tts_client:
            self.logger.error("TTS客户端未初始化")
            return False

        try:
            return self.tts_client.synthesize_to_file(paragraph, output_file, **kwargs)
        except Exception as e:
            self.logger.error(f"合成段落失败: {e}")
            return False

    def play_document(self, file_path: str, output_dir: str = "/tmp/tts_audio", **kwargs) -> List[str]:
        """
        播放整个文档

        Args:
            file_path: 文档路径
            output_dir: 输出目录
            **kwargs: 合成参数

        Returns:
            生成的音频文件列表
        """
        # 读取文档
        content = self.read_document(file_path)
        if not content:
            return []

        # 分割文本
        paragraphs = self.split_text(content, max_length=kwargs.get('max_length', 100))

        # 创建输出目录
        os.makedirs(output_dir, exist_ok=True)

        # 合成音频
        audio_files = []
        for i, paragraph in enumerate(paragraphs, 1):
            if not paragraph.strip():
                continue

            output_file = os.path.join(output_dir, f"segment_{i:03d}.wav")

            self.logger.info(f"合成段落 {i}/{len(paragraphs)}: {paragraph[:50]}...")

            if self.synthesize_paragraph(paragraph, output_file, **kwargs):
                audio_files.append(output_file)
                self.logger.info(f"✓ 段落 {i} 合成完成: {output_file}")
            else:
                self.logger.error(f"✗ 段落 {i} 合成失败")

        return audio_files

    def play_and_concatenate(self, file_path: str, output_file: str = "/tmp/full_document.wav", **kwargs) -> bool:
        """
        播放文档并合并为一个文件

        Args:
            file_path: 文档路径
            output_file: 输出文件路径
            **kwargs: 合成参数

        Returns:
            是否成功
        """
        # 播放文档
        audio_files = self.play_document(file_path, **kwargs)

        if not audio_files:
            self.logger.error("没有生成任何音频文件")
            return False

        # 合并音频文件（使用ffmpeg或简单拼接）
        try:
            import subprocess

            # 使用ffmpeg合并音频
            concat_file = "/tmp/concat_list.txt"
            with open(concat_file, 'w') as f:
                for audio_file in audio_files:
                    f.write(f"file '{audio_file}'\n")

            cmd = [
                'ffmpeg', '-f', 'concat', '-safe', '0', '-i', concat_file,
                '-c', 'copy', output_file, '-y'
            ]

            result = subprocess.run(cmd, capture_output=True, text=True)

            if result.returncode == 0:
                self.logger.info(f"✓ 音频合并完成: {output_file}")
                return True
            else:
                self.logger.error(f"合并音频失败: {result.stderr}")
                return False

        except Exception as e:
            self.logger.error(f"合并音频异常: {e}")
            return False

    def play_with_pauses(self, file_path: str, output_dir: str = "/tmp/tts_audio", **kwargs) -> List[str]:
        """
        播放文档，带暂停

        Args:
            file_path: 文档路径
            output_dir: 输出目录
            **kwargs: 合成参数

        Returns:
            生成的音频文件列表
        """
        audio_files = self.play_document(file_path, output_dir, **kwargs)

        if not audio_files:
            return []

        self.logger.info(f"✓ 文档播放完成，共 {len(audio_files)} 个段落")

        # 播放音频文件（使用aplay或其他播放器）
        try:
            for audio_file in audio_files:
                self.logger.info(f"播放: {os.path.basename(audio_file)}")
                os.system(f"aplay -q {audio_file}")

                # 在段落之间暂停
                if audio_file != audio_files[-1]:
                    time.sleep(1)  # 1秒暂停

        except Exception as e:
            self.logger.error(f"播放音频失败: {e}")

        return audio_files


# 便利函数
def play_document_tts(file_path: str, output_dir: str = "/tmp/tts_audio", **kwargs) -> List[str]:
    """
    便利函数：播放文档

    Args:
        file_path: 文档路径
        output_dir: 输出目录
        **kwargs: 合成参数

    Returns:
        生成的音频文件列表
    """
    player = DocumentPlayer()
    return player.play_document(file_path, output_dir, **kwargs)


def play_full_document(file_path: str, output_file: str = "/tmp/full_document.wav", **kwargs) -> bool:
    """
    便利函数：播放完整文档

    Args:
        file_path: 文档路径
        output_file: 输出文件路径
        **kwargs: 合成参数

    Returns:
        是否成功
    """
    player = DocumentPlayer()
    return player.play_and_concatenate(file_path, output_file, **kwargs)


if __name__ == "__main__":
    # 测试
    import sys

    # 配置日志
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

    # 文档路径
    doc_path = "/home/sunrise/xlerobot/docs/bmm-product-brief-zh.md"

    if not os.path.exists(doc_path):
        print(f"文档不存在: {doc_path}")
        sys.exit(1)

    # 创建播放器
    player = DocumentPlayer()

    # 播放文档
    print(f"\n🎵 开始播放文档: {doc_path}")
    audio_files = player.play_with_pauses(doc_path, max_length=80)

    if audio_files:
        print(f"\n✅ 播放完成，共生成 {len(audio_files)} 个音频文件")
        for i, file in enumerate(audio_files, 1):
            print(f"  {i}. {file}")
    else:
        print("\n❌ 播放失败")
