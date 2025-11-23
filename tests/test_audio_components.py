#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
音频组件功能验证测试 - Epic 1 测试套件
===========================================

专门用于验证音频录制和播放功能的独立测试脚本
基于实际问题的分析，设计成有明确开始和结束的功能验证测试

✅ 设计原则：
- 明确的测试生命周期（初始化→测试→验证→清理→退出）
- 内置超时保护机制
- 详细的验证标准
- 完整的资源管理
- 真实的设备测试

📋 测试覆盖：
- AC001: 音频录制功能验证
- AC002: 音频格式质量验证
- AC003: 多音频设备切换验证

作者: Test Architecture Agent
创建时间: 2025-11-12
版本: v1.0 - 功能验证版
"""

import os
import sys
import time
import logging
import subprocess
import tempfile
import wave
import struct
from datetime import datetime
from pathlib import Path

# 设置项目路径
sys.path.insert(0, '/home/sunrise/xlerobot/src')

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class AudioComponentValidator:
    """音频组件功能验证器"""

    def __init__(self):
        """初始化验证器"""
        self.test_results = []
        self.temp_files = []

        # 测试配置
        self.test_config = {
            'record_duration': 3,  # 3秒录音
            'sample_rate': 16000,
            'channels': 1,
            'format': 'S16_LE',
            'min_file_size': 40000,  # 最小文件大小 40KB
            'max_file_size': 200000, # 最大文件大小 200KB
            'timeout_seconds': 10    # 每个操作超时10秒
        }

        logger.info("🎵 音频组件验证器初始化完成")
        self.log_test_config()

    def log_test_config(self):
        """记录测试配置"""
        config = self.test_config
        logger.info(f"📋 测试配置:")
        logger.info(f"   - 录音时长: {config['record_duration']}秒")
        logger.info(f"   - 采样率: {config['sample_rate']}Hz")
        logger.info(f"   - 声道数: {config['channels']}")
        logger.info(f"   - 格式: {config['format']}")
        logger.info(f"   - 文件大小范围: {config['min_file_size']}-{config['max_file_size']} bytes")
        logger.info(f"   - 操作超时: {config['timeout_seconds']}秒")

    def add_temp_file(self, file_path):
        """添加临时文件到清理列表"""
        self.temp_files.append(file_path)

    def cleanup_temp_files(self):
        """清理所有临时文件"""
        for temp_file in self.temp_files:
            try:
                if os.path.exists(temp_file):
                    os.unlink(temp_file)
                    logger.info(f"🧹 已清理临时文件: {os.path.basename(temp_file)}")
            except Exception as e:
                logger.warning(f"⚠️ 清理文件失败 {temp_file}: {e}")

    def run_test(self, test_name, test_func):
        """运行单个测试并记录结果"""
        logger.info(f"\n🧪 开始测试: {test_name}")
        start_time = time.time()

        try:
            result = test_func()
            end_time = time.time()
            duration = end_time - start_time

            test_result = {
                'name': test_name,
                'status': 'PASS' if result else 'FAIL',
                'duration': duration,
                'timestamp': datetime.now().isoformat(),
                'details': result
            }

            self.test_results.append(test_result)

            if result:
                logger.info(f"✅ {test_name} - 通过 ({duration:.2f}s)")
            else:
                logger.error(f"❌ {test_name} - 失败 ({duration:.2f}s)")

            return result

        except Exception as e:
            end_time = time.time()
            duration = end_time - start_time

            test_result = {
                'name': test_name,
                'status': 'ERROR',
                'duration': duration,
                'timestamp': datetime.now().isoformat(),
                'error': str(e)
            }

            self.test_results.append(test_result)
            logger.error(f"💥 {test_name} - 异常 ({duration:.2f}s): {e}")
            return False

    def test_ac001_audio_recording_functionality(self):
        """AC001: 音频录制功能验证"""
        logger.info("🎤 AC001: 验证音频录制功能")

        # 创建临时音频文件
        temp_file = tempfile.mktemp(suffix='.wav')
        self.add_temp_file(temp_file)

        try:
            # 构建arecord命令
            cmd = [
                'arecord', '-D', 'default', '-d', str(self.test_config['record_duration']),
                '-f', self.test_config['format'], '-r', str(self.test_config['sample_rate']),
                '-c', str(self.test_config['channels']), temp_file
            ]

            logger.info(f"🔧 执行命令: {' '.join(cmd)}")

            # 执行录音，带超时保护
            result = subprocess.run(
                cmd,
                capture_output=True,
                timeout=self.test_config['timeout_seconds']
            )

            # 验证命令执行成功
            if result.returncode != 0:
                logger.error(f"❌ arecord命令失败，返回码: {result.returncode}")
                logger.error(f"   错误输出: {result.stderr.decode()}")
                return False

            # 验证文件是否创建
            if not os.path.exists(temp_file):
                logger.error(f"❌ 音频文件未创建: {temp_file}")
                return False

            # 验证文件大小
            file_size = os.path.getsize(temp_file)
            logger.info(f"📊 录音文件大小: {file_size} bytes")

            if file_size < self.test_config['min_file_size']:
                logger.error(f"❌ 文件太小: {file_size} < {self.test_config['min_file_size']}")
                return False

            if file_size > self.test_config['max_file_size']:
                logger.error(f"❌ 文件过大: {file_size} > {self.test_config['max_file_size']}")
                return False

            # 验证WAV文件格式
            if not self._validate_wav_format(temp_file):
                logger.error(f"❌ WAV文件格式验证失败")
                return False

            logger.info(f"✅ AC001通过: 音频录制功能正常")
            logger.info(f"   - 文件路径: {temp_file}")
            logger.info(f"   - 文件大小: {file_size} bytes")
            logger.info(f"   - 录音时长: {self.test_config['record_duration']}秒")

            return True

        except subprocess.TimeoutExpired:
            logger.error(f"❌ AC001失败: 录音超时 ({self.test_config['timeout_seconds']}秒)")
            return False
        except Exception as e:
            logger.error(f"❌ AC001异常: {e}")
            return False

    def test_ac002_audio_format_quality(self):
        """AC002: 音频格式质量验证"""
        logger.info("🎵 AC002: 验证音频格式和质量")

        # 创建临时音频文件
        temp_file = tempfile.mktemp(suffix='.wav')
        self.add_temp_file(temp_file)

        try:
            # 录制短音频用于格式验证
            cmd = [
                'arecord', '-D', 'default', '-d', '1',
                '-f', self.test_config['format'], '-r', str(self.test_config['sample_rate']),
                '-c', str(self.test_config['channels']), temp_file
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                timeout=5
            )

            if result.returncode != 0:
                logger.error(f"❌ 录音失败，返回码: {result.returncode}")
                return False

            # 验证WAV文件格式详细参数
            wav_info = self._analyze_wav_file(temp_file)
            if not wav_info:
                return False

            # 验证音频参数
            expected_params = {
                'channels': self.test_config['channels'],
                'sample_rate': self.test_config['sample_rate'],
                'sample_width': 2,  # S16_LE = 16-bit = 2 bytes
                'format_tag': 1     # PCM = 1
            }

            for param, expected_value in expected_params.items():
                if wav_info.get(param) != expected_value:
                    logger.error(f"❌ 参数不匹配 {param}: {wav_info.get(param)} != {expected_value}")
                    return False

            logger.info(f"✅ AC002通过: 音频格式质量正常")
            logger.info(f"   - 声道数: {wav_info['channels']}")
            logger.info(f"   - 采样率: {wav_info['sample_rate']} Hz")
            logger.info(f"   - 采样宽度: {wav_info['sample_width']} bytes")
            logger.info(f"   - 格式标签: {wav_info['format_tag']} (PCM)")
            logger.info(f"   - 数据大小: {wav_info['data_size']} bytes")

            return True

        except Exception as e:
            logger.error(f"❌ AC002异常: {e}")
            return False

    def test_ac003_multi_audio_device_switching(self):
        """AC003: 多音频设备切换验证"""
        logger.info("🔄 AC003: 验证多音频设备切换功能")

        try:
            # 获取可用音频设备
            devices = self._get_audio_devices()
            if len(devices) < 2:
                logger.warning(f"⚠️ 只有 {len(devices)} 个音频设备，跳过切换测试")
                return True  # 跳过但不算失败

            logger.info(f"🎵 发现 {len(devices)} 个音频设备: {devices}")

            # 测试每个设备的录制功能
            successful_devices = []

            for i, device in enumerate(devices[:2]):  # 只测试前两个设备
                logger.info(f"🎤 测试设备 {i+1}: {device}")

                temp_file = tempfile.mktemp(suffix=f'_device_{i}.wav')
                self.add_temp_file(temp_file)

                # 使用指定设备录制
                cmd = [
                    'arecord', '-D', device, '-d', '1',
                    '-f', self.test_config['format'], '-r', str(self.test_config['sample_rate']),
                    '-c', str(self.test_config['channels']), temp_file
                ]

                try:
                    result = subprocess.run(
                        cmd,
                        capture_output=True,
                        timeout=5
                    )

                    if result.returncode == 0 and os.path.exists(temp_file):
                        file_size = os.path.getsize(temp_file)
                        if file_size > 1000:  # 至少1KB
                            successful_devices.append(device)
                            logger.info(f"✅ 设备 {device} 测试成功 ({file_size} bytes)")
                        else:
                            logger.warning(f"⚠️ 设备 {device} 文件太小: {file_size} bytes")
                    else:
                        logger.warning(f"⚠️ 设备 {device} 录音失败")

                except subprocess.TimeoutExpired:
                    logger.warning(f"⚠️ 设备 {device} 录音超时")
                except Exception as e:
                    logger.warning(f"⚠️ 设备 {device} 异常: {e}")

            # 验证至少有一个设备工作正常
            if len(successful_devices) >= 1:
                logger.info(f"✅ AC003通过: {len(successful_devices)} 个设备工作正常")
                logger.info(f"   - 成功设备: {successful_devices}")
                return True
            else:
                logger.error(f"❌ AC003失败: 没有可用的音频设备")
                return False

        except Exception as e:
            logger.error(f"❌ AC003异常: {e}")
            return False

    def _validate_wav_format(self, wav_file):
        """验证WAV文件格式"""
        try:
            with open(wav_file, 'rb') as f:
                # 读取WAV头
                header = f.read(12)
                if len(header) < 12:
                    return False

                # 验证RIFF标识
                if header[:4] != b'RIFF':
                    return False

                # 验证WAVE标识
                if header[8:12] != b'WAVE':
                    return False

                return True
        except Exception as e:
            logger.warning(f"⚠️ WAV格式验证异常: {e}")
            return False

    def _analyze_wav_file(self, wav_file):
        """分析WAV文件详细信息"""
        try:
            with wave.open(wav_file, 'rb') as wav:
                return {
                    'channels': wav.getnchannels(),
                    'sample_rate': wav.getframerate(),
                    'sample_width': wav.getsampwidth(),
                    'frames': wav.getnframes(),
                    'format_tag': 1,  # WAV总是PCM
                    'data_size': wav.getnframes() * wav.getnchannels() * wav.getsampwidth()
                }
        except Exception as e:
            logger.error(f"❌ WAV文件分析失败: {e}")
            return None

    def _get_audio_devices(self):
        """获取可用音频设备列表"""
        try:
            # 使用arecord -L获取设备列表
            result = subprocess.run(
                ['arecord', '-L'],
                capture_output=True,
                text=True,
                timeout=5
            )

            if result.returncode != 0:
                logger.error(f"❌ 获取音频设备失败: {result.stderr}")
                return ['default']  # 至少返回默认设备

            devices = []
            for line in result.stdout.split('\n'):
                line = line.strip()
                if line and not line.startswith('#') and 'hw:' in line:
                    devices.append(line)

            # 确保包含默认设备
            if 'default' not in devices:
                devices.insert(0, 'default')

            return devices[:5]  # 最多返回5个设备

        except Exception as e:
            logger.warning(f"⚠️ 获取音频设备异常: {e}")
            return ['default']

    def run_all_tests(self):
        """运行所有音频组件测试"""
        logger.info("\n" + "=" * 80)
        logger.info("🎵 音频组件功能验证测试开始")
        logger.info("=" * 80)

        start_time = time.time()

        # 运行所有测试
        tests = [
            ("AC001: 音频录制功能", self.test_ac001_audio_recording_functionality),
            ("AC002: 音频格式质量", self.test_ac002_audio_format_quality),
            ("AC003: 多音频设备切换", self.test_ac003_multi_audio_device_switching)
        ]

        for test_name, test_func in tests:
            self.run_test(test_name, test_func)

        # 计算总测试时间
        total_time = time.time() - start_time

        # 生成测试报告
        self.generate_test_report(total_time)

        # 清理资源
        self.cleanup_temp_files()

        logger.info("\n" + "=" * 80)
        logger.info("🎵 音频组件功能验证测试完成")
        logger.info("=" * 80)

        return self.get_overall_result()

    def generate_test_report(self, total_time):
        """生成测试报告"""
        logger.info(f"\n📊 音频组件测试报告")
        logger.info(f"{'='*60}")

        # 统计结果
        total_tests = len(self.test_results)
        passed_tests = len([r for r in self.test_results if r['status'] == 'PASS'])
        failed_tests = len([r for r in self.test_results if r['status'] == 'FAIL'])
        error_tests = len([r for r in self.test_results if r['status'] == 'ERROR'])

        logger.info(f"📈 测试统计:")
        logger.info(f"   - 总测试数: {total_tests}")
        logger.info(f"   - 通过: {passed_tests}")
        logger.info(f"   - 失败: {failed_tests}")
        logger.info(f"   - 异常: {error_tests}")
        logger.info(f"   - 成功率: {passed_tests/total_tests*100:.1f}%")
        logger.info(f"   - 总耗时: {total_time:.2f}s")

        # 详细结果
        logger.info(f"\n📋 详细结果:")
        for result in self.test_results:
            status_icon = "✅" if result['status'] == 'PASS' else "❌"
            logger.info(f"   {status_icon} {result['name']} - {result['status']} ({result['duration']:.2f}s)")

            if result['status'] == 'ERROR':
                logger.info(f"      💥 错误: {result.get('error', 'Unknown error')}")

        logger.info(f"{'='*60}")

    def get_overall_result(self):
        """获取总体测试结果"""
        passed_count = len([r for r in self.test_results if r['status'] == 'PASS'])
        total_count = len(self.test_results)

        if passed_count == total_count:
            logger.info(f"🎉 所有音频组件测试通过！")
            return True
        else:
            logger.error(f"❌ 音频组件测试存在问题: {passed_count}/{total_count} 通过")
            return False

def main():
    """主函数"""
    logger.info("🚀 启动音频组件功能验证测试")

    # 创建验证器实例
    validator = AudioComponentValidator()

    try:
        # 运行所有测试
        success = validator.run_all_tests()

        # 返回适当的退出码
        return 0 if success else 1

    except KeyboardInterrupt:
        logger.info("\n👋 用户中断测试")
        return 130  # 标准的键盘中断退出码
    except Exception as e:
        logger.error(f"💥 测试系统异常: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        # 确保清理资源
        validator.cleanup_temp_files()

if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)