#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Epic 1 真实环境严格验证脚本
严格遵守: 严禁使用任何Mock、模拟或硬编码数据
使用真实: 麦克风输入 + 真实算法 + 真实测试 + 扬声器输出
"""

import os
import sys
import time
import subprocess
import tempfile
import json
from pathlib import Path

class RealEpic1Verifier:
    """Epic 1 真实环境验证器 - 严禁Mock数据"""

    def __init__(self):
        """初始化真实验证器"""
        self.project_root = Path(__file__).parent
        self.src_path = self.project_root / "src"
        self.test_results = {}
        self.real_test_data = {}

    def log(self, message: str, level: str = "REAL"):
        """真实验证日志"""
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] {level}: {message}")

    def check_real_audio_hardware(self) -> dict:
        """检查真实音频硬件"""
        self.log("检查真实音频硬件 - 严禁Mock数据")
        result = {"success": False, "real_devices": [], "details": []}

        try:
            # 检查真实音频输入设备
            arecord_result = subprocess.run(['arecord', '-l'],
                                          capture_output=True, text=True, timeout=10)

            if arecord_result.returncode == 0:
                devices = arecord_result.stdout.strip().split('\n')
                real_devices = [d for d in devices if 'card' in d and 'USB Audio' in d]
                result["real_devices"] = real_devices
                result["details"].append(f"✅ 真实音频设备检测: {len(real_devices)}个")

                for device in real_devices:
                    result["details"].append(f"  🎤 {device.strip()}")

                if real_devices:
                    result["success"] = True
            else:
                result["details"].append(f"❌ 音频设备检测失败: {arecord_result.stderr}")

            # 检查真实音频输出设备
            aplay_result = subprocess.run(['aplay', '-l'],
                                         capture_output=True, text=True, timeout=10)

            if aplay_result.returncode == 0:
                output_devices = aplay_result.stdout.strip().split('\n')
                real_output = [d for d in output_devices if 'card' in d]
                result["details"].append(f"✅ 真实输出设备: {len(real_output)}个")

                for device in real_output[:2]:  # 只显示前2个
                    result["details"].append(f"  🔊 {device.strip()}")

        except Exception as e:
            result["details"].append(f"❌ 真实音频硬件检查异常: {e}")

        return result

    def test_real_microphone_input(self) -> dict:
        """测试真实麦克风输入 - 严禁Mock数据"""
        self.log("测试真实麦克风输入 - 录制真实音频")
        result = {"success": False, "audio_file": None, "file_size": 0, "details": []}

        try:
            # 创建真实音频文件
            test_audio = tempfile.mktemp(suffix='.wav')

            self.log("开始录制真实音频 (3秒)...")
            self.log("请对着麦克风说话或制造声音")

            # 使用真实设备录制音频
            cmd = ['arecord', '-d', '3', '-f', 'cd', '-D', 'plughw:0,0', test_audio]
            process = subprocess.run(cmd, capture_output=True, text=True, timeout=15)

            if process.returncode == 0 and Path(test_audio).exists():
                file_size = Path(test_audio).stat().st_size
                result["file_size"] = file_size
                result["audio_file"] = test_audio

                if file_size > 1000:  # 真实音频文件应该大于1KB
                    result["success"] = True
                    result["details"].append(f"✅ 真实音频录制成功")
                    result["details"].append(f"  📁 文件: {test_audio}")
                    result["details"].append(f"  📏 大小: {file_size:,} 字节")

                    # 获取音频信息
                    soxi_cmd = ['soxi', test_audio]
                    try:
                        soxi_result = subprocess.run(soxi_cmd, capture_output=True, text=True, timeout=5)
                        if soxi_result.returncode == 0:
                            result["details"].append(f"  🎵 音频信息:")
                            for line in soxi_result.stdout.strip().split('\n')[:3]:
                                result["details"].append(f"    {line}")
                    except:
                        result["details"].append("  ⚠️ 无法获取音频详细信息")
                else:
                    result["details"].append(f"❌ 录制的音频文件过小: {file_size} 字节")
                    Path(test_audio).unlink()  # 删除无效文件
            else:
                result["details"].append(f"❌ 真实音频录制失败: {process.stderr}")

        except subprocess.TimeoutExpired:
            result["details"].append("❌ 音频录制超时")
        except Exception as e:
            result["details"].append(f"❌ 真实麦克风测试异常: {e}")

        return result

    def test_real_speaker_output(self, audio_file: str) -> dict:
        """测试真实扬声器输出 - 严禁Mock数据"""
        self.log("测试真实扬声器输出 - 播放录制音频")
        result = {"success": False, "playback_time": 0, "details": []}

        if not audio_file or not Path(audio_file).exists():
            result["details"].append("❌ 没有可播放的真实音频文件")
            return result

        try:
            self.log("播放录制的真实音频...")
            start_time = time.time()

            # 使用真实设备播放音频
            cmd = ['aplay', '-D', 'plughw:0,0', audio_file]
            process = subprocess.run(cmd, capture_output=True, text=True, timeout=30)

            result["playback_time"] = time.time() - start_time

            if process.returncode == 0:
                result["success"] = True
                result["details"].append(f"✅ 真实扬声器播放成功")
                result["details"].append(f"  ⏱️ 播放时间: {result['playback_time']:.2f}秒")
            else:
                result["details"].append(f"❌ 真实扬声器播放失败: {process.stderr}")

        except subprocess.TimeoutExpired:
            result["details"].append("❌ 音频播放超时")
        except Exception as e:
            result["details"].append(f"❌ 真实扬声器测试异常: {e}")

        return result

    def test_real_aliyun_api(self) -> dict:
        """测试真实阿里云API调用 - 严禁Mock数据"""
        self.log("测试真实阿里云API调用 - 真实网络请求")
        result = {"success": False, "api_type": "", "response_time": 0, "details": []}

        try:
            # 检查真实API凭证
            access_key_id = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_ID')
            access_key_secret = os.getenv('ALIBABA_CLOUD_ACCESS_KEY_SECRET')

            if not access_key_id or not access_key_secret:
                result["details"].append("❌ 未配置真实API凭证")
                result["details"].append("  请设置环境变量 ALIBABA_CLOUD_ACCESS_KEY_ID 和 ALIBABA_CLOUD_ACCESS_KEY_SECRET")
                return result

            result["details"].append("✅ 真实API凭证已配置")
            result["details"].append(f"  🔑 Access Key ID: {access_key_id[:8]}...{access_key_id[-4:]}")

            # 导入真实API客户端
            sys.path.insert(0, str(self.src_path))

            # 测试真实ASR API
            self.log("测试真实阿里云ASR API...")
            start_time = time.time()

            try:
                from modules.asr.websocket.websocket_asr_service import AliyunASRWebSocketService

                # 创建真实客户端
                asr_client = AliyunASRWebSocketService()
                result["details"].append("✅ 真实ASR客户端创建成功")

                # 尝试获取Token (真实API调用)
                if hasattr(asr_client, 'get_token'):
                    token = asr_client.get_token()
                    if token:
                        result["api_type"] = "ASR"
                        result["success"] = True
                        result["response_time"] = time.time() - start_time
                        result["details"].append(f"✅ 真实ASR Token获取成功")
                        result["details"].append(f"  ⏱️ 响应时间: {result['response_time']:.2f}秒")
                        result["details"].append(f"  🔐 Token长度: {len(token)}字符")
                    else:
                        result["details"].append("❌ 真实ASR Token获取失败")
                else:
                    result["details"].append("❌ ASR客户端缺少get_token方法")

            except ImportError as e:
                result["details"].append(f"❌ ASR模块导入失败: {e}")
            except Exception as e:
                result["details"].append(f"❌ ASR API调用异常: {e}")

            # 如果ASR失败，尝试TTS
            if not result["success"]:
                self.log("测试真实阿里云TTS API...")
                start_time = time.time()

                try:
                    from modules.tts.engine.aliyun_tts_websocket_client import AliyunTTSWebSocketService

                    # 创建真实客户端
                    tts_client = AliyunTTSWebSocketService()
                    result["details"].append("✅ 真实TTS客户端创建成功")

                    # 尝试获取Token (真实API调用)
                    if hasattr(tts_client, 'get_token'):
                        token = tts_client.get_token()
                        if token:
                            result["api_type"] = "TTS"
                            result["success"] = True
                            result["response_time"] = time.time() - start_time
                            result["details"].append(f"✅ 真实TTS Token获取成功")
                            result["details"].append(f"  ⏱️ 响应时间: {result['response_time']:.2f}秒")
                            result["details"].append(f"  🔐 Token长度: {len(token)}字符")
                        else:
                            result["details"].append("❌ 真实TTS Token获取失败")
                    else:
                        result["details"].append("❌ TTS客户端缺少get_token方法")

                except ImportError as e:
                    result["details"].append(f"❌ TTS模块导入失败: {e}")
                except Exception as e:
                    result["details"].append(f"❌ TTS API调用异常: {e}")

        except Exception as e:
            result["details"].append(f"❌ 真实API测试异常: {e}")

        return result

    def verify_real_code_files(self) -> dict:
        """验证真实代码文件 - 严禁Mock文件"""
        self.log("验证真实代码文件 existence and content")
        result = {"success": False, "real_files": [], "mock_files": [], "details": []}

        # 检查关键真实文件
        required_files = [
            "xlerobot/asr/aliyun_asr_client.py",
            "xlerobot/tts/aliyun_tts_client.py",
            "xlerobot_phase1/wake_word_detector.py",
            "modules/asr/simple_aliyun_asr_service.py"
        ]

        real_count = 0
        for file_path in required_files:
            full_path = self.src_path / file_path
            if full_path.exists():
                real_count += 1
                result["real_files"].append(file_path)
                result["details"].append(f"✅ 真实代码文件: {file_path}")

                # 检查文件内容是否包含Mock警告
                try:
                    with open(full_path, 'r', encoding='utf-8') as f:
                        content = f.read()
                        if 'mock' in content.lower() or 'fake' in content.lower():
                            result["mock_files"].append(file_path)
                            result["details"].append(f"  ⚠️ 文件可能包含Mock数据")
                        else:
                            line_count = len([line for line in content.split('\n') if line.strip()])
                            result["details"].append(f"  📄 {line_count} 行真实代码")
                except Exception as e:
                    result["details"].append(f"  ❌ 读取文件失败: {e}")
            else:
                result["details"].append(f"❌ 缺失文件: {file_path}")

        result["success"] = real_count >= 3  # 至少3个关键文件存在

        # 检查是否有明确禁止Mock的标注
        result["details"].append(f"📋 真实文件统计: {real_count}/{len(required_files)}")
        if result["mock_files"]:
            result["details"].append(f"⚠️ 发现 {len(result['mock_files'])} 个可能包含Mock的文件")

        return result

    def run_strict_real_verification(self) -> dict:
        """运行严格的真实环境验证"""
        self.log("🚀 开始 Epic 1 严格真实环境验证")
        self.log("🚨 严格遵守: 严禁使用任何Mock、模拟或硬编码数据")

        # 1. 验证真实硬件
        hardware_result = self.check_real_audio_hardware()
        self.test_results["hardware"] = hardware_result

        # 2. 验证真实代码文件
        code_result = self.verify_real_code_files()
        self.test_results["code_files"] = code_result

        # 3. 测试真实麦克风输入
        mic_result = self.test_real_microphone_input()
        self.test_results["microphone"] = mic_result

        # 4. 测试真实扬声器输出 (如果有录音文件)
        speaker_result = {"success": False, "details": ["跳过: 没有录音文件"]}
        if mic_result.get("audio_file"):
            speaker_result = self.test_real_speaker_output(mic_result["audio_file"])
        self.test_results["speaker"] = speaker_result

        # 5. 测试真实API调用
        api_result = self.test_real_aliyun_api()
        self.test_results["real_api"] = api_result

        # 清理临时文件
        if mic_result.get("audio_file") and Path(mic_result["audio_file"]).exists():
            try:
                Path(mic_result["audio_file"]).unlink()
                self.log("清理临时音频文件")
            except:
                pass

        return self.test_results

    def generate_real_verification_report(self, results: dict) -> str:
        """生成真实验证报告"""
        report = []
        report.append("=" * 80)
        report.append("🔬 Epic 1 严格真实环境验证报告")
        report.append("=" * 80)
        report.append("🚨 严格遵守: 严禁使用任何Mock、模拟或硬编码数据")
        report.append(f"验证时间: {time.strftime('%Y-%m-%d %H:%M:%S')}")
        report.append("")

        # 验证概览
        total_tests = len(results)
        passed_tests = sum(1 for r in results.values() if r["success"])
        real_rate = passed_tests / total_tests * 100

        report.append("## 📊 真实验证概览")
        report.append(f"- 总验证项: {total_tests}")
        report.append(f"- 通过验证: {passed_tests}")
        report.append(f"- 真实率: {real_rate:.1f}%")
        report.append("")

        # 详细结果
        test_names = {
            "hardware": "真实音频硬件",
            "code_files": "真实代码文件",
            "microphone": "真实麦克风输入",
            "speaker": "真实扬声器输出",
            "real_api": "真实阿里云API"
        }

        for test_key, test_name in test_names.items():
            if test_key in results:
                test_result = results[test_key]
                status = "✅ 真实" if test_result["success"] else "❌ 失败"
                report.append(f"## {test_name} - {status}")

                for detail in test_result.get("details", []):
                    report.append(f"  {detail}")

                report.append("")

        # 真实环境评估
        report.append("## 🎯 真实环境评估")

        if real_rate >= 80:
            report.append("✅ Epic 1 在真实环境中基本可用")
        elif real_rate >= 60:
            report.append("⚠️ Epic 1 部分功能在真实环境中可用")
        else:
            report.append("❌ Epic 1 在真实环境中存在重大问题")

        # 真实使用建议
        report.append("## 💡 真实环境使用建议")

        if results.get("real_api", {}).get("success"):
            api_type = results["real_api"]["api_type"]
            report.append(f"- ✅ 真实{api_type} API已验证，可以进行真实调用")
        else:
            report.append("- ❌ 需要配置真实阿里云API凭证")

        if results.get("microphone", {}).get("success"):
            report.append("- ✅ 真实麦克风输入已验证，可以录制真实音频")
        else:
            report.append("- ❌ 需要解决真实音频输入问题")

        if results.get("speaker", {}).get("success"):
            report.append("- ✅ 真实扬声器输出已验证，可以播放真实音频")
        else:
            report.append("- ❌ 需要解决真实音频输出问题")

        # Mock数据检查
        mock_files = results.get("code_files", {}).get("mock_files", [])
        if mock_files:
            report.append("")
            report.append("## ⚠️ Mock数据检查")
            report.append(f"发现 {len(mock_files)} 个可能包含Mock数据的文件:")
            for file_path in mock_files:
                report.append(f"- {file_path}")
            report.append("建议检查这些文件并替换为真实实现")

        report.append("")
        report.append("=" * 80)
        report.append("🔬 严格真实环境验证完成 - 严禁Mock数据")
        report.append("=" * 80)

        return "\n".join(report)

def main():
    """主函数 - 严格真实验证"""
    print("🔬 Epic 1 严格真实环境验证开始...")
    print("🚨 警告: 本验证严禁使用任何Mock、模拟或硬编码数据")
    print("🎤 将测试真实麦克风录制和扬声器播放")
    print("🌐 将测试真实阿里云API调用")

    verifier = RealEpic1Verifier()

    # 运行严格真实验证
    results = verifier.run_strict_real_verification()

    # 生成真实验证报告
    report = verifier.generate_real_verification_report(results)

    # 输出报告
    print(report)

    # 保存报告
    report_file = Path(__file__).parent / f"real_epic1_verification_{int(time.time())}.md"
    with open(report_file, 'w', encoding='utf-8') as f:
        f.write(report)

    print(f"\n📄 真实验证报告已保存到: {report_file}")

    return results

if __name__ == "__main__":
    main()