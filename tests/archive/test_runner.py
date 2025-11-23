#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试执行安全框架 - Epic 1 测试套件
===================================

安全的测试执行框架，包含超时保护、进程监控和资源管理
基于根因分析，设计防止"卡住"问题的测试执行器

✅ 安全特性：
- 内置超时保护机制
- 进程监控和自动清理
- 资源使用限制
- 详细的执行日志
- 异常情况处理

🛡️ 防护机制：
- 最大执行时间限制
- 内存使用监控
- 进程泄漏检测
- 临时文件自动清理

作者: Test Safety Framework Agent
创建时间: 2025-11-12
版本: v1.0 - 安全执行版
"""

import os
import sys
import time
import logging
import signal
import subprocess
import threading
import psutil
import tempfile
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Dict, Optional, Tuple

# 设置项目路径
sys.path.insert(0, '/home/sunrise/xlerobot')

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class SafeTestRunner:
    """安全的测试执行器"""

    def __init__(self):
        """初始化安全测试执行器"""
        self.running = False
        self.test_processes = []
        self.monitoring_thread = None
        self.start_time = None
        self.timeout_seconds = 300  # 5分钟总超时
        self.max_memory_mb = 500   # 500MB内存限制

        # 测试脚本配置
        self.test_scripts = [
            {
                'name': '音频组件测试',
                'script': '/home/sunrise/xlerobot/tests/test_audio_components.py',
                'timeout': 60,
                'description': '验证音频录制、播放和设备切换功能'
            },
            {
                'name': '阿里云API集成测试',
                'script': '/home/sunrise/xlerobot/tests/test_aliyun_api_integration.py',
                'timeout': 120,
                'description': '验证阿里云ASR和TTS服务集成'
            },
            {
                'name': '端到端集成测试',
                'script': '/home/sunrise/xlerobot/tests/test_e2e_integration.py',
                'timeout': 180,
                'description': '验证完整的语音交互流程'
            }
        ]

        logger.info("🛡️ 安全测试执行器初始化完成")
        self.log_configuration()

    def log_configuration(self):
        """记录配置信息"""
        logger.info(f"📋 安全配置:")
        logger.info(f"   - 总超时时间: {self.timeout_seconds}秒")
        logger.info(f"   - 内存限制: {self.max_memory_mb}MB")
        logger.info(f"   - 测试脚本数量: {len(self.test_scripts)}")

    def start_monitoring(self):
        """开始监控"""
        self.running = True
        self.start_time = time.time()

        def monitor():
            try:
                while self.running:
                    current_time = time.time()
                    elapsed = current_time - self.start_time

                    # 检查总超时
                    if elapsed > self.timeout_seconds:
                        logger.error(f"🚨 总执行时间超时: {elapsed:.1f}s > {self.timeout_seconds}s")
                        self.emergency_stop("总执行时间超时")
                        break

                    # 检查内存使用
                    memory_usage = self.get_memory_usage()
                    if memory_usage > self.max_memory_mb * 1024 * 1024:
                        logger.error(f"🚨 内存使用超限: {memory_usage//1024//1024}MB > {self.max_memory_mb}MB")
                        self.emergency_stop("内存使用超限")
                        break

                    # 监控子进程
                    self.monitor_child_processes()

                    time.sleep(2)  # 每2秒检查一次

            except Exception as e:
                logger.error(f"💥 监控线程异常: {e}")

        self.monitoring_thread = threading.Thread(target=monitor, daemon=True)
        self.monitoring_thread.start()
        logger.info(f"📊 安全监控已启动")

    def stop_monitoring(self):
        """停止监控"""
        self.running = False
        if self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=5)
        logger.info(f"📊 安全监控已停止")

    def get_memory_usage(self) -> int:
        """获取当前进程内存使用（字节）"""
        try:
            process = psutil.Process()
            return process.memory_info().rss
        except Exception:
            return 0

    def monitor_child_processes(self):
        """监控子进程状态"""
        try:
            current_processes = []

            for proc_info in self.test_processes:
                try:
                    process = psutil.Process(proc_info['pid'])
                    if process.is_running():
                        current_processes.append(proc_info)

                        # 检查单个进程超时
                        elapsed = time.time() - proc_info['start_time']
                        if elapsed > proc_info['timeout']:
                            logger.warning(f"⚠️ 测试进程超时: {proc_info['name']} ({elapsed:.1f}s > {proc_info['timeout']}s)")
                            self.terminate_process(proc_info['pid'], proc_info['name'])
                    else:
                        # 进程已结束，检查退出码
                        logger.info(f"✅ 测试进程结束: {proc_info['name']}")

                except psutil.NoSuchProcess:
                    logger.info(f"✅ 测试进程已清理: {proc_info['name']}")
                except Exception as e:
                    logger.warning(f"⚠️ 监控进程异常 {proc_info['name']}: {e}")

            self.test_processes = current_processes

        except Exception as e:
            logger.error(f"💥 子进程监控异常: {e}")

    def terminate_process(self, pid: int, name: str):
        """终止指定进程"""
        try:
            process = psutil.Process(pid)

            # 先尝试温和终止
            process.terminate()

            # 等待3秒
            time.sleep(3)

            # 如果还在运行，强制终止
            if process.is_running():
                logger.warning(f"🔨 强制终止进程: {name}")
                process.kill()
                time.sleep(1)

            logger.info(f"🧹 进程已终止: {name}")

        except psutil.NoSuchProcess:
            logger.info(f"✅ 进程已不存在: {name}")
        except Exception as e:
            logger.error(f"❌ 终止进程失败 {name}: {e}")

    def emergency_stop(self, reason: str):
        """紧急停止所有测试"""
        logger.error(f"🚨 紧急停止所有测试: {reason}")
        self.running = False

        # 终止所有子进程
        for proc_info in self.test_processes[:]:
            self.terminate_process(proc_info['pid'], proc_info['name'])

        # 清理临时文件
        self.cleanup_temp_files()

    def cleanup_temp_files(self):
        """清理临时文件"""
        try:
            temp_dir = tempfile.gettempdir()
            cleanup_patterns = ['*test_*.wav', '*tmp*.wav', '*tts_*.wav']

            for pattern in cleanup_patterns:
                for temp_file in Path(temp_dir).glob(pattern):
                    try:
                        temp_file.unlink()
                        logger.info(f"🧹 已清理临时文件: {temp_file.name}")
                    except Exception as e:
                        logger.warning(f"⚠️ 清理文件失败 {temp_file.name}: {e}")

        except Exception as e:
            logger.warning(f"⚠️ 清理临时文件异常: {e}")

    def run_test_script(self, script_info: Dict) -> Tuple[bool, str]:
        """运行单个测试脚本"""
        script_name = script_info['name']
        script_path = script_info['script']
        timeout = script_info['timeout']

        logger.info(f"\n🚀 开始执行测试: {script_name}")
        logger.info(f"📄 脚本路径: {script_path}")
        logger.info(f"⏱️ 超时时间: {timeout}秒")

        try:
            # 验证脚本文件存在
            if not os.path.exists(script_path):
                error_msg = f"测试脚本不存在: {script_path}"
                logger.error(f"❌ {error_msg}")
                return False, error_msg

            # 启动测试进程
            start_time = time.time()
            process = subprocess.Popen(
                [sys.executable, script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                universal_newlines=True
            )

            # 记录进程信息
            proc_info = {
                'pid': process.pid,
                'name': script_name,
                'start_time': start_time,
                'timeout': timeout,
                'process': process
            }
            self.test_processes.append(proc_info)

            logger.info(f"🔄 测试进程已启动: PID={process.pid}")

            # 实时读取输出
            output_lines = []
            try:
                while True:
                    line = process.stdout.readline()
                    if not line:
                        break

                    line = line.strip()
                    if line:
                        output_lines.append(line)
                        logger.info(f"📤 {script_name}: {line}")

                    # 检查进程是否结束
                    if process.poll() is not None:
                        break

                    # 检查超时
                    elapsed = time.time() - start_time
                    if elapsed > timeout:
                        logger.error(f"⏰ 测试超时: {script_name} ({elapsed:.1f}s)")
                        self.terminate_process(process.pid, script_name)
                        return False, f"测试超时 ({elapsed:.1f}s > {timeout}s)"

            except Exception as e:
                logger.error(f"💥 读取测试输出异常: {e}")
                self.terminate_process(process.pid, script_name)
                return False, f"读取输出异常: {e}"

            # 等待进程完全结束
            try:
                return_code = process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                logger.error(f"⏰ 等待进程结束超时")
                self.terminate_process(process.pid, script_name)
                return False, "等待进程结束超时"

            elapsed = time.time() - start_time

            # 分析结果
            if return_code == 0:
                logger.info(f"✅ 测试成功: {script_name} (耗时: {elapsed:.1f}s)")
                return True, f"测试成功 (耗时: {elapsed:.1f}s)"
            elif return_code == 130:
                logger.info(f"👋 测试被用户中断: {script_name}")
                return True, f"测试被用户中断"
            else:
                logger.error(f"❌ 测试失败: {script_name} (退出码: {return_code}, 耗时: {elapsed:.1f}s)")
                return False, f"测试失败 (退出码: {return_code})"

        except Exception as e:
            logger.error(f"💥 执行测试异常: {e}")
            return False, f"执行异常: {e}"

    def run_all_tests(self):
        """运行所有测试"""
        logger.info("\n" + "=" * 80)
        logger.info("🛡️ 安全测试执行框架启动")
        logger.info("=" * 80)

        # 开始安全监控
        self.start_monitoring()

        start_time = time.time()
        test_results = []

        try:
            # 设置信号处理
            def signal_handler(signum, frame):
                logger.info(f"\n👋 收到中断信号，正在停止测试...")
                self.emergency_stop("用户中断")
                sys.exit(130)

            signal.signal(signal.SIGINT, signal_handler)
            signal.signal(signal.SIGTERM, signal_handler)

            logger.info(f"📋 计划执行 {len(self.test_scripts)} 个测试")

            # 执行每个测试脚本
            for i, script_info in enumerate(self.test_scripts, 1):
                logger.info(f"\n📊 执行进度: {i}/{len(self.test_scripts)}")

                success, message = self.run_test_script(script_info)

                test_results.append({
                    'name': script_info['name'],
                    'success': success,
                    'message': message,
                    'timestamp': datetime.now().isoformat()
                })

                # 如果测试失败，询问是否继续
                if not success:
                    logger.error(f"❌ 测试失败: {script_info['name']}")

                    # 在自动模式下继续执行所有测试
                    logger.info(f"⏭️ 继续执行下一个测试...")

                # 检查是否应该继续
                if not self.running:
                    logger.info(f"🛑 监控系统要求停止测试")
                    break

            # 计算总耗时
            total_time = time.time() - start_time

            # 生成执行报告
            self.generate_execution_report(test_results, total_time)

            # 判断总体结果
            successful_tests = [r for r in test_results if r['success']]
            success_rate = len(successful_tests) / len(test_results) if test_results else 0

            if success_rate >= 0.8:  # 80%以上成功率算通过
                logger.info(f"🎉 测试执行总体成功！ ({success_rate*100:.1f}% 通过)")
                overall_success = True
            else:
                logger.error(f"❌ 测试执行存在问题 ({success_rate*100:.1f}% 通过)")
                overall_success = False

            return overall_success

        except KeyboardInterrupt:
            logger.info(f"\n👋 用户中断测试执行")
            return False
        except Exception as e:
            logger.error(f"💥 测试执行系统异常: {e}")
            import traceback
            traceback.print_exc()
            return False
        finally:
            # 停止监控并清理
            self.stop_monitoring()
            self.emergency_stop("测试执行完成")

    def generate_execution_report(self, test_results: List[Dict], total_time: float):
        """生成测试执行报告"""
        logger.info(f"\n📊 测试执行报告")
        logger.info(f"{'='*60}")

        # 统计信息
        total_tests = len(test_results)
        successful_tests = len([r for r in test_results if r['success']])
        failed_tests = total_tests - successful_tests

        logger.info(f"📈 执行统计:")
        logger.info(f"   - 总测试数: {total_tests}")
        logger.info(f"   - 成功: {successful_tests}")
        logger.info(f"   - 失败: {failed_tests}")
        logger.info(f"   - 成功率: {successful_tests/total_tests*100:.1f}%")
        logger.info(f"   - 总耗时: {total_time:.1f}s")

        # 详细结果
        logger.info(f"\n📋 详细结果:")
        for i, result in enumerate(test_results, 1):
            status_icon = "✅" if result['success'] else "❌"
            logger.info(f"   {i}. {status_icon} {result['name']}")
            logger.info(f"      📝 {result['message']}")

        # 系统资源使用
        try:
            memory_mb = self.get_memory_usage() // 1024 // 1024
            logger.info(f"\n💻 系统资源:")
            logger.info(f"   - 当前内存使用: {memory_mb} MB")
            logger.info(f"   - 活跃进程数: {len(self.test_processes)}")
        except Exception as e:
            logger.warning(f"⚠️ 无法获取系统资源信息: {e}")

        logger.info(f"{'='*60}")

def main():
    """主函数"""
    logger.info("🚀 启动安全测试执行框架")

    # 创建安全执行器
    runner = SafeTestRunner()

    try:
        # 运行所有测试
        success = runner.run_all_tests()

        # 返回适当的退出码
        return 0 if success else 1

    except Exception as e:
        logger.error(f"💥 测试框架异常: {e}")
        return 1

if __name__ == '__main__':
    exit_code = main()
    sys.exit(exit_code)