#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XleRobot Story 1.4 - TTS性能测试脚本
BMad-Method v6 Brownfield Level 4 企业级实现
Story 1.4: 基础语音合成 (阿里云TTS API集成)

全面测试TTS服务的性能指标，包括响应时间、资源使用、并发能力等
"""

import os
import sys
import time
import threading
import statistics
import json
import logging
import psutil
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List, Dict, Any
from dataclasses import dataclass, asdict
import tempfile

# 添加项目路径
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / 'src'))

from xlerobot.tts.aliyun_tts_client import AliyunTTSClient
from xlerobot.tts.audio_processor import AudioProcessor


@dataclass
class PerformanceMetrics:
    """性能指标数据类"""
    test_name: str
    total_requests: int
    successful_requests: int
    failed_requests: int
    avg_response_time: float
    min_response_time: float
    max_response_time: float
    p95_response_time: float
    p99_response_time: float
    requests_per_second: float
    memory_usage_mb: float
    cpu_usage_percent: float
    audio_quality_avg: float
    error_rate: float

    def to_dict(self):
        """转换为字典"""
        return asdict(self)


class TTSPerformanceTester:
    """TTS性能测试器"""

    def __init__(self):
        """初始化性能测试器"""
        self.setup_logging()
        self.client = None
        self.processor = None
        self.process = psutil.Process()

    def setup_logging(self):
        """设置日志"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)

    def initialize_components(self):
        """初始化TTS组件"""
        try:
            from xlerobot.tts.audio_processor import TTSConfigManager
            config_manager = TTSConfigManager()
            config = config_manager.get_config()

            self.client = AliyunTTSClient(config)
            self.processor = AudioProcessor()

            self.logger.info("✅ TTS组件初始化完成")
            return True
        except Exception as e:
            self.logger.error(f"❌ TTS组件初始化失败: {e}")
            return False

    def get_system_metrics(self) -> Dict[str, float]:
        """获取系统指标"""
        try:
            memory_info = self.process.memory_info()
            return {
                'memory_mb': memory_info.rss / 1024 / 1024,
                'cpu_percent': self.process.cpu_percent(),
                'threads': self.process.num_threads()
            }
        except Exception as e:
            self.logger.error(f"获取系统指标失败: {e}")
            return {'memory_mb': 0, 'cpu_percent': 0, 'threads': 0}

    def single_request_test(self, text: str, **kwargs) -> Dict[str, Any]:
        """单个请求测试"""
        result = {
            'success': False,
            'response_time': 0,
            'audio_size': 0,
            'quality_score': 0,
            'error': None
        }

        try:
            start_time = time.time()
            audio_data = self.client.synthesize_speech(text, **kwargs)
            end_time = time.time()

            if audio_data:
                result['success'] = True
                result['response_time'] = end_time - start_time
                result['audio_size'] = len(audio_data)

                # 评估音频质量
                if self.processor:
                    quality = self.processor.evaluate_audio_quality(audio_data)
                    result['quality_score'] = quality.get('quality_score', 0)
            else:
                result['error'] = "No audio data returned"

        except Exception as e:
            result['error'] = str(e)

        return result

    def test_basic_performance(self, request_count: int = 20) -> PerformanceMetrics:
        """基础性能测试"""
        self.logger.info(f"🚀 开始基础性能测试 ({request_count} 个请求)")

        results = []
        start_test_time = time.time()
        initial_metrics = self.get_system_metrics()

        test_text = "这是一个基础性能测试，用于评估TTS服务的响应时间和音频质量。"

        for i in range(request_count):
            # 添加一些变化避免缓存影响
            text = f"{test_text} (测试 {i+1}/{request_count})"
            result = self.single_request_test(text)
            results.append(result)

            if (i + 1) % 5 == 0:
                self.logger.info(f"已完成 {i+1}/{request_count} 个请求")

        end_test_time = time.time()
        final_metrics = self.get_system_metrics()

        # 计算性能指标
        successful_results = [r for r in results if r['success']]
        response_times = [r['response_time'] for r in successful_results]
        quality_scores = [r['quality_score'] for r in successful_results if r['quality_score'] > 0]

        if response_times:
            avg_response_time = statistics.mean(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)
            p95_response_time = statistics.quantiles(response_times, n=20)[18] if len(response_times) > 20 else max(response_times)
            p99_response_time = statistics.quantiles(response_times, n=100)[98] if len(response_times) > 100 else max(response_times)
        else:
            avg_response_time = min_response_time = max_response_time = p95_response_time = p99_response_time = 0

        if quality_scores:
            audio_quality_avg = statistics.mean(quality_scores)
        else:
            audio_quality_avg = 0

        total_test_time = end_test_time - start_test_time
        requests_per_second = len(successful_results) / total_test_time if total_test_time > 0 else 0

        metrics = PerformanceMetrics(
            test_name="基础性能测试",
            total_requests=request_count,
            successful_requests=len(successful_results),
            failed_requests=request_count - len(successful_results),
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            p95_response_time=p95_response_time,
            p99_response_time=p99_response_time,
            requests_per_second=requests_per_second,
            memory_usage_mb=final_metrics['memory_mb'] - initial_metrics['memory_mb'],
            cpu_usage_percent=final_metrics['cpu_percent'],
            audio_quality_avg=audio_quality_avg,
            error_rate=(request_count - len(successful_results)) / request_count
        )

        self.logger.info(f"✅ 基础性能测试完成: {len(successful_results)}/{request_count} 成功")
        return metrics

    def test_concurrent_performance(self, thread_count: int = 5, requests_per_thread: int = 10) -> PerformanceMetrics:
        """并发性能测试"""
        self.logger.info(f"🚀 开始并发性能测试 ({thread_count} 线程, 每线程 {requests_per_thread} 请求)")

        results = []
        start_test_time = time.time()
        initial_metrics = self.get_system_metrics()

        def worker_task(thread_id: int) -> List[Dict[str, Any]]:
            """工作线程任务"""
            thread_results = []
            test_text = f"这是一个并发性能测试，线程 {thread_id} 正在执行请求。"

            for i in range(requests_per_thread):
                text = f"{test_text} (请求 {i+1}/{requests_per_thread})"
                result = self.single_request_test(text)
                result['thread_id'] = thread_id
                thread_results.append(result)

            return thread_results

        # 使用线程池执行并发测试
        with ThreadPoolExecutor(max_workers=thread_count) as executor:
            futures = [executor.submit(worker_task, i) for i in range(thread_count)]

            for future in as_completed(futures):
                try:
                    thread_results = future.result()
                    results.extend(thread_results)
                except Exception as e:
                    self.logger.error(f"线程执行失败: {e}")

        end_test_time = time.time()
        final_metrics = self.get_system_metrics()

        # 计算性能指标 (与基础测试相同)
        successful_results = [r for r in results if r['success']]
        response_times = [r['response_time'] for r in successful_results]
        quality_scores = [r['quality_score'] for r in successful_results if r['quality_score'] > 0]

        if response_times:
            avg_response_time = statistics.mean(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)
            p95_response_time = statistics.quantiles(response_times, n=20)[18] if len(response_times) > 20 else max(response_times)
            p99_response_time = statistics.quantiles(response_times, n=100)[98] if len(response_times) > 100 else max(response_times)
        else:
            avg_response_time = min_response_time = max_response_time = p95_response_time = p99_response_time = 0

        if quality_scores:
            audio_quality_avg = statistics.mean(quality_scores)
        else:
            audio_quality_avg = 0

        total_test_time = end_test_time - start_test_time
        requests_per_second = len(successful_results) / total_test_time if total_test_time > 0 else 0

        metrics = PerformanceMetrics(
            test_name=f"并发性能测试({thread_count}线程)",
            total_requests=thread_count * requests_per_thread,
            successful_requests=len(successful_results),
            failed_requests=len(results) - len(successful_results),
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            p95_response_time=p95_response_time,
            p99_response_time=p99_response_time,
            requests_per_second=requests_per_second,
            memory_usage_mb=final_metrics['memory_mb'] - initial_metrics['memory_mb'],
            cpu_usage_percent=final_metrics['cpu_percent'],
            audio_quality_avg=audio_quality_avg,
            error_rate=(len(results) - len(successful_results)) / len(results)
        )

        self.logger.info(f"✅ 并发性能测试完成: {len(successful_results)}/{len(results)} 成功")
        return metrics

    def test_parameter_performance(self) -> PerformanceMetrics:
        """参数调节性能测试"""
        self.logger.info("🚀 开始参数调节性能测试")

        test_cases = [
            {"name": "标准语音", "params": {}},
            {"name": "快速语音", "params": {"speech_rate": 80}},
            {"name": "慢速语音", "params": {"speech_rate": 20}},
            {"name": "高音调", "params": {"pitch_rate": 80}},
            {"name": "低音调", "params": {"pitch_rate": 20}},
            {"name": "大声语音", "params": {"volume": 100}},
            {"name": "情感友好", "params": {"emotion": "friendly"}},
            {"name": "情感确认", "params": {"emotion": "confirm"}},
            {"name": "质量增强", "params": {"enhance_quality": True}},
        ]

        results = []
        start_test_time = time.time()
        initial_metrics = self.get_system_metrics()

        for case in test_cases:
            self.logger.info(f"测试 {case['name']}...")
            case_results = []

            for i in range(5):  # 每种参数测试5次
                text = f"这是{case['name']}的性能测试，正在执行第 {i+1} 次测试。"

                start_time = time.time()
                audio_data = self.client.synthesize_speech(text, **case['params'])
                end_time = time.time()

                if audio_data:
                    # 应用额外的音频处理
                    if 'emotion' in case['params']:
                        audio_data = self.processor.apply_emotion_style(audio_data, case['params']['emotion'])
                    if case['params'].get('enhance_quality'):
                        audio_data = self.processor.enhance_audio_quality(audio_data)

                    # 评估质量
                    quality = self.processor.evaluate_audio_quality(audio_data)

                    result = {
                        'success': True,
                        'response_time': end_time - start_time,
                        'audio_size': len(audio_data),
                        'quality_score': quality.get('quality_score', 0),
                        'case_name': case['name']
                    }
                else:
                    result = {
                        'success': False,
                        'response_time': end_time - start_time,
                        'audio_size': 0,
                        'quality_score': 0,
                        'case_name': case['name']
                    }

                case_results.append(result)

            results.extend(case_results)

        end_test_time = time.time()
        final_metrics = self.get_system_metrics()

        # 计算性能指标
        successful_results = [r for r in results if r['success']]
        response_times = [r['response_time'] for r in successful_results]
        quality_scores = [r['quality_score'] for r in successful_results if r['quality_score'] > 0]

        if response_times:
            avg_response_time = statistics.mean(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)
            p95_response_time = statistics.quantiles(response_times, n=20)[18] if len(response_times) > 20 else max(response_times)
            p99_response_time = statistics.quantiles(response_times, n=100)[98] if len(response_times) > 100 else max(response_times)
        else:
            avg_response_time = min_response_time = max_response_time = p95_response_time = p99_response_time = 0

        if quality_scores:
            audio_quality_avg = statistics.mean(quality_scores)
        else:
            audio_quality_avg = 0

        total_test_time = end_test_time - start_test_time
        requests_per_second = len(successful_results) / total_test_time if total_test_time > 0 else 0

        metrics = PerformanceMetrics(
            test_name="参数调节性能测试",
            total_requests=len(results),
            successful_requests=len(successful_results),
            failed_requests=len(results) - len(successful_results),
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            p95_response_time=p95_response_time,
            p99_response_time=p99_response_time,
            requests_per_second=requests_per_second,
            memory_usage_mb=final_metrics['memory_mb'] - initial_metrics['memory_mb'],
            cpu_usage_percent=final_metrics['cpu_percent'],
            audio_quality_avg=audio_quality_avg,
            error_rate=(len(results) - len(successful_results)) / len(results)
        )

        self.logger.info(f"✅ 参数调节性能测试完成: {len(successful_results)}/{len(results)} 成功")
        return metrics

    def test_stability_performance(self, duration_minutes: int = 2) -> PerformanceMetrics:
        """稳定性性能测试"""
        self.logger.info(f"🚀 开始稳定性性能测试 (持续 {duration_minutes} 分钟)")

        results = []
        start_test_time = time.time()
        initial_metrics = self.get_system_metrics()
        end_time = start_test_time + (duration_minutes * 60)
        request_count = 0

        while time.time() < end_time:
            text = f"稳定性测试请求 {request_count + 1}，当前时间: {time.strftime('%H:%M:%S')}"
            result = self.single_request_test(text)
            result['request_id'] = request_count
            results.append(result)
            request_count += 1

            # 每10个请求记录一次系统指标
            if request_count % 10 == 0:
                current_metrics = self.get_system_metrics()
                memory_growth = current_metrics['memory_mb'] - initial_metrics['memory_mb']
                self.logger.info(f"已完成 {request_count} 个请求, 内存增长: {memory_growth:.2f}MB")

            # 请求间隔
            time.sleep(0.5)

        actual_duration = time.time() - start_test_time
        final_metrics = self.get_system_metrics()

        # 计算性能指标
        successful_results = [r for r in results if r['success']]
        response_times = [r['response_time'] for r in successful_results]
        quality_scores = [r['quality_score'] for r in successful_results if r['quality_score'] > 0]

        if response_times:
            avg_response_time = statistics.mean(response_times)
            min_response_time = min(response_times)
            max_response_time = max(response_times)
            p95_response_time = statistics.quantiles(response_times, n=20)[18] if len(response_times) > 20 else max(response_times)
            p99_response_time = statistics.quantiles(response_times, n=100)[98] if len(response_times) > 100 else max(response_times)
        else:
            avg_response_time = min_response_time = max_response_time = p95_response_time = p99_response_time = 0

        if quality_scores:
            audio_quality_avg = statistics.mean(quality_scores)
        else:
            audio_quality_avg = 0

        requests_per_second = len(successful_results) / actual_duration if actual_duration > 0 else 0

        metrics = PerformanceMetrics(
            test_name=f"稳定性性能测试({duration_minutes}分钟)",
            total_requests=request_count,
            successful_requests=len(successful_results),
            failed_requests=request_count - len(successful_results),
            avg_response_time=avg_response_time,
            min_response_time=min_response_time,
            max_response_time=max_response_time,
            p95_response_time=p95_response_time,
            p99_response_time=p99_response_time,
            requests_per_second=requests_per_second,
            memory_usage_mb=final_metrics['memory_mb'] - initial_metrics['memory_mb'],
            cpu_usage_percent=final_metrics['cpu_percent'],
            audio_quality_avg=audio_quality_avg,
            error_rate=(request_count - len(successful_results)) / request_count
        )

        self.logger.info(f"✅ 稳定性性能测试完成: {len(successful_results)}/{request_count} 成功")
        return metrics

    def save_results(self, metrics_list: List[PerformanceMetrics], output_file: str = None):
        """保存测试结果"""
        if output_file is None:
            output_file = f"performance_test_results_{int(time.time())}.json"

        results_data = {
            'timestamp': time.time(),
            'system_info': {
                'platform': sys.platform,
                'python_version': sys.version,
                'cpu_count': psutil.cpu_count(),
                'memory_total_gb': psutil.virtual_memory().total / 1024 / 1024 / 1024
            },
            'test_results': [metric.to_dict() for metric in metrics_list]
        }

        try:
            with open(output_file, 'w', encoding='utf-8') as f:
                json.dump(results_data, f, indent=2, ensure_ascii=False)
            self.logger.info(f"📊 测试结果已保存到: {output_file}")
        except Exception as e:
            self.logger.error(f"保存测试结果失败: {e}")

    def print_summary(self, metrics_list: List[PerformanceMetrics]):
        """打印测试摘要"""
        print("\n" + "=" * 60)
        print("📊 TTS性能测试报告")
        print("=" * 60)

        for metric in metrics_list:
            print(f"\n🔍 {metric.test_name}")
            print(f"   总请求数: {metric.total_requests}")
            print(f"   成功请求: {metric.successful_requests}")
            print(f"   失败请求: {metric.failed_requests}")
            print(f"   错误率: {metric.error_rate:.2%}")
            print(f"   平均响应时间: {metric.avg_response_time:.3f}s")
            print(f"   响应时间范围: {metric.min_response_time:.3f}s - {metric.max_response_time:.3f}s")
            print(f"   P95响应时间: {metric.p95_response_time:.3f}s")
            print(f"   P99响应时间: {metric.p99_response_time:.3f}s")
            print(f"   请求速率: {metric.requests_per_second:.2f} req/s")
            print(f"   内存增长: {metric.memory_usage_mb:.2f} MB")
            print(f"   CPU使用率: {metric.cpu_usage_percent:.1f}%")
            print(f"   平均音频质量: {metric.audio_quality_avg:.1f} 分")

        # 性能评级
        print(f"\n🏆 性能评级:")
        avg_response_time = statistics.mean([m.avg_response_time for m in metrics_list])
        avg_error_rate = statistics.mean([m.error_rate for m in metrics_list])
        avg_memory_growth = statistics.mean([m.memory_usage_mb for m in metrics_list])

        if avg_response_time < 2.0 and avg_error_rate < 0.05 and avg_memory_growth < 50:
            grade = "A+ (优秀)"
        elif avg_response_time < 3.0 and avg_error_rate < 0.1 and avg_memory_growth < 100:
            grade = "B+ (良好)"
        elif avg_response_time < 5.0 and avg_error_rate < 0.2:
            grade = "C+ (一般)"
        else:
            grade = "D (需要优化)"

        print(f"   综合评级: {grade}")
        print(f"   平均响应时间: {avg_response_time:.3f}s")
        print(f"   平均错误率: {avg_error_rate:.2%}")
        print(f"   平均内存增长: {avg_memory_growth:.2f} MB")

        print("\n" + "=" * 60)


def main():
    """主函数"""
    print("XleRobot TTS性能测试工具")
    print("=" * 50)

    # 检查环境
    if not os.getenv("ALIBABA_CLOUD_TOKEN"):
        print("⚠️  警告: ALIBABA_CLOUD_TOKEN 未设置")
        print("   测试可能会失败，请设置环境变量后重试")
        return

    # 创建测试器
    tester = TTSPerformanceTester()

    # 初始化组件
    if not tester.initialize_components():
        print("❌ 组件初始化失败，无法进行性能测试")
        return

    # 执行测试
    metrics_list = []

    try:
        # 基础性能测试
        metrics = tester.test_basic_performance(request_count=20)
        metrics_list.append(metrics)

        # 并发性能测试
        metrics = tester.test_concurrent_performance(thread_count=3, requests_per_thread=5)
        metrics_list.append(metrics)

        # 参数调节性能测试
        metrics = tester.test_parameter_performance()
        metrics_list.append(metrics)

        # 稳定性测试 (缩短时间用于演示)
        metrics = tester.test_stability_performance(duration_minutes=1)
        metrics_list.append(metrics)

    except KeyboardInterrupt:
        print("\n⏹️  用户中断测试")
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")

    # 保存和显示结果
    if metrics_list:
        tester.save_results(metrics_list)
        tester.print_summary(metrics_list)
    else:
        print("❌ 没有完成任何测试")


if __name__ == "__main__":
    main()