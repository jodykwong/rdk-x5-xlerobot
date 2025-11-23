"""
CPU调度器 - Story 4.3
实现CPU核心分配、线程池管理、负载均衡算法
支持实时调度优化和优先级控制
"""

import asyncio
import threading
import psutil
import time
import queue
from typing import Any, Dict, List, Optional, Callable, Set
from dataclasses import dataclass, field
from enum import Enum
from concurrent.futures import ThreadPoolExecutor, Future
import logging

from collections import defaultdict, deque

logger = logging.getLogger(__name__)


class CPUPriority(Enum):
    """CPU优先级"""
    LOW = 0
    NORMAL = 1
    HIGH = 2
    CRITICAL = 3
    REALTIME = 4


class LoadBalanceStrategy(Enum):
    """负载均衡策略"""
    ROUND_ROBIN = "round_robin"
    LEAST_LOADED = "least_loaded"
    WEIGHTED_ROUND_ROBIN = "weighted_round_robin"
    CPU_AFFINITY = "cpu_affinity"


@dataclass
class CPUCore:
    """CPU核心信息"""
    core_id: int
    utilization: float = 0.0
    load: float = 0.0
    task_count: int = 0
    priority_tasks: Set[str] = field(default_factory=set)
    affinity_tasks: Set[str] = field(default_factory=set)
    last_updated: float = field(default_factory=time.time)


@dataclass
class Task:
    """CPU任务"""
    task_id: str
    func: Callable
    args: tuple = field(default_factory=tuple)
    kwargs: dict = field(default_factory=dict)
    priority: CPUPriority = CPUPriority.NORMAL
    cpu_affinity: Optional[int] = None  # CPU亲和性
    timeout: Optional[float] = None
    callback: Optional[Callable] = None
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    completed_at: Optional[float] = None


@dataclass
class TaskResult:
    """任务结果"""
    task_id: str
    result: Any = None
    exception: Optional[Exception] = None
    execution_time: float = 0.0
    cpu_core: Optional[int] = None


class CPUScheduler:
    """
    CPU调度器
    实现CPU核心分配、线程池管理、负载均衡
    支持实时调度优化和优先级控制
    """

    def __init__(self,
                 max_workers: Optional[int] = None,
                 load_balance_strategy: LoadBalanceStrategy = LoadBalanceStrategy.ROUND_ROBIN,
                 enable_cpu_affinity: bool = True):
        # 获取CPU信息
        self._cpu_count = psutil.cpu_count(logical=True)
        self._physical_cpu_count = psutil.cpu_count(logical=False)
        self._max_workers = max_workers or self._cpu_count

        # 负载均衡策略
        self._load_balance_strategy = load_balance_strategy
        self._enable_cpu_affinity = enable_cpu_affinity

        # CPU核心状态
        self._cores: Dict[int, CPUCore] = {}
        for i in range(self._cpu_count):
            self._cores[i] = CPUCore(core_id=i)

        # 任务队列
        self._priority_queues: Dict[CPUPriority, queue.Queue] = {
            priority: queue.Queue() for priority in CPUPriority
        }
        self._task_queue = queue.Queue()

        # 任务存储
        self._tasks: Dict[str, Task] = {}
        self._task_futures: Dict[str, Future] = {}
        self._task_results: Dict[str, TaskResult] = {}

        # 线程池
        self._executor = ThreadPoolExecutor(
            max_workers=self._max_workers,
            thread_name_prefix="CPU_Scheduler"
        )

        # 控制标志
        self._running = False
        self._scheduler_thread: Optional[threading.Thread] = None
        self._monitor_thread: Optional[threading.Thread] = None

        # 统计信息
        self._stats = {
            'tasks_submitted': 0,
            'tasks_completed': 0,
            'tasks_failed': 0,
            'average_execution_time': 0.0,
            'cpu_utilization': 0.0,
            'load_balance_efficiency': 0.0
        }

        logger.info(
            "CPU调度器初始化完成",
            cpu_count=self._cpu_count,
            physical_cpu_count=self._physical_cpu_count,
            max_workers=self._max_workers,
            strategy=load_balance_strategy.value
        )

    def start(self):
        """启动CPU调度器"""
        if not self._running:
            self._running = True

            # 启动调度线程
            self._scheduler_thread = threading.Thread(
                target=self._scheduler_loop,
                name="CPU_Scheduler",
                daemon=True
            )
            self._scheduler_thread.start()

            # 启动监控线程
            self._monitor_thread = threading.Thread(
                target=self._monitor_loop,
                name="CPU_Monitor",
                daemon=True
            )
            self._monitor_thread.start()

            logger.info("CPU调度器已启动")

    def stop(self):
        """停止CPU调度器"""
        if self._running:
            self._running = False

            # 等待线程结束
            if self._scheduler_thread:
                self._scheduler_thread.join(timeout=1.0)
            if self._monitor_thread:
                self._monitor_thread.join(timeout=1.0)

            # 关闭线程池
            self._executor.shutdown(wait=True)

            logger.info("CPU调度器已停止")

    def submit_task(self,
                   task_id: str,
                   func: Callable,
                   priority: CPUPriority = CPUPriority.NORMAL,
                   cpu_affinity: Optional[int] = None,
                   timeout: Optional[float] = None,
                   callback: Optional[Callable] = None,
                   *args,
                   **kwargs) -> bool:
        """
        提交任务

        Args:
            task_id: 任务ID
            func: 任务函数
            priority: 任务优先级
            cpu_affinity: CPU亲和性
            timeout: 超时时间
            callback: 完成回调
            *args: 位置参数
            **kwargs: 关键字参数

        Returns:
            bool: 提交是否成功
        """
        try:
            # 创建任务
            task = Task(
                task_id=task_id,
                func=func,
                args=args,
                kwargs=kwargs,
                priority=priority,
                cpu_affinity=cpu_affinity,
                timeout=timeout,
                callback=callback
            )

            # 存储任务
            self._tasks[task_id] = task

            # 添加到优先级队列
            self._priority_queues[priority].put(task)

            # 更新统计
            self._stats['tasks_submitted'] += 1

            logger.debug(
                "任务已提交",
                task_id=task_id,
                priority=priority.name,
                cpu_affinity=cpu_affinity
            )

            return True

        except Exception as e:
            logger.error("任务提交失败", task_id=task_id, error=str(e))
            return False

    def get_task_result(self, task_id: str, timeout: Optional[float] = None) -> Optional[TaskResult]:
        """
        获取任务结果

        Args:
            task_id: 任务ID
            timeout: 超时时间

        Returns:
            Optional[TaskResult]: 任务结果
        """
        if task_id not in self._task_results:
            return None

        return self._task_results[task_id]

    def cancel_task(self, task_id: str) -> bool:
        """
        取消任务

        Args:
            task_id: 任务ID

        Returns:
            bool: 取消是否成功
        """
        try:
            if task_id in self._tasks:
                # 从队列中移除
                task = self._tasks[task_id]
                # 注意：queue.Queue不支持直接移除特定元素，这里简化处理

                # 取消Future
                future = self._task_futures.get(task_id)
                if future and not future.done():
                    future.cancel()

                logger.info("任务已取消", task_id=task_id)
                return True

            return False

        except Exception as e:
            logger.error("取消任务失败", task_id=task_id, error=str(e))
            return False

    def _scheduler_loop(self):
        """调度循环"""
        while self._running:
            try:
                # 获取下一个任务
                task = self._get_next_task()
                if task:
                    # 执行任务
                    self._execute_task(task)
                else:
                    time.sleep(0.001)  # 1ms休眠

            except Exception as e:
                logger.error("调度循环错误", error=str(e))
                time.sleep(0.1)

    def _get_next_task(self) -> Optional[Task]:
        """获取下一个待执行任务"""
        # 按优先级顺序检查队列
        for priority in sorted(CPUPriority, key=lambda p: p.value, reverse=True):
            queue_obj = self._priority_queues[priority]
            try:
                if not queue_obj.empty():
                    return queue_obj.get_nowait()
            except queue.Empty:
                continue

        return None

    def _execute_task(self, task: Task):
        """执行任务"""
        try:
            # 记录开始时间
            task.started_at = time.time()

            # 选择CPU核心
            cpu_core = self._select_cpu_core(task)

            # 创建执行函数
            def _execute():
                # 设置CPU亲和性
                if self._enable_cpu_affinity and task.cpu_affinity is not None:
                    try:
                        import os
                        os.sched_setaffinity(0, {task.cpu_affinity})
                    except (OSError, AttributeError):
                        pass  # 平台不支持或权限不足

                # 执行任务
                start_time = time.perf_counter()
                result = None
                exception = None

                try:
                    result = task.func(*task.args, **task.kwargs)
                except Exception as e:
                    exception = e

                execution_time = time.perf_counter() - start_time

                # 创建结果
                task_result = TaskResult(
                    task_id=task.task_id,
                    result=result,
                    exception=exception,
                    execution_time=execution_time,
                    cpu_core=cpu_core
                )

                # 记录结果
                self._task_results[task.task_id] = task_result

                # 执行回调
                if task.callback:
                    try:
                        task.callback(task_result)
                    except Exception as e:
                        logger.error("任务回调执行失败", task_id=task.task_id, error=str(e))

                # 更新统计
                self._stats['tasks_completed'] += 1
                if exception:
                    self._stats['tasks_failed'] += 1

                # 更新核心状态
                self._update_core_state(cpu_core, -1)

                logger.debug(
                    "任务执行完成",
                    task_id=task.task_id,
                    cpu_core=cpu_core,
                    execution_time=execution_time,
                    success=exception is None
                )

            # 提交到线程池
            future = self._executor.submit(_execute)
            self._task_futures[task.task_id] = future

            # 更新核心状态
            self._update_core_state(cpu_core, 1)

            logger.debug(
                "任务已调度",
                task_id=task.task_id,
                cpu_core=cpu_core
            )

        except Exception as e:
            logger.error("任务执行失败", task_id=task.task_id, error=str(e))
            self._stats['tasks_failed'] += 1

    def _select_cpu_core(self, task: Task) -> int:
        """选择CPU核心"""
        # CPU亲和性优先
        if task.cpu_affinity is not None and task.cpu_affinity in self._cores:
            return task.cpu_affinity

        # 根据负载均衡策略选择
        if self._load_balance_strategy == LoadBalanceStrategy.ROUND_ROBIN:
            return self._round_robin_select()
        elif self._load_balance_strategy == LoadBalanceStrategy.LEAST_LOADED:
            return self._least_loaded_select()
        elif self._load_balance_strategy == LoadBalanceStrategy.WEIGHTED_ROUND_ROBIN:
            return self._weighted_round_robin_select()
        else:
            return 0  # 默认选择核心0

    def _round_robin_select(self) -> int:
        """轮询选择"""
        # 选择任务数最少的核心
        min_tasks = min(core.task_count for core in self._cores.values())
        for core in self._cores.values():
            if core.task_count == min_tasks:
                return core.core_id
        return 0

    def _least_loaded_select(self) -> int:
        """最少负载选择"""
        min_load = min(core.load for core in self._cores.values())
        for core in self._cores.values():
            if core.load == min_load:
                return core.core_id
        return 0

    def _weighted_round_robin_select(self) -> int:
        """加权轮询选择"""
        # 物理核心优先
        physical_cores = list(range(self._physical_cpu_count))
        if physical_cores:
            return self._round_robin_select() % self._physical_cpu_count
        return self._round_robin_select()

    def _update_core_state(self, core_id: int, delta: int):
        """更新核心状态"""
        if core_id in self._cores:
            self._cores[core_id].task_count += delta
            self._cores[core_id].last_updated = time.time()

    def _monitor_loop(self):
        """监控循环"""
        while self._running:
            try:
                # 更新CPU使用率
                self._update_cpu_utilization()

                # 更新负载信息
                self._update_load_info()

                # 更新统计
                self._update_stats()

                time.sleep(1.0)  # 每秒更新一次

            except Exception as e:
                logger.error("监控循环错误", error=str(e))
                time.sleep(1.0)

    def _update_cpu_utilization(self):
        """更新CPU使用率"""
        try:
            # 获取整体CPU使用率
            cpu_percent = psutil.cpu_percent(interval=None)
            self._stats['cpu_utilization'] = cpu_percent

            # 更新每个核心的使用率
            cpu_per_core = psutil.cpu_percent(interval=None, percpu=True)
            for i, usage in enumerate(cpu_per_core):
                if i in self._cores:
                    self._cores[i].utilization = usage

        except Exception as e:
            logger.error("更新CPU使用率失败", error=str(e))

    def _update_load_info(self):
        """更新负载信息"""
        try:
            # 获取系统负载
            load_avg = psutil.getloadavg()[0]  # 1分钟平均负载
            normalized_load = load_avg / self._cpu_count

            # 更新每个核心的负载
            for core in self._cores.values():
                # 基于任务数和使用率计算负载
                task_load = core.task_count / max(self._max_workers, 1)
                utilization_load = core.utilization / 100.0
                core.load = max(task_load, utilization_load, normalized_load)

        except Exception as e:
            logger.error("更新负载信息失败", error=str(e))

    def _update_stats(self):
        """更新统计信息"""
        try:
            # 计算平均执行时间
            if self._stats['tasks_completed'] > 0:
                total_time = sum(
                    result.execution_time
                    for result in self._task_results.values()
                )
                self._stats['average_execution_time'] = (
                    total_time / self._stats['tasks_completed']
                )

            # 计算负载均衡效率
            task_counts = [core.task_count for core in self._cores.values()]
            if task_counts:
                avg_tasks = sum(task_counts) / len(task_counts)
                if avg_tasks > 0:
                    variance = sum((count - avg_tasks) ** 2 for count in task_counts) / len(task_counts)
                    self._stats['load_balance_efficiency'] = max(0, 1 - variance / (avg_tasks ** 2))

        except Exception as e:
            logger.error("更新统计信息失败", error=str(e))

    def get_core_status(self, core_id: Optional[int] = None) -> Dict[int, CPUCore]:
        """
        获取核心状态

        Args:
            core_id: 核心ID，None表示获取所有

        Returns:
            Dict[int, CPUCore]: 核心状态字典
        """
        if core_id is not None:
            return {core_id: self._cores.get(core_id, CPUCore(core_id))}
        else:
            return self._cores.copy()

    def get_stats(self) -> Dict[str, Any]:
        """
        获取调度器统计信息

        Returns:
            Dict: 统计信息
        """
        stats = self._stats.copy()
        stats['total_cores'] = len(self._cores)
        stats['active_tasks'] = len(self._task_futures)
        stats['completed_tasks'] = len(self._task_results)
        stats['strategy'] = self._load_balance_strategy.value
        stats['cpu_affinity_enabled'] = self._enable_cpu_affinity
        return stats


if __name__ == "__main__":
    import random

    async def main():
        """测试CPU调度器"""
        scheduler = CPUScheduler(
            max_workers=4,
            load_balance_strategy=LoadBalanceStrategy.LEAST_LOADED,
            enable_cpu_affinity=True
        )

        scheduler.start()

        # 提交测试任务
        def cpu_intensive_task(duration: float, cpu_id: int):
            """CPU密集型任务"""
            start = time.time()
            while time.time() - start < duration:
                # CPU密集型计算
                sum(range(10000))
            return f"Task completed on CPU {cpu_id}"

        for i in range(10):
            task_id = f"task_{i}"
            cpu_id = random.randint(0, 3)
            duration = random.uniform(0.1, 0.5)

            scheduler.submit_task(
                task_id=task_id,
                func=cpu_intensive_task,
                args=(duration, cpu_id),
                priority=CPUPriority.NORMAL,
                cpu_affinity=cpu_id
            )

        # 等待任务完成
        await asyncio.sleep(5)

        # 获取统计信息
        stats = scheduler.get_stats()
        print(f"调度器统计: {stats}")

        # 获取核心状态
        core_status = scheduler.get_core_status()
        for core_id, core in core_status.items():
            print(f"核心 {core_id}: 利用率={core.utilization:.1f}%, 任务数={core.task_count}")

        # 停止调度器
        scheduler.stop()

    asyncio.run(main())
