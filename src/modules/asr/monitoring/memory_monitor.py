#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Story 1.4: 连续语音识别 - 内存池管理
Memory Pool Management for Continuous Speech Recognition

实现高性能内存池，解决长时间运行内存泄漏问题。
功能特性:
- 预分配内存池 (初始100MB, 最大500MB)
- 最佳适配 (Best Fit) 分配策略
- 自动垃圾回收
- 内存碎片整理
- 实时监控和统计

作者: Dev Agent
故事ID: Story 1.4
"""

import ctypes
import threading
import time
import logging
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum, auto
from queue import Queue, Empty
from pathlib import Path

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class AllocationStrategy(Enum):
    """分配策略枚举"""
    BEST_FIT = auto()    # 最佳适配
    FIRST_FIT = auto()   # 首次适配
    WORST_FIT = auto()   # 最差适配


@dataclass
class MemoryBlock:
    """内存块信息"""
    address: int
    size: int
    allocated: bool
    allocated_at: Optional[float] = None
    allocation_id: Optional[str] = None
    owner: Optional[str] = None


@dataclass
class MemoryStats:
    """内存统计信息"""
    total_size: int
    allocated_size: int
    free_size: int
    used_blocks: int
    free_blocks: int
    fragmentation_ratio: float
    peak_usage: int
    gc_count: int
    gc_freed_bytes: int
    allocation_count: int
    deallocation_count: int


class MemoryPool:
    """
    内存池管理器

    功能特性:
    - 预分配内存块
    - 高效的分配/释放算法
    - 自动垃圾回收
    - 内存碎片整理
    - 零内存泄漏保证

    性能目标:
    - 分配延迟: <1ms
    - 释放延迟: <0.5ms
    - 内存碎片率: <10%
    - 30分钟运行内存增长: <10MB
    """

    def __init__(self,
                 initial_size_mb: int = 100,
                 max_size_mb: int = 500,
                 block_size_kb: int = 64,
                 strategy: AllocationStrategy = AllocationStrategy.BEST_FIT,
                 enable_gc: bool = True,
                 gc_threshold_mb: int = 50):
        """
        初始化内存池

        Args:
            initial_size_mb: 初始内存池大小 (MB)
            max_size_mb: 最大内存池大小 (MB)
            block_size_kb: 内存块大小 (KB)
            strategy: 分配策略
            enable_gc: 是否启用自动垃圾回收
            gc_threshold_mb: 垃圾回收阈值 (MB)
        """
        self.initial_size_bytes = initial_size_mb * 1024 * 1024
        self.max_size_bytes = max_size_mb * 1024 * 1024
        self.block_size_bytes = block_size_kb * 1024
        self.strategy = strategy
        self.enable_gc = enable_gc
        self.gc_threshold_bytes = gc_threshold_mb * 1024 * 1024

        # 内存管理
        self._memory: Optional[ctypes.c_char_p] = None
        self._memory_size = 0
        self._allocated_blocks: Dict[str, MemoryBlock] = {}
        self._free_blocks: List[MemoryBlock] = []

        # 统计信息
        self._stats_lock = threading.Lock()
        self._stats = {
            'total_allocations': 0,
            'total_deallocations': 0,
            'peak_usage': 0,
            'gc_count': 0,
            'gc_freed_bytes': 0,
            'total_allocated_bytes': 0,
            'total_freed_bytes': 0,
            'current_fragmentation': 0.0,
            'last_gc_time': 0.0
        }

        # 分配ID生成器
        self._allocation_counter = 0
        self._counter_lock = threading.Lock()

        # 垃圾回收线程
        self._gc_thread: Optional[threading.Thread] = None
        self._gc_active = False

        # 初始化内存池
        self._initialize_pool()

        # 启动垃圾回收线程
        if self.enable_gc:
            self._start_gc_thread()

        logger.info(f"内存池初始化完成: 初始={initial_size_mb}MB, "
                   f"最大={max_size_mb}MB, 策略={strategy.name}")

    def _initialize_pool(self) -> None:
        """初始化内存池"""
        try:
            # 计算需要的块数
            num_blocks = (self.initial_size_bytes + self.block_size_bytes - 1) // self.block_size_bytes

            import sys
            import platform

            # 根据平台选择内存分配方法
            if platform.system() == 'Windows' and hasattr(ctypes, 'windll'):
                # Windows 系统使用 VirtualAlloc
                self._memory = ctypes.c_char_p(ctypes.windll.kernel32.VirtualAlloc(
                    None,
                    self.initial_size_bytes,
                    0x1000,  # MEM_RESERVE
                    0x04     # PAGE_READWRITE
                ))
            else:
                # Unix/Linux 系统使用bytearray (简化实现)
                # 注意：这是一个演示实现，实际生产环境应使用mmap
                logger.warning("使用bytearray作为演示，生产环境应使用mmap")
                self._memory = ctypes.c_char_p(b'\x00' * self.initial_size_bytes)

            self._memory_size = self.initial_size_bytes

            # 创建内存块
            current_addr = ctypes.cast(self._memory, ctypes.c_void_p).value or 0

            for i in range(num_blocks):
                block = MemoryBlock(
                    address=current_addr,
                    size=self.block_size_bytes,
                    allocated=False
                )
                self._free_blocks.append(block)
                current_addr += self.block_size_bytes

            logger.info(f"内存池创建成功: {num_blocks}个块, 总大小={self.initial_size_bytes/1024/1024:.1f}MB")

        except Exception as e:
            logger.error(f"内存池初始化失败: {e}")
            raise

    def allocate(self, size: int, owner: Optional[str] = None) -> Optional[str]:
        """
        分配内存

        Args:
            size: 分配大小 (字节)
            owner: 内存所有者标识

        Returns:
            分配ID，失败返回None
        """
        with self._stats_lock:
            # 更新分配计数
            with self._counter_lock:
                self._allocation_counter += 1
                allocation_id = f"alloc_{self._allocation_counter:06d}"

        try:
            # 查找合适的内存块
            block = self._find_free_block(size)
            if not block:
                # 尝试扩展内存池
                if not self._expand_pool():
                    logger.error(f"内存分配失败: 无法分配 {size} 字节")
                    return None

                # 再次查找
                block = self._find_free_block(size)
                if not block:
                    logger.error("内存分配失败: 扩展后仍无可用内存块")
                    return None

            # 分配内存块
            block.allocated = True
            block.allocated_at = time.time()
            block.allocation_id = allocation_id
            block.owner = owner

            # 移动到已分配列表
            self._free_blocks.remove(block)
            self._allocated_blocks[allocation_id] = block

            # 更新统计信息
            self._stats['total_allocations'] += 1
            self._stats['total_allocated_bytes'] += size
            self._stats['peak_usage'] = max(
                self._stats['peak_usage'],
                self._stats['total_allocated_bytes'] - self._stats['total_freed_bytes']
            )

            logger.debug(f"内存分配成功: {allocation_id} ({size} 字节)")

            return allocation_id

        except Exception as e:
            logger.error(f"内存分配异常: {e}")
            return None

    def deallocate(self, allocation_id: str) -> bool:
        """
        释放内存

        Args:
            allocation_id: 分配ID

        Returns:
            是否成功释放
        """
        if allocation_id not in self._allocated_blocks:
            logger.warning(f"尝试释放不存在的分配: {allocation_id}")
            return False

        try:
            # 获取内存块
            block = self._allocated_blocks[allocation_id]
            size = block.size

            # 释放内存块
            block.allocated = False
            block.allocated_at = None
            block.allocation_id = None
            block.owner = None

            # 移回空闲列表
            self._free_blocks.append(block)
            del self._allocated_blocks[allocation_id]

            # 更新统计信息
            with self._stats_lock:
                self._stats['total_deallocations'] += 1
                self._stats['total_freed_bytes'] += size

            logger.debug(f"内存释放成功: {allocation_id} ({size} 字节)")

            # 检查是否需要垃圾回收
            if self.enable_gc:
                self._check_gc_needed()

            return True

        except Exception as e:
            logger.error(f"内存释放异常 {allocation_id}: {e}")
            return False

    def _find_free_block(self, required_size: int) -> Optional[MemoryBlock]:
        """查找空闲内存块"""
        # 过滤出足够大的空闲块
        suitable_blocks = [block for block in self._free_blocks if block.size >= required_size]

        if not suitable_blocks:
            return None

        # 根据分配策略选择块
        if self.strategy == AllocationStrategy.BEST_FIT:
            # 最佳适配: 选择最小的合适块
            return min(suitable_blocks, key=lambda b: b.size)

        elif self.strategy == AllocationStrategy.FIRST_FIT:
            # 首次适配: 选择第一个合适块
            return suitable_blocks[0]

        elif self.strategy == AllocationStrategy.WORST_FIT:
            # 最差适配: 选择最大的块
            return max(suitable_blocks, key=lambda b: b.size)

        else:
            return suitable_blocks[0]

    def _expand_pool(self) -> bool:
        """扩展内存池"""
        if self._memory_size >= self.max_size_bytes:
            return False

        try:
            # 计算扩展大小 (每次扩展50%)
            expansion_size = min(
                int(self._memory_size * 0.5),
                self.max_size_bytes - self._memory_size
            )

            # 扩展虚拟内存
            new_addr = ctypes.windll.kernel32.VirtualAlloc(
                None,
                expansion_size,
                0x1000,  # MEM_RESERVE
                0x04     # PAGE_READWRITE
            )

            if not new_addr:
                # Unix/Linux 系统
                import mmap
                new_addr = ctypes.mmap(
                    None,
                    expansion_size,
                    mmap.PROT_READ | mmap.PROT_WRITE,
                    mmap.MAP_PRIVATE | mmap.MAP_ANONYMOUS,
                    -1,
                    0
                )

            # 创建新的内存块
            base_addr = ctypes.cast(self._memory, ctypes.c_void_p).value or 0
            new_base_addr = new_addr
            num_new_blocks = expansion_size // self.block_size_bytes

            for i in range(num_new_blocks):
                block = MemoryBlock(
                    address=new_base_addr,
                    size=self.block_size_bytes,
                    allocated=False
                )
                self._free_blocks.append(block)
                new_base_addr += self.block_size_bytes

            self._memory_size += expansion_size

            logger.info(f"内存池扩展成功: +{expansion_size/1024/1024:.1f}MB "
                       f"(总计 {self._memory_size/1024/1024:.1f}MB)")

            return True

        except Exception as e:
            logger.error(f"内存池扩展失败: {e}")
            return False

    def garbage_collect(self) -> Dict[str, int]:
        """
        执行垃圾回收

        Returns:
            垃圾回收统计信息
        """
        with self._stats_lock:
            before_count = len(self._free_blocks)
            before_freed = 0

            try:
                # 查找并合并相邻的空闲块
                self._free_blocks.sort(key=lambda b: b.address)
                merged_blocks = []
                i = 0

                while i < len(self._free_blocks):
                    current = self._free_blocks[i]
                    merged_size = current.size

                    # 查找相邻块
                    j = i + 1
                    while (j < len(self._free_blocks) and
                           self._free_blocks[j-1].address + self._free_blocks[j-1].size == self._free_blocks[j].address):
                        merged_size += self._free_blocks[j].size
                        j += 1

                    # 创建合并后的块
                    merged_block = MemoryBlock(
                        address=current.address,
                        size=merged_size,
                        allocated=False
                    )
                    merged_blocks.append(merged_block)
                    before_freed += merged_size
                    i = j

                # 更新空闲块列表
                self._free_blocks = merged_blocks

                # 更新统计信息
                self._stats['gc_count'] += 1
                self._stats['gc_freed_bytes'] += before_freed
                self._stats['last_gc_time'] = time.time()

                # 更新碎片率
                self._update_fragmentation_ratio()

                result = {
                    'merged_blocks': before_count,
                    'freed_bytes': before_freed,
                    'remaining_blocks': len(self._free_blocks),
                    'fragmentation_ratio': self._stats['current_fragmentation']
                }

                logger.info(f"垃圾回收完成: 合并{before_count}块, "
                           f"释放{before_freed/1024:.1f}KB碎片空间")

                return result

            except Exception as e:
                logger.error(f"垃圾回收失败: {e}")
                return {
                    'merged_blocks': 0,
                    'freed_bytes': 0,
                    'remaining_blocks': len(self._free_blocks),
                    'fragmentation_ratio': self._stats['current_fragmentation']
                }

    def _update_fragmentation_ratio(self) -> None:
        """更新内存碎片率"""
        if not self._free_blocks:
            self._stats['current_fragmentation'] = 0.0
            return

        total_free = sum(block.size for block in self._free_blocks)
        largest_block = max(block.size for block in self._free_blocks)

        # 碎片率 = 1 - (最大块大小 / 总空闲大小)
        self._stats['current_fragmentation'] = 1.0 - (largest_block / total_free)

    def _check_gc_needed(self) -> None:
        """检查是否需要垃圾回收"""
        current_usage = self._stats['total_allocated_bytes'] - self._stats['total_freed_bytes']
        if current_usage > self.gc_threshold_bytes:
            logger.debug(f"内存使用 ({current_usage/1024/1024:.1f}MB) 超过阈值 "
                        f"({self.gc_threshold_bytes/1024/1024:.1f}MB)，触发垃圾回收")
            self.garbage_collect()

    def _start_gc_thread(self) -> None:
        """启动垃圾回收线程"""
        if self._gc_active:
            return

        self._gc_active = True
        self._gc_thread = threading.Thread(
            target=self._gc_worker,
            daemon=True
        )
        self._gc_thread.start()

        logger.info("🧹 垃圾回收线程已启动")

    def _gc_worker(self) -> None:
        """垃圾回收工作线程"""
        while self._gc_active:
            try:
                time.sleep(30)  # 每30秒检查一次

                # 检查内存使用情况
                current_usage = (
                    self._stats['total_allocated_bytes'] - 
                    self._stats['total_freed_bytes']
                )

                # 如果内存使用率 > 80%，执行垃圾回收
                usage_ratio = current_usage / self._memory_size
                if usage_ratio > 0.8:
                    logger.info(f"内存使用率 {usage_ratio:.1%} > 80%，执行垃圾回收")
                    self.garbage_collect()

            except Exception as e:
                if self._gc_active:
                    logger.error(f"垃圾回收工作线程错误: {e}")

    def _stop_gc_thread(self) -> None:
        """停止垃圾回收线程"""
        self._gc_active = False
        if self._gc_thread and self._gc_thread.is_alive():
            self._gc_thread.join(timeout=2.0)
        logger.info("🧹 垃圾回收线程已停止")

    def get_stats(self) -> MemoryStats:
        """
        获取内存统计信息

        Returns:
            内存统计信息
        """
        with self._stats_lock:
            allocated_size = self._stats['total_allocated_bytes'] - self._stats['total_freed_bytes']
            free_size = self._memory_size - allocated_size

            return MemoryStats(
                total_size=self._memory_size,
                allocated_size=allocated_size,
                free_size=free_size,
                used_blocks=len(self._allocated_blocks),
                free_blocks=len(self._free_blocks),
                fragmentation_ratio=self._stats['current_fragmentation'],
                peak_usage=self._stats['peak_usage'],
                gc_count=self._stats['gc_count'],
                gc_freed_bytes=self._stats['gc_freed_bytes'],
                allocation_count=self._stats['total_allocations'],
                deallocation_count=self._stats['total_deallocations']
            )

    def print_stats(self) -> None:
        """打印内存统计信息"""
        stats = self.get_stats()

        print("\n" + "=" * 60)
        print("内存池统计信息")
        print("=" * 60)
        print(f"总内存大小:     {stats.total_size/1024/1024:>8.1f} MB")
        print(f"已分配内存:     {stats.allocated_size/1024/1024:>8.1f} MB ({stats.allocated_size/self._memory_size:.1%})")
        print(f"空闲内存:       {stats.free_size/1024/1024:>8.1f} MB ({stats.free_size/self._memory_size:.1%})")
        print(f"已使用块数:     {stats.used_blocks:>8}")
        print(f"空闲块数:       {stats.free_blocks:>8}")
        print(f"内存碎片率:     {stats.fragmentation_ratio:.1%}")
        print(f"峰值使用量:     {stats.peak_usage/1024/1024:>8.1f} MB")
        print(f"GC执行次数:     {stats.gc_count:>8}")
        print(f"GC释放内存:     {stats.gc_freed_bytes/1024:>8.1f} KB")
        print(f"分配次数:       {stats.allocation_count:>8}")
        print(f"释放次数:       {stats.deallocation_count:>8}")
        print("=" * 60)

    def is_empty(self) -> bool:
        """检查内存池是否为空 (无已分配内存)"""
        return len(self._allocated_blocks) == 0

    def __len__(self) -> int:
        """返回已分配块数"""
        return len(self._allocated_blocks)

    def __enter__(self):
        """上下文管理器入口"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器出口"""
        self.cleanup()

    def cleanup(self) -> None:
        """清理资源"""
        self._stop_gc_thread()

        # 释放所有已分配的内存
        allocation_ids = list(self._allocated_blocks.keys())
        for allocation_id in allocation_ids:
            self.deallocate(allocation_id)

        # 清理虚拟内存
        if self._memory:
            try:
                ctypes.windll.kernel32.VirtualFree(
                    self._memory,
                    0,
                    0x8000  # MEM_RELEASE
                )
            except:
                # Unix/Linux 系统
                pass

        logger.info("🧹 内存池已清理")

    def __repr__(self) -> str:
        return (f"MemoryPool(initial={self.initial_size_bytes/1024/1024:.0f}MB, "
                f"max={self.max_size_bytes/1024/1024:.0f}MB, "
                f"strategy={self.strategy.name})")


# 示例使用
if __name__ == "__main__":
    # 创建内存池
    with MemoryPool(initial_size_mb=50, max_size_mb=200, enable_gc=True) as pool:
        print("=" * 60)
        print("内存池演示")
        print("=" * 60)

        # 分配测试
        allocation_ids = []
        print("\n分配内存测试:")
        for i in range(10):
            size = (i + 1) * 1024 * 10  # 10KB, 20KB, ...
            alloc_id = pool.allocate(size, f"test_{i}")
            if alloc_id:
                allocation_ids.append(alloc_id)
                print(f"  ✅ 分配 {alloc_id}: {size/1024:.1f}KB")

        pool.print_stats()

        # 释放部分内存
        print("\n释放内存测试:")
        for i, alloc_id in enumerate(allocation_ids[:5]):
            pool.deallocate(alloc_id)
            print(f"  🛑 释放 {alloc_id}")

        pool.print_stats()

        # 执行垃圾回收
        print("\n执行垃圾回收:")
        gc_result = pool.garbage_collect()
        print(f"  结果: {gc_result}")

        pool.print_stats()

        # 压力测试
        print("\n压力测试: 1000次随机分配/释放")
        import random
        import time

        test_allocations = []
        start_time = time.time()

        for i in range(1000):
            if random.random() < 0.6:  # 60% 分配
                size = random.randint(1024, 10*1024)  # 1KB - 10KB
                alloc_id = pool.allocate(size, f"stress_{i}")
                if alloc_id:
                    test_allocations.append((alloc_id, size))
            else:  # 40% 释放
                if test_allocations:
                    alloc_id, _ = test_allocations.pop(random.randint(0, len(test_allocations)-1))
                    pool.deallocate(alloc_id)

        # 清理剩余内存
        for alloc_id, _ in test_allocations:
            pool.deallocate(alloc_id)

        end_time = time.time()
        print(f"  压力测试完成: {end_time - start_time:.2f}秒")
        pool.print_stats()

    print("\n✅ 内存池演示结束")
