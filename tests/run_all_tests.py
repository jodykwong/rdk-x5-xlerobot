#!/usr/bin/env python3.10
"""
XLeRobot 测试套件管理器

自动发现、分类和运行所有测试文件，生成详细的测试报告。
支持按类别运行测试，提供选择性执行功能。

作者: XLeRobot团队
版本: 2.0
日期: 2025-11-15
"""

import os
import sys
import time
import subprocess
import argparse
import json
from pathlib import Path
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
from enum import Enum

# 添加项目路径
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


class TestCategory(Enum):
    """测试类别枚举"""
    INTEGRATION = "integration"
    UNIT = "unit"
    API = "api"
    PIPELINE = "pipeline"
    TOOLS = "tools"
    SPECIALIZED = "specialized"
    ALL = "all"


class TestStatus(Enum):
    """测试状态枚举"""
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    SKIPPED = "skipped"
    ERROR = "error"


@dataclass
class TestResult:
    """测试结果数据类"""
    name: str
    category: TestCategory
    file_path: str
    status: TestStatus
    duration: float
    output: str
    error: Optional[str] = None
    exit_code: Optional[int] = None


@dataclass
class TestSuite:
    """测试套件数据类"""
    name: str
    category: TestCategory
    test_files: List[str]
    description: str


class TestRunner:
    """测试运行器"""

    def __init__(self, project_root: Path):
        self.project_root = project_root
        self.tests_dir = project_root / "tests"
        self.results: List[TestResult] = []
        self.test_suites = self._define_test_suites()

    def _define_test_suites(self) -> Dict[TestCategory, TestSuite]:
        """定义测试套件"""
        return {
            TestCategory.INTEGRATION: TestSuite(
                name="集成测试",
                category=TestCategory.INTEGRATION,
                test_files=[
                    "test_epic1_complete_integration.py",
                    "verify_epic1_complete_functionality.py",
                    "real_epic1_verification.py"
                ],
                description="Epic 1完整功能集成测试，验证ASR->LLM->TTS完整流程"
            ),

            TestCategory.UNIT: TestSuite(
                name="单元测试",
                category=TestCategory.UNIT,
                test_files=[
                    "test_aliyun_api_integration.py",
                    "test_audio_components.py",
                    "test_e2e_integration.py"
                ],
                description="各个模块的单元测试，验证独立功能"
            ),

            TestCategory.API: TestSuite(
                name="API测试",
                category=TestCategory.API,
                test_files=[
                    "test_aliyun_api_integration.py"
                ],
                description="阿里云API连接和服务测试"
            ),

            TestCategory.PIPELINE: TestSuite(
                name="管道测试",
                category=TestCategory.PIPELINE,
                test_files=[
                    "test_complete_pipeline.py",
                    "real_pipeline_test.py",
                    "fixed_real_pipeline_test.py",
                    "test_audio_pipeline.py",
                    "run_voice_assistant_test.py"
                ],
                description="语音处理管道和流程测试"
            ),

            TestCategory.TOOLS: TestSuite(
                name="工具测试",
                category=TestCategory.TOOLS,
                test_files=[
                    "camera_init.py",
                    "quick_verification.py",
                    "simple_epic1_check.py",
                    "story1_2_detailed_analysis.py"
                ],
                description="验证脚本和工具测试"
            ),

            TestCategory.SPECIALIZED: TestSuite(
                name="专项测试",
                category=TestCategory.SPECIALIZED,
                test_files=[
                    "test_dynamic_messages.py",
                    "test_correct_audio_fix.py",
                    "test_ros2_nodes.py"
                ],
                description="特殊功能和修复验证测试"
            )
        }

    def discover_tests(self, category: TestCategory = TestCategory.ALL) -> List[str]:
        """发现测试文件"""
        if category == TestCategory.ALL:
            # 返回所有测试文件
            all_tests = []
            for suite in self.test_suites.values():
                all_tests.extend(suite.test_files)
            return list(set(all_tests))  # 去重
        else:
            suite = self.test_suites.get(category)
            return suite.test_files if suite else []

    def run_single_test(self, test_file: str, timeout: int = 300) -> TestResult:
        """运行单个测试"""
        test_path = self.tests_dir / test_file
        if not test_path.exists():
            return TestResult(
                name=test_file,
                category=self._get_test_category(test_file),
                file_path=str(test_path),
                status=TestStatus.ERROR,
                duration=0.0,
                output="",
                error=f"测试文件不存在: {test_path}",
                exit_code=-1
            )

        print(f"🧪 运行测试: {test_file}")
        start_time = time.time()

        try:
            result = subprocess.run(
                [sys.executable, str(test_path)],
                cwd=self.project_root,
                capture_output=True,
                text=True,
                timeout=timeout
            )

            duration = time.time() - start_time
            status = TestStatus.PASSED if result.returncode == 0 else TestStatus.FAILED

            return TestResult(
                name=test_file,
                category=self._get_test_category(test_file),
                file_path=str(test_path),
                status=status,
                duration=duration,
                output=result.stdout,
                error=result.stderr if result.stderr else None,
                exit_code=result.returncode
            )

        except subprocess.TimeoutExpired:
            duration = time.time() - start_time
            return TestResult(
                name=test_file,
                category=self._get_test_category(test_file),
                file_path=str(test_path),
                status=TestStatus.ERROR,
                duration=duration,
                output="",
                error=f"测试超时 ({timeout}秒)",
                exit_code=-1
            )
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                name=test_file,
                category=self._get_test_category(test_file),
                file_path=str(test_path),
                status=TestStatus.ERROR,
                duration=duration,
                output="",
                error=str(e),
                exit_code=-1
            )

    def _get_test_category(self, test_file: str) -> TestCategory:
        """根据文件名确定测试类别"""
        for category, suite in self.test_suites.items():
            if test_file in suite.test_files:
                return category

        # 默认归类
        if "integration" in test_file.lower():
            return TestCategory.INTEGRATION
        elif "test_" in test_file.lower():
            return TestCategory.UNIT
        else:
            return TestCategory.SPECIALIZED

    def run_tests(self, category: TestCategory = TestCategory.ALL,
                  test_names: Optional[List[str]] = None,
                  timeout: int = 300) -> List[TestResult]:
        """运行测试"""
        self.results.clear()

        if test_names:
            # 运行指定的测试
            test_files = test_names
        else:
            # 运行指定类别的测试
            test_files = self.discover_tests(category)

        if not test_files:
            print(f"⚠️  未找到测试文件 (类别: {category.value})")
            return self.results

        print(f"🚀 开始运行测试套件: {category.value}")
        print(f"📋 找到 {len(test_files)} 个测试文件")
        print("=" * 60)

        for i, test_file in enumerate(test_files, 1):
            print(f"[{i}/{len(test_files)}] ", end="")
            result = self.run_single_test(test_file, timeout)
            self.results.append(result)

            # 打印结果
            status_icon = {
                TestStatus.PASSED: "✅",
                TestStatus.FAILED: "❌",
                TestStatus.ERROR: "💥",
                TestStatus.SKIPPED: "⏭️"
            }.get(result.status, "❓")

            print(f"{status_icon} {test_file} ({result.duration:.2f}s)")

            if result.status == TestStatus.FAILED and result.error:
                print(f"   错误: {result.error[:100]}...")

        return self.results

    def generate_report(self, results: List[TestResult], output_format: str = "console") -> str:
        """生成测试报告"""
        if not results:
            return "没有测试结果"

        passed = sum(1 for r in results if r.status == TestStatus.PASSED)
        failed = sum(1 for r in results if r.status == TestStatus.FAILED)
        errors = sum(1 for r in results if r.status == TestStatus.ERROR)
        total_duration = sum(r.duration for r in results)

        if output_format == "console":
            report = self._generate_console_report(results, passed, failed, errors, total_duration)
        elif output_format == "json":
            report = self._generate_json_report(results, passed, failed, errors, total_duration)
        else:
            report = "不支持的报告格式"

        return report

    def _generate_console_report(self, results: List[TestResult],
                                passed: int, failed: int, errors: int,
                                total_duration: float) -> str:
        """生成控制台报告"""
        report = []
        report.append("=" * 70)
        report.append("🧪 XLeRobot 测试报告")
        report.append("=" * 70)
        report.append(f"📊 测试统计:")
        report.append(f"   总计: {len(results)} 个测试")
        report.append(f"   通过: {passed} ✅")
        report.append(f"   失败: {failed} ❌")
        report.append(f"   错误: {errors} 💥")
        report.append(f"   成功率: {passed/len(results)*100:.1f}%")
        report.append(f"   总耗时: {total_duration:.2f}秒")
        report.append("")

        # 按类别分组
        by_category = {}
        for result in results:
            category = result.category.value
            if category not in by_category:
                by_category[category] = []
            by_category[category].append(result)

        report.append("📋 分类结果:")
        for category, cat_results in by_category.items():
            cat_passed = sum(1 for r in cat_results if r.status == TestStatus.PASSED)
            report.append(f"   {category}: {cat_passed}/{len(cat_results)} 通过")

        report.append("")
        report.append("🔍 详细结果:")
        report.append("-" * 70)

        for result in results:
            status_icon = {
                TestStatus.PASSED: "✅",
                TestStatus.FAILED: "❌",
                TestStatus.ERROR: "💥",
                TestStatus.SKIPPED: "⏭️"
            }.get(result.status, "❓")

            report.append(f"{status_icon} {result.name:<35} ({result.duration:6.2f}s) [{result.category.value}]")

            if result.error:
                report.append(f"   💬 {result.error}")

        report.append("=" * 70)

        # 总结
        if failed == 0 and errors == 0:
            report.append("🎉 所有测试通过！系统状态良好。")
        else:
            report.append(f"⚠️  有 {failed + errors} 个测试失败，需要检查。")

        return "\n".join(report)

    def _generate_json_report(self, results: List[TestResult],
                             passed: int, failed: int, errors: int,
                             total_duration: float) -> str:
        """生成JSON格式报告"""
        report_data = {
            "summary": {
                "total": len(results),
                "passed": passed,
                "failed": failed,
                "errors": errors,
                "success_rate": passed/len(results)*100 if results else 0,
                "total_duration": total_duration,
                "timestamp": time.time()
            },
            "results": [
                {
                    "name": r.name,
                    "category": r.category.value,
                    "status": r.status.value,
                    "duration": r.duration,
                    "exit_code": r.exit_code,
                    "error": r.error,
                    "output": r.output[:1000] if r.output else None  # 限制输出长度
                }
                for r in results
            ]
        }

        return json.dumps(report_data, indent=2, ensure_ascii=False)


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description="XLeRobot测试套件管理器")
    parser.add_argument(
        "--category", "-c",
        choices=[cat.value for cat in TestCategory],
        default="all",
        help="测试类别 (默认: all)"
    )
    parser.add_argument(
        "--tests", "-t",
        nargs="*",
        help="指定要运行的测试文件名"
    )
    parser.add_argument(
        "--timeout", "-to",
        type=int,
        default=300,
        help="单个测试超时时间（秒，默认: 300）"
    )
    parser.add_argument(
        "--output", "-o",
        choices=["console", "json"],
        default="console",
        help="报告输出格式（默认: console）"
    )
    parser.add_argument(
        "--report-file", "-rf",
        help="报告输出文件路径（可选）"
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="列出所有可用的测试文件"
    )

    args = parser.parse_args()

    # 获取项目根目录
    project_root = Path(__file__).parent.parent

    # 创建测试运行器
    runner = TestRunner(project_root)

    if args.list:
        # 列出所有测试文件
        print("📋 可用的测试文件:")
        for category, suite in runner.test_suites.items():
            print(f"\n{category.value.upper()} - {suite.name}:")
            print(f"   描述: {suite.description}")
            print(f"   文件:")
            for test_file in suite.test_files:
                test_path = project_root / "tests" / test_file
                status = "✅" if test_path.exists() else "❌"
                print(f"     {status} {test_file}")
        return

    # 运行测试
    try:
        category = TestCategory(args.category)
        results = runner.run_tests(category, args.tests, args.timeout)

        # 生成报告
        report = runner.generate_report(results, args.output)

        # 输出报告
        print("\n")
        print(report)

        # 保存报告到文件
        if args.report_file:
            report_path = project_root / args.report_file
            with open(report_path, 'w', encoding='utf-8') as f:
                f.write(report)
            print(f"\n📄 报告已保存到: {report_path}")

        # 设置退出码
        failed_count = sum(1 for r in results if r.status in [TestStatus.FAILED, TestStatus.ERROR])
        sys.exit(1 if failed_count > 0 else 0)

    except KeyboardInterrupt:
        print("\n⏹️  测试被用户中断")
        sys.exit(130)
    except Exception as e:
        print(f"\n💥 测试运行器错误: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()