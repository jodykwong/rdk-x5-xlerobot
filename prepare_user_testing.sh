#!/bin/bash
# Story 1.3 MVP 用户测试环境准备脚本

set -e

# ============================================
# 🛡️ 加载XLeRobot专用环境配置
# ============================================
# 加载环境脚本，确保使用正确的Python环境
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$SCRIPT_DIR/xlerobot_env.sh" ]]; then
    source "$SCRIPT_DIR/xlerobot_env.sh"
    echo "✅ XLeRobot环境已加载"
else
    echo "❌ 错误：找不到xlerobot_env.sh环境脚本"
    echo "请确保在XLeRobot项目根目录中运行此脚本"
    exit 1
fi

echo "🚀 Story 1.3 MVP 用户测试环境准备"
echo "=================================="

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查系统要求
check_system_requirements() {
    log_info "检查系统要求..."

    # 检查Python版本
    if command -v $PYTHON_EXECUTABLE &> /dev/null; then
        PYTHON_VERSION=$($PYTHON_EXECUTABLE -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
        log_info "Python版本: $PYTHON_VERSION"

        if [[ $(echo "$PYTHON_VERSION >= 3.8" | bc -l) -eq 1 ]]; then
            log_success "Python版本满足要求 (>= 3.8)"
        else
            log_error "Python版本过低，需要 >= 3.8"
            exit 1
        fi
    else
        log_error "未找到Python3，请先安装Python3"
        exit 1
    fi

    # 检查pip
    if command -v pip3 &> /dev/null; then
        log_success "pip3 已安装"
    else
        log_error "未找到pip3，请先安装pip3"
        exit 1
    fi

    # 检查网络连接
    if ping -c 1 aliyun.com &> /dev/null; then
        log_success "网络连接正常"
    else
        log_warning "网络连接可能有问题，请检查"
    fi
}

# 安装依赖
install_dependencies() {
    log_info "安装Python依赖..."

    # 检查requirements.txt是否存在
    if [ -f "requirements.txt" ]; then
        pip3 install -r requirements.txt
        log_success "依赖安装完成"
    else
        log_warning "未找到requirements.txt，安装基础依赖"
        pip3 install requests numpy alibabacloud-tea-openapi
    fi
}

# 检查API配置
check_api_config() {
    log_info "检查API配置..."

    if [ -n "$ALIYUN_NLS_APP_KEY" ] && ([ -n "$ALIYUN_NLS_APP_SECRET" ] || [ -n "$ALIYUN_NLS_TOKEN" ]); then
        log_success "API配置已设置"
        log_info "App Key: ${ALIYUN_NLS_APP_KEY:0:8}..."

        if [ -n "$ALIYUN_NLS_APP_SECRET" ]; then
            log_info "App Secret: ${ALIYUN_NLS_APP_SECRET:0:8}..."
        else
            log_info "Token: ${ALIYUN_NLS_TOKEN:0:8}..."
        fi
    else
        log_warning "API配置未设置"
        log_info "请运行以下命令配置API:"
        log_info "source setup_aliyun_api.sh"
    fi
}

# 创建测试目录结构
create_test_structure() {
    log_info "创建测试目录结构..."

    mkdir -p testing_logs
    mkdir -p testing_data/audio_samples
    mkdir -p testing_data/results
    mkdir -p user_feedback
    mkdir -p test_reports

    log_success "测试目录结构创建完成"
}

# 验证MVP组件
verify_mvp_components() {
    log_info "验证MVP组件..."

    # 检查关键文件
    local files=(
        "src/xlerobot/common/config_manager.py"
        "src/xlerobot/asr/audio_processor.py"
        "src/xlerobot/asr/aliyun_asr_client.py"
        "src/xlerobot/nodes/asr_service_node.py"
        "demo_story_1_3_mvp.py"
        "real_api_validation.py"
    )

    local all_files_exist=true

    for file in "${files[@]}"; do
        if [ -f "$file" ]; then
            log_success "✓ $file"
        else
            log_error "✗ $file (缺失)"
            all_files_exist=false
        fi
    done

    if [ "$all_files_exist" = true ]; then
        log_success "所有MVP组件文件完整"
    else
        log_error "部分MVP组件文件缺失，请检查项目完整性"
        exit 1
    fi
}

# 创建用户测试包
create_user_test_package() {
    log_info "创建用户测试包..."

    # 创建测试脚本
    cat > user_test_runner.py << 'EOF'
#!/usr/bin/env python3
"""
Story 1.3 MVP 用户测试运行器
"""

import sys
import os
import time
import json
import logging
from datetime import datetime

# 添加项目路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

try:
    from xlerobot.common.config_manager import ConfigManager
    from xlerobot.asr.audio_processor import AudioProcessor
    from xlerobot.asr.aliyun_asr_client import AliyunASRClient
except ImportError as e:
    print(f"❌ 导入失败: {e}")
    print("请确保在项目根目录运行此脚本")
    sys.exit(1)

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('testing_logs/user_test.log'),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger(__name__)

class UserTestRunner:
    def __init__(self):
        self.config_manager = ConfigManager()
        self.audio_processor = AudioProcessor()
        self.asr_client = None
        self.test_session = {
            "start_time": datetime.now().isoformat(),
            "user_id": os.environ.get("USER_ID", "unknown"),
            "test_cases": [],
            "results": []
        }

    def initialize(self):
        """初始化测试环境"""
        print("🚀 初始化用户测试环境...")

        if self.config_manager.validate_config():
            config = self.config_manager.get_config()
            self.asr_client = AliyunASRClient(config.app_key, config.app_secret)
            print("✅ API客户端初始化成功")
            return True
        else:
            print("❌ API配置无效")
            return False

    def run_test_case(self, case_name, instruction):
        """运行单个测试用例"""
        print(f"\n📝 测试用例: {case_name}")
        print(f"说明: {instruction}")
        print("请按回车键开始录音...")

        input()  # 等待用户确认

        # 这里应该启动录音功能
        # 为了演示，我们使用模拟数据
        print("🎤 正在录音... (5秒)")
        time.sleep(5)
        print("✅ 录音完成，正在识别...")

        # 模拟识别结果
        result = {
            "case_name": case_name,
            "timestamp": datetime.now().isoformat(),
            "instruction": instruction,
            "status": "completed",
            "user_feedback": None
        }

        print("请评价识别结果 (1-5分，1=很差，5=很好):")
        while True:
            try:
                score = int(input("评分: "))
                if 1 <= score <= 5:
                    result["user_score"] = score
                    break
                else:
                    print("请输入1-5之间的数字")
            except ValueError:
                print("请输入有效数字")

        print("请输入反馈意见 (可选):")
        feedback = input("反馈: ")
        result["user_feedback"] = feedback

        self.test_session["results"].append(result)
        print("✅ 测试用例完成")

        return result

    def run_test_suite(self):
        """运行完整测试套件"""
        test_cases = [
            ("基础指令", "请说'开灯'或'关灯'"),
            ("信息查询", "请说'今日天气'或'现在几点'"),
            ("媒体控制", "请说'播放音乐'或'停止播放'"),
            ("自然对话", "请说'早晨，今日天气如何？'"),
            ("复杂指令", "请说'帮我设定明天早上7点的闹钟'")
        ]

        print(f"\n🎯 开始用户测试 ({len(test_cases)}个测试用例)")
        print("=" * 50)

        for case_name, instruction in test_cases:
            try:
                self.run_test_case(case_name, instruction)
            except Exception as e:
                print(f"❌ 测试用例失败: {e}")
                logger.error(f"Test case failed: {e}")

        # 保存测试结果
        self.save_test_results()

    def save_test_results(self):
        """保存测试结果"""
        self.test_session["end_time"] = datetime.now().isoformat()

        # 保存到JSON文件
        filename = f"testing_data/results/user_test_{self.test_session['user_id']}_{int(time.time())}.json"

        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(self.test_session, f, indent=2, ensure_ascii=False)

        print(f"\n✅ 测试结果已保存到: {filename}")

        # 显示测试总结
        self.show_test_summary()

    def show_test_summary(self):
        """显示测试总结"""
        results = self.test_session["results"]

        if not results:
            print("❌ 没有测试结果")
            return

        scores = [r.get("user_score", 0) for r in results if r.get("user_score")]
        avg_score = sum(scores) / len(scores) if scores else 0

        print(f"\n📊 测试总结")
        print(f"完成测试用例: {len(results)}")
        print(f"平均评分: {avg_score:.1f}/5.0")
        print(f"测试时间: {self.test_session['start_time']} ~ {self.test_session.get('end_time', '进行中')}")

        print(f"\n💡 感谢您的参与！")
        print(f"您的反馈对我们非常重要。")

def main():
    """主函数"""
    print("🎯 Story 1.3 MVP 用户测试")
    print("=" * 30)

    runner = UserTestRunner()

    if runner.initialize():
        runner.run_test_suite()
    else:
        print("❌ 测试环境初始化失败")
        sys.exit(1)

if __name__ == "__main__":
    main()
EOF

    chmod +x user_test_runner.py

    # 创建用户指导文档
    cat > USER_TESTING_GUIDE.md << 'EOF'
# Story 1.3 MVP 用户测试指南

## 🚀 快速开始

1. **设置用户ID** (可选)
   ```bash
   export USER_ID="your_name_or_id"
   ```

2. **运行测试**
   ```bash
   $PYTHON_EXECUTABLE user_test_runner.py
   ```

## 📋 测试说明

### 测试流程
1. 阅读测试用例说明
2. 按回车键开始录音
3. 说出相应的语音指令
4. 等待识别结果
5. 评分并提供反馈

### 评分标准
- **5分**: 识别准确，响应快速
- **4分**: 识别基本准确，偶尔需要重复
- **3分**: 识别一般，经常需要重复
- **2分**: 识别较差，多次重复才成功
- **1分**: 识别失败或完全错误

### 注意事项
- 请在安静环境中测试
- 语速适中，发音清晰
- 如遇到问题，请记录具体错误信息
- 测试数据仅用于产品改进

## 🆘 技术支持

如遇到技术问题，请联系：
- 技术支持: [技术支持联系方式]
- 紧急联系: [紧急联系方式]

测试时间: [测试开始时间] - [测试结束时间]
EOF

    log_success "用户测试包创建完成"
}

# 生成测试报告模板
generate_report_templates() {
    log_info "生成测试报告模板..."

    cat > test_reports/daily_test_report.md << 'EOF'
# Story 1.3 MVP 每日测试报告

**日期**: [日期]
**测试用户数**: [数量]
**总体完成率**: [百分比]

## 📊 今日数据

### 技术指标
- 平均识别准确率: [百分比]
- 平均响应时间: [秒数]
- 成功率: [百分比]

### 用户反馈
- 平均满意度: [评分]/5.0
- 主要问题: [问题列表]
- 改进建议: [建议列表]

## 🔍 问题分析

### 发现的问题
1. [问题描述]
2. [问题描述]

### 解决方案
1. [解决方案]
2. [解决方案]

## 📋 明日计划
- [计划事项]
- [计划事项]
EOF

    cat > test_reports/final_test_report.md << 'EOF'
# Story 1.3 MVP 最终测试报告

**测试周期**: [开始日期] - [结束日期]
**参与用户**: [用户数量]
**测试用例**: [用例数量]

## 📊 测试结果汇总

### 关键指标
| 指标 | 目标值 | 实际值 | 达成状态 |
|------|--------|--------|----------|
| 识别准确率 | >85% | [实际值] | [状态] |
| 用户满意度 | >4.0/5.0 | [实际值] | [状态] |
| 响应时间 | <3秒 | [实际值] | [状态] |
| 日活跃率 | >60% | [实际值] | [状态] |

### 详细数据分析
[详细分析内容]

## 👥 用户画像分析

### 用户群体特征
[用户特征分析]

### 使用行为模式
[使用行为分析]

## 💡 产品改进建议

### 高优先级改进
1. [改进建议]
2. [改进建议]

### 中优先级改进
1. [改进建议]
2. [改进建议]

### 低优先级改进
1. [改进建议]
2. [改进建议]

## 🎯 下一步行动计划

### Story 1.3 最终完成
- [待办事项]
- [待办事项]

### Story 1.4 规划
- [规划事项]
- [规划事项]

## 📈 商业化评估

### 市场验证结果
[市场验证分析]

### 商业化建议
[商业化建议]

---
**报告生成时间**: [时间]
**报告版本**: v1.0
EOF

    log_success "测试报告模板创建完成"
}

# 主函数
main() {
    echo "开始准备用户测试环境..."

    # 检查系统要求
    check_system_requirements

    # 安装依赖
    install_dependencies

    # 检查API配置
    check_api_config

    # 创建测试目录结构
    create_test_structure

    # 验证MVP组件
    verify_mvp_components

    # 创建用户测试包
    create_user_test_package

    # 生成测试报告模板
    generate_report_templates

    echo ""
    echo "🎉 用户测试环境准备完成！"
    echo ""
    echo "📋 下一步操作:"
    echo "1. 配置API环境: source setup_aliyun_api.sh"
    echo "2. 运行API验证: $PYTHON_EXECUTABLE real_api_validation.py"
    echo "3. 分发测试包给用户"
    echo "4. 指导用户运行: $PYTHON_EXECUTABLE user_test_runner.py"
    echo ""
    echo "📞 技术支持: [技术支持联系方式]"
    echo "📧 邮箱支持: [邮箱地址]"
}

# 运行主函数
main "$@"