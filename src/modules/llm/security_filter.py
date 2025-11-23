#!/usr/bin/env python3.10
# -*- coding: utf-8 -*-
"""
安全过滤器模块 - XLeBot LLM安全过滤
提供内容安全检查、敏感信息过滤、恶意输入检测等功能
"""

import re
import logging
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RiskLevel(Enum):
    """风险等级枚举"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"

@dataclass
class SecurityResult:
    """安全检查结果"""
    is_safe: bool
    risk_level: RiskLevel
    risk_score: float  # 0.0-1.0
    detected_issues: List[str]
    filtered_content: Optional[str] = None
    confidence: float = 0.0

class SecurityFilter:
    """安全过滤器主类"""

    def __init__(self):
        # 敏感词列表（示例）
        self.sensitive_words = [
            # 政治敏感词
            "政治", "政府", "选举", "抗议", "革命",
            # 暴力词汇
            "杀死", "暴力", "攻击", "伤害", "威胁",
            # 色情词汇
            "色情", "性", "裸体", "成人",
            # 违法词汇
            "毒品", "赌博", "诈骗", "黑客", "破解"
        ]

        # 个人信息识别正则
        self.personal_info_patterns = {
            "phone": r'1[3-9]\d{9}',
            "email": r'\b[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Z|a-z]{2,}\b',
            "id_card": r'\b[1-9]\d{5}(19|20)\d{2}(0[1-9]|1[0-2])(0[1-9]|[12]\d|3[01])\d{3}[\dXx]\b',
            "bank_card": r'\b\d{16,19}\b',
            "address": r'(省|市|区|县|镇|街道|路|号)',
            "password": r'(密码|password|pwd).{0,10}[:：=]\s*\S+'
        }

        # 恶意指令模式
        self.malicious_patterns = [
            r'(?i)(system|exec|eval|import|__import__)',
            r'(?i)(rm\s+-rf|del\s+/format|format\s+c:)',
            r'(?i)(sudo|su\s+root|admin|administrator)',
            r'(?i)(\$\(|`|;|&&|\|\||&|\|)',
            r'(?i)(cat\s+/etc/passwd|cat\s+/etc/shadow)',
            r'(?i)(wget|curl|nc|netcat|telnet)',
            r'(?i)(python|perl|ruby|bash|sh|powershell)\s+'
        ]

        # SQL注入模式
        self.sql_injection_patterns = [
            r'(?i)(union\s+select|select\s+.*\s+from)',
            r'(?i)(drop\s+table|delete\s+from|insert\s+into)',
            r'(?i)(update\s+.*\s+set|create\s+table)',
            r"(?i)('.*'|\".*\"|;\s*|\/\*.*\*\/|--.*$)",
            r'(?i)(or\s+1\s*=\s*1|and\s+1\s*=\s*1)',
            r'(?i)(exec\s*\(|execute\s*\()'
        ]

        logger.info("安全过滤器初始化完成")

    def check_input_safety(self, user_input: str) -> SecurityResult:
        """检查用户输入的安全性"""
        risk_score = 0.0
        detected_issues = []

        # 1. 敏感词检查
        sensitive_score, sensitive_issues = self._check_sensitive_words(user_input)
        risk_score += sensitive_score * 0.3
        detected_issues.extend(sensitive_issues)

        # 2. 个人信息检查
        info_score, info_issues = self._check_personal_info(user_input)
        risk_score += info_score * 0.2
        detected_issues.extend(info_issues)

        # 3. 恶意指令检查
        malicious_score, malicious_issues = self._check_malicious_commands(user_input)
        risk_score += malicious_score * 0.3
        detected_issues.extend(malicious_issues)

        # 4. SQL注入检查
        sql_score, sql_issues = self._check_sql_injection(user_input)
        risk_score += sql_score * 0.2
        detected_issues.extend(sql_issues)

        # 确定风险等级
        risk_level = self._determine_risk_level(risk_score)

        # 生成过滤后的内容
        filtered_content = self._filter_content(user_input, detected_issues)

        return SecurityResult(
            is_safe=risk_score < 0.5,
            risk_level=risk_level,
            risk_score=min(risk_score, 1.0),
            detected_issues=detected_issues,
            filtered_content=filtered_content,
            confidence=0.8
        )

    def check_output_safety(self, system_output: str) -> SecurityResult:
        """检查系统输出的安全性"""
        risk_score = 0.0
        detected_issues = []

        # 1. 敏感信息泄露检查
        leak_score, leak_issues = self._check_info_leak(system_output)
        risk_score += leak_score * 0.4
        detected_issues.extend(leak_issues)

        # 2. 有害内容检查
        harmful_score, harmful_issues = self._check_harmful_content(system_output)
        risk_score += harmful_score * 0.3
        detected_issues.extend(harmful_issues)

        # 3. 误导信息检查
        misleading_score, misleading_issues = self._check_misleading_info(system_output)
        risk_score += misleading_score * 0.3
        detected_issues.extend(misleading_issues)

        # 确定风险等级
        risk_level = self._determine_risk_level(risk_score)

        # 生成过滤后的内容
        filtered_content = self._filter_content(system_output, detected_issues)

        return SecurityResult(
            is_safe=risk_score < 0.5,
            risk_level=risk_level,
            risk_score=min(risk_score, 1.0),
            detected_issues=detected_issues,
            filtered_content=filtered_content,
            confidence=0.8
        )

    def _check_sensitive_words(self, text: str) -> Tuple[float, List[str]]:
        """检查敏感词"""
        issues = []
        matched_words = []

        for word in self.sensitive_words:
            if word in text:
                matched_words.append(word)
                issues.append(f"检测到敏感词: {word}")

        if not matched_words:
            return 0.0, issues

        # 根据匹配的敏感词数量和严重程度计算风险分
        risk_score = min(len(matched_words) * 0.2, 1.0)
        return risk_score, issues

    def _check_personal_info(self, text: str) -> Tuple[float, List[str]]:
        """检查个人信息"""
        issues = []
        found_info = []

        for info_type, pattern in self.personal_info_patterns.items():
            matches = re.findall(pattern, text)
            if matches:
                found_info.append(info_type)
                issues.append(f"检测到{info_type}信息")

        if not found_info:
            return 0.0, issues

        # 个人信息风险较高
        risk_score = min(len(found_info) * 0.3, 1.0)
        return risk_score, issues

    def _check_malicious_commands(self, text: str) -> Tuple[float, List[str]]:
        """检查恶意指令"""
        issues = []
        matched_patterns = []

        for pattern in self.malicious_patterns:
            if re.search(pattern, text):
                matched_patterns.append(pattern)
                issues.append("检测到可疑的系统指令")

        if not matched_patterns:
            return 0.0, issues

        # 恶意指令风险很高
        risk_score = min(len(matched_patterns) * 0.4, 1.0)
        return risk_score, issues

    def _check_sql_injection(self, text: str) -> Tuple[float, List[str]]:
        """检查SQL注入"""
        issues = []
        matched_patterns = []

        for pattern in self.sql_injection_patterns:
            if re.search(pattern, text):
                matched_patterns.append(pattern)
                issues.append("检测到可能的SQL注入攻击")

        if not matched_patterns:
            return 0.0, issues

        # SQL注入风险极高
        risk_score = min(len(matched_patterns) * 0.5, 1.0)
        return risk_score, issues

    def _check_info_leak(self, text: str) -> Tuple[float, List[str]]:
        """检查信息泄露"""
        issues = []

        # 检查API密钥模式
        api_key_patterns = [
            r'[A-Za-z0-9]{32,}',  # 长字符串可能是API密钥
            r'[A-Za-z0-9_-]{20,}=[A-Za-z0-9_-]{20,}',  # Base64编码
            r'password\s*[:=]\s*\S+',
            r'token\s*[:=]\s*\S+',
            r'key\s*[:=]\s*\S+',
            r'secret\s*[:=]\s*\S+'
        ]

        for pattern in api_key_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                issues.append("可能包含敏感的密钥信息")
                break

        if not issues:
            return 0.0, issues

        return 0.6, issues

    def _check_harmful_content(self, text: str) -> Tuple[float, List[str]]:
        """检查有害内容"""
        issues = []

        # 检查仇恨言论、歧视内容等
        harmful_keywords = [
            "歧视", "仇恨", "偏见", "侮辱", "诽谤",
            "威胁", "恐吓", "骚扰", "欺凌"
        ]

        for keyword in harmful_keywords:
            if keyword in text:
                issues.append(f"可能包含有害内容: {keyword}")

        if not issues:
            return 0.0, issues

        return 0.4, issues

    def _check_misleading_info(self, text: str) -> Tuple[float, List[str]]:
        """检查误导信息"""
        issues = []

        # 检查可能的误导性表述
        misleading_patterns = [
            r'100%.*保证',
            r'绝对.*安全',
            r'永不.*失败',
            r'包治.*百病',
            r'一夜.*暴富'
        ]

        for pattern in misleading_patterns:
            if re.search(pattern, text):
                issues.append("可能包含误导性信息")
                break

        if not issues:
            return 0.0, issues

        return 0.3, issues

    def _determine_risk_level(self, risk_score: float) -> RiskLevel:
        """确定风险等级"""
        if risk_score < 0.2:
            return RiskLevel.LOW
        elif risk_score < 0.4:
            return RiskLevel.MEDIUM
        elif risk_score < 0.7:
            return RiskLevel.HIGH
        else:
            return RiskLevel.CRITICAL

    def _filter_content(self, content: str, issues: List[str]) -> str:
        """过滤内容"""
        filtered = content

        # 如果存在高风险问题，完全屏蔽内容
        high_risk_keywords = ["恶意指令", "SQL注入", "API密钥", "敏感的密钥"]
        if any(keyword in " ".join(issues) for keyword in high_risk_keywords):
            return "[内容已被安全过滤器屏蔽]"

        # 替换敏感词
        for word in self.sensitive_words:
            if word in filtered:
                filtered = filtered.replace(word, "*" * len(word))

        # 屏蔽个人信息
        for info_type, pattern in self.personal_info_patterns.items():
            filtered = re.sub(pattern, f"[{info_type}信息已屏蔽]", filtered)

        return filtered

    def get_security_report(self, result: SecurityResult) -> Dict[str, Any]:
        """生成安全报告"""
        return {
            "is_safe": result.is_safe,
            "risk_level": result.risk_level.value,
            "risk_score": result.risk_score,
            "confidence": result.confidence,
            "detected_issues": result.detected_issues,
            "recommendations": self._get_recommendations(result)
        }

    def _get_recommendations(self, result: SecurityResult) -> List[str]:
        """获取安全建议"""
        recommendations = []

        if result.risk_level == RiskLevel.CRITICAL:
            recommendations.append("立即阻止此请求，存在严重安全风险")
        elif result.risk_level == RiskLevel.HIGH:
            recommendations.append("建议拒绝此请求，存在较高安全风险")
        elif result.risk_level == RiskLevel.MEDIUM:
            recommendations.append("谨慎处理此请求，建议人工审核")
        else:
            recommendations.append("请求相对安全，可正常处理")

        # 根据具体问题给出建议
        issue_text = " ".join(result.detected_issues).lower()
        if "敏感词" in issue_text:
            recommendations.append("建议用户使用更合适的词汇")
        if "个人信息" in issue_text:
            recommendations.append("提醒用户不要泄露个人信息")
        if "恶意指令" in issue_text:
            recommendations.append("检查用户意图，可能存在恶意行为")

        return recommendations

    def get_stats(self) -> Dict[str, Any]:
        """获取安全过滤器统计信息"""
        return {
            'checked_count': getattr(self, 'checked_count', 0),
            'blocked_count': getattr(self, 'blocked_count', 0),
            'sensitive_words_count': len(self.sensitive_words),
            'patterns_count': len(self.malicious_patterns) + len(self.sql_injection_patterns)
        }

# 简化版本接口（用于向后兼容）
class SimpleSecurityFilter:
    """简化版安全过滤器"""

    def __init__(self):
        self.filter = SecurityFilter()

    def is_safe_input(self, text: str) -> bool:
        """检查输入是否安全"""
        result = self.filter.check_input_safety(text)
        return result.is_safe

    def is_safe_output(self, text: str) -> bool:
        """检查输出是否安全"""
        result = self.filter.check_output_safety(text)
        return result.is_safe

    def filter_text(self, text: str) -> str:
        """过滤文本内容"""
        input_result = self.filter.check_input_safety(text)
        return input_result.filtered_content or text

    def get_security_level(self, text: str) -> str:
        """获取安全等级"""
        input_result = self.filter.check_input_safety(text)
        return input_result.risk_level.value

# 全局安全过滤器实例
security_filter = SecurityFilter()

def get_security_filter() -> SecurityFilter:
    """获取安全过滤器实例"""
    return security_filter

# 向后兼容的实例
simple_filter = SimpleSecurityFilter()