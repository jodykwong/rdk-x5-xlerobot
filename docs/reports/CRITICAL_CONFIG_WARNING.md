# ⚠️ 关键配置警告

## 本项目存在已知的配置陷阱

### 🚨 ASR模块400错误问题（已修复）

**问题根源**：
- 麦克风采样率: 44100 Hz
- 阿里云要求: 16000 Hz
- 必须重采样才能正常工作

**当前状态**：✅ 已修复（2025-11-13）

**如何防止复现**：
1. 修改ASR相关代码前，先阅读 `src/modules/asr/README_CRITICAL.md`
2. 运行 `tools/verify_critical_config.sh` 验证配置
3. 不要禁用音频重采样功能
4. 不要修改AppKey（YOUR_NLS_APPKEY）

**如果400错误再次出现**：
```bash
# 1. 恢复工作版本
git checkout v1.0-asr-working -- src/modules/asr/

# 2. 重启服务测试
./start_voice_assistant.sh restart

# 3. 查看详细排查指南
cat docs/problems/asr_400_error_solution.md
```

---
**创建时间**: 2025-11-13
**重要性**: 🔴 极高