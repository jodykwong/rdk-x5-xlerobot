# ⚠️ ASR模块关键配置说明

## 🚨 修改此模块前必读

本模块已完成以下关键修复，**任何修改都可能导致400错误复现**：

### ✅ 已解决的问题（2025-11-13）
1. **ASR API 400错误** - 采样率不匹配
2. **Token获取失败** - 使用官方SDK
3. **音频格式错误** - 自动重采样

### 🔒 关键配置（禁止修改）

#### 1. 音频重采样配置
```python
# simple_aliyun_asr_service.py
# ✅ 正确配置：
input_sample_rate = 44100   # 麦克风实际采样率
target_sample_rate = 16000  # 阿里云要求
enable_resample = True      # 必须启用

# ❌ 错误配置：
# - 不重采样直接发送44100Hz音频
# - target_sample_rate != 16000
# - enable_resample = False
```

#### 2. AppKey配置
```python
# ✅ 正确：
APP_KEY = "YOUR_NLS_APPKEY"  # ASR+TTS项目的AppKey

# ❌ 错误：
# - 使用其他项目的AppKey
# - AppKey有空格或换行符
```

#### 3. Token获取方式
```python
# ✅ 正确：使用阿里云官方SDK
from nls.token import getToken

# ❌ 错误：
# - 手动构造HTTP请求
# - 使用过期的Token
```

### 📋 修改前检查清单

在修改本模块任何代码前，必须：
- [ ] 阅读本文档
- [ ] 运行 `tools/verify_critical_config.sh` 验证当前配置
- [ ] 备份当前工作版本
- [ ] 修改后重新验证配置
- [ ] 测试无400错误后再提交

### 🔄 如何恢复工作版本

如果修改后出现问题：
```bash
# 恢复到工作版本
git checkout v1.0-asr-working -- src/modules/asr/

# 或查看工作配置快照
cat config/snapshots/working_config_*.txt
```

### 📞 故障排除

如果再次出现400错误：
1. 检查重采样是否被禁用
2. 检查AppKey是否被修改
3. 检查Token获取方式是否改变
4. 运行 `tools/verify_critical_config.sh`

---
**最后更新**: 2025-11-13
**状态**: ✅ 功能正常
**维护人**: Jody