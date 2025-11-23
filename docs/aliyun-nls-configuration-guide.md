# 阿里云NLS服务配置指南
## XleRobot Story 1.1 - 语音唤醒和基础识别

**文档编号**: XLR-CONFIG-ALIYUN-20251108-001
**项目名称**: XleRobot 家用机器人控制系统
**Story ID**: 1.1
**创建日期**: 2025-11-08
**适用范围**: 阿里云智能语音交互服务配置
**Brownfield级别**: Level 4 企业级

---

## 📋 概述

本文档详细说明如何配置阿里云智能语音交互(NLS)服务，为 XleRobot Story 1.1 的语音唤醒和基础识别功能提供技术支持。

## 🎯 配置目标

- ✅ 支持16kHz采样率的音频输入
- ✅ 粤语语音识别（默认"傻强"唤醒词）
- ✅ 普通话语音识别
- ✅ 混合语言识别能力
- ✅ 高准确性（>90%）和低延迟（<2秒）
- ✅ Brownfield Level 4 企业级合规

---

## 🔐 前置条件

### 1. 阿里云账号准备

1. **注册阿里云账号**
   - 访问 [阿里云官网](https://www.aliyun.com/)
   - 注册并完成实名认证

2. **开通智能语音交互服务**
   - 进入 [智能语音交互产品页面](https://www.aliyun.com/product/nls)
   - 选择"语音合成"和"语音识别"服务
   - 完成服务开通和计费配置

3. **创建项目**
   - 登录 [智能语音交互控制台](https://nls-portal.console.aliyun.com/)
   - 创建新项目
   - 记录项目AppKey

### 2. 访问权限配置

1. **创建RAM用户**（推荐）
   - 进入 [RAM控制台](https://ram.console.aliyun.com/)
   - 创建RAM用户
   - 分配"AliyunNLSFullAccess"权限

2. **生成AccessKey**
   - 为RAM用户创建AccessKey
   - 记录AccessKey ID和AccessKey Secret

---

## 📝 配置文件填写

### 配置文件位置
```
/home/sunrise/xlerobot/config/aliyun_nls_config.yaml
```

### 必填配置项

#### 1. 项目AppKey
```yaml
authentication:
  appkey: "YOUR_APPKEY_HERE"  # 📝 填写你的AppKey
```

**获取方法：**
1. 登录智能语音交互控制台
2. 进入"项目管理"页面
3. 选择你的项目
4. 复制"AppKey"值

#### 2. 访问密钥
```yaml
authentication:
  token:
    access_key_id: "YOUR_ACCESS_KEY_ID_HERE"      # 📝 填写你的AccessKey ID
    access_key_secret: "YOUR_ACCESS_KEY_SECRET_HERE"  # 📝 填写你的AccessKey Secret
```

**获取方法：**
1. 进入RAM控制台
2. 选择你的RAM用户
3. 点击"创建AccessKey"
4. 安全记录AccessKey ID和Secret

### 可选配置项

#### 1. 服务端点选择
```yaml
service_endpoints:
  intelligent:
    websocket: "wss://nls-gateway.aliyuncs.com/ws/v1"  # 推荐：智能就近接入
  # 或指定地域：
  regions:
    shanghai:
      websocket: "wss://nls-gateway-cn-shanghai.aliyuncs.com/ws/v1"
    beijing:
      websocket: "wss://nls-gateway-cn-beijing.aliyuncs.com/ws/v1"
```

#### 2. 发音人配置
```yaml
tts:
  voice: "xiaoyun"  # 默认女声

  # 粤语发音人选项
  cantonese_voices:
    female: ["shanshan", "jiajia", "taozi"]
    male: ["abin"]
    preferred: "shanshan"  # 推荐粤语女声
```

#### 3. ASR语音识别配置
```yaml
asr:
  format: "pcm"           # 音频格式
  sample_rate: 16000      # 采样率（必须与硬件匹配）
  cantonese:
    enabled: true         # 启用粤语识别
  multilingual:
    enabled: true         # 启用混合语言识别
```

---

## 🔧 配置验证步骤

### 1. 运行配置验证脚本
```bash
bash /home/sunrise/xlerobot/scripts/validate_aliyun_config.sh
```

**预期输出：**
```
✅ 配置文件存在
✅ ROS2命令可用
✅ AppKey已配置
✅ Access Key ID已配置
✅ Access Key Secret已配置
✅ WebSocket端点已配置
✅ ASR服务已启用
✅ TTS服务已启用
🎉 恭喜！所有配置检查都通过了！
```

### 2. 运行连接测试脚本
```bash
bash /home/sunrise/xlerobot/scripts/test_aliyun_connection.sh
```

**预期输出：**
```
✅ 配置文件加载成功
✅ Token获取测试成功
✅ DNS解析成功
✅ HTTPS连接测试成功
✅ 音频输入设备可用
🎉 所有连接测试都通过了！
```

---

## 🚨 安全注意事项

### 1. 密钥安全
- ❌ **禁止** 将配置文件提交到版本控制系统
- ✅ **推荐** 使用环境变量或密钥管理系统
- ✅ **定期** 轮换AccessKey
- ✅ **最小化** RAM用户权限

### 2. 环境变量配置（可选）
```bash
export ALIYUN_NLS_APPKEY="your_appkey"
export ALIYUN_NLS_ACCESS_KEY_ID="your_access_key_id"
export ALIYUN_NLS_ACCESS_KEY_SECRET="your_access_key_secret"
```

### 3. 文件权限
```bash
chmod 600 /home/sunrise/xlerobot/config/aliyun_nls_config.yaml
```

---

## 🛠️ 故障排除

### 常见问题

#### 1. Token获取失败
**错误信息：** `40000001 The token has expired`

**解决方案：**
- 检查AccessKey ID和Secret是否正确
- 确认RAM用户有NLS访问权限
- 验证网络连接

#### 2. WebSocket连接失败
**错误信息：** `Connection refused`

**解决方案：**
- 检查防火墙设置
- 确认WebSocket端点正确
- 验证网络连通性

#### 3. ASR识别不准确
**可能原因：**
- 采样率配置不匹配
- 音频质量较差
- 环境噪声干扰

**解决方案：**
- 确保采样率设置为16000Hz
- 使用高质量麦克风
- 启用噪声抑制

#### 4. TTS合成失败
**错误信息：** `40000001 No privilege to this voice!`

**解决方案：**
- 检查发音人名称是否正确
- 确认服务包含该发音人
- 检查服务版本是否支持

### 调试模式

启用调试模式获取详细日志：
```yaml
development:
  debug:
    enabled: true
    log_requests: true
    log_responses: true
```

---

## 📞 技术支持

### 配置相关问题
- **配置验证脚本**：`/home/sunrise/xlerobot/scripts/validate_aliyun_config.sh`
- **连接测试脚本**：`/home/sunrise/xlerobot/scripts/test_aliyun_connection.sh`

### 阿里云技术支持
- **官方文档**：[智能语音交互文档](https://help.aliyun.com/zh/isi/)
- **技术支持**：[阿里云工单系统](https://selfservice.console.aliyun.com/ticket/create.htm)
- **社区支持**：[阿里云开发者社区](https://developer.aliyun.com/)

### 项目相关支持
- **项目配置**：`/home/sunrise/xlerobot/config/`
- **日志文件**：`/var/log/xlerobot/aliyun_nls.log`
- **测试音频**：`/home/sunrise/xlerobot/test_audio/`

---

## 📋 配置检查清单

### 基础配置
- [ ] 阿里云账号已注册并实名认证
- [ ] 智能语音交互服务已开通
- [ ] 项目AppKey已获取
- [ ] RAM用户已创建并分配权限
- [ ] AccessKey已生成

### 配置文件
- [ ] 配置文件已创建
- [ ] AppKey已填写
- [ ] Access Key ID已填写
- [ ] Access Key Secret已填写
- [ ] 服务端点已配置
- [ ] 音频参数已设置

### 粤语优化
- [ ] 粤语识别已启用
- [ ] 粤语发音人已配置
- [ ] 混合语言识别已启用
- [ ] 采样率设置为16kHz

### 验证测试
- [ ] 配置验证脚本通过
- [ ] 连接测试脚本通过
- [ ] Token获取正常
- [ ] WebSocket连接正常
- [ ] 音频设备可用

---

## 📚 参考文档

- [阿里云智能语音交互产品文档](https://help.aliyun.com/zh/isi/)
- [语音合成API文档](https://help.aliyun.com/zh/isi/developer-reference/overview-of-speech-synthesis)
- [语音识别API文档](https://help.aliyun.com/zh/isi/developer-reference/overview-of-speech-recognition)
- [音色列表](https://help.aliyun.com/zh/isi/product-overview/tts-person)
- [计费说明](https://help.aliyun.com/zh/isi/product-overview/pricing)

---

**文档状态**：✅ 已完成
**审核状态**：待审核
**版本**：1.0
**最后更新**：2025-11-08

*本文档严格遵循Brownfield Level 4企业级标准，为XleRobot项目提供完整的阿里云NLS服务配置指导。通过详细的配置步骤、验证方法和故障排除指南，确保语音识别和合成功能的稳定运行。*