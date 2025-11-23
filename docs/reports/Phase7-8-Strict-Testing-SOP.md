# Phase 7-8 严谨测试标准操作程序 (SOP)

**文档编号**: XLR-TEST-SOP-PHASE7-8-20251114-001
**项目名称**: XleRobot 家用机器人控制系统
**测试阶段**: Phase 7 (程序逻辑检查) + Phase 8 (完整链路测试)
**创建日期**: 2025-11-14
**版本**: v1.0
**严谨级别**: Level 4 企业级严格测试

> **架构**：全在线（阿里云ASR + 通义千问 + 阿里云TTS）+ 本地唤醒词
> **设备**：RDK X5
> **测试环境**: 真实物理环境，非模拟

---

## ⚠️ 关键原则

### 绝对禁止的行为
- ❌ **禁止使用模拟数据**：不能用预设文本代替真实语音
- ❌ **禁止跳过环节**：必须走完整流程
- ❌ **禁止假设成功**：必须有明确的验证证据

### 必须做到的验证
- ✅ **真实麦克风输入**：必须真的对着麦克风说话
- ✅ **真实API调用**：必须调用真实的阿里云/通义千问服务
- ✅ **真实状态转换**：必须验证状态机的每一步转换
- ✅ **真实时间测量**：必须记录每个环节的实际耗时

---

## Phase 7: 程序逻辑和流程深度检查

**目的**：验证程序内部逻辑、状态管理、错误处理的正确性

### 7.1 状态机验证（核心）

**检查内容**：
```bash
# 查找状态机实现文件
find ~/xlerobot -name "*state*" -type f | grep -E "\.(py|yaml|json)$"

# 查看状态机代码
cat ~/xlerobot/src/modules/asr/streaming/state_machine.py

# 确认状态定义
grep -n "class.*State\|IDLE\|LISTENING\|WAKE_DETECTED\|RECOGNIZING" ~/xlerobot/src/modules/asr/streaming/state_machine.py
```

**必须验证的状态转换**：
```
预期状态转换流程：
1. IDLE (初始状态)
   ↓ 检测到唤醒词"傻强"
2. WAKE_DETECTED (唤醒检测)
   ↓ 播放粤语回应 + 延迟0.5秒
3. LISTENING (监听中)
   ↓ 开始录音（3-5秒）
4. RECOGNIZING (识别中)
   ↓ ASR识别 + LLM生成
5. RESPONDING (回应中)
   ↓ TTS播放完成
6. IDLE (返回初始状态)
```

**判断标准**：
- ✅ **Pass**: 代码中有完整的状态定义和转换逻辑
- ❌ **Fail**: 
  - 缺少任何关键状态
  - 状态转换逻辑不完整
  - 没有状态重置机制

### 7.2 ROS2 Topics实时监控

**检查内容**：
```bash
# 启动服务（如果未启动）
# 在终端1执行：
cd ~/xlerobot
source config/.env.sprint1
./start_voice_services_final.sh

# 在新终端2执行：监控关键topics
source /opt/ros/humble/setup.bash

# 监控唤醒词检测topic
ros2 topic echo /wake_word_detected &

# 监控ASR结果topic
ros2 topic echo /asr/result &

# 监控LLM响应topic
ros2 topic echo /llm_response &

# 监控TTS状态topic
ros2 topic echo /tts/status &

# 查看所有活跃topics
ros2 topic list
ros2 topic hz /wake_word_detected
```

**判断标准**：
- ✅ **Pass**: 
  - 所有关键topics都存在
  - topics有发布者和订阅者
  - `ros2 topic hz`显示有消息流动
- ❌ **Fail**: 
  - 任何关键topic缺失
  - topic无发布者或订阅者
  - 长时间无消息

### 7.3 回调函数验证

**检查内容**：
```bash
# 查看ASR服务的回调函数定义
grep -A 10 "def on_start\|def on_result\|def on_error\|def on_completed" \
  ~/xlerobot/src/modules/asr/*.py

# 查看是否有日志输出这些回调
tail -100 ~/xlerobot/logs/asr_service.log | grep -i "callback\|on_start\|on_result"
```

**必须验证的回调函数**：
```python
ASR回调：
- on_start()          # WebSocket连接建立
- on_sentence_begin() # 句子开始
- on_sentence_end()   # 句子结束
- on_result()         # 识别结果
- on_completed()      # 识别完成
- on_error()          # 错误处理
- on_close()          # 连接关闭

TTS回调：
- on_data_received()  # 接收音频数据
- on_completed()      # 合成完成
- on_error()          # 错误处理
```

**判断标准**：
- ✅ **Pass**: 
  - 所有关键回调函数都已定义
  - 回调函数中有日志输出
  - 回调函数有实际的处理逻辑
- ❌ **Fail**:
  - 回调函数只是`pass`空实现
  - 回调中没有状态更新
  - 回调中没有错误处理

### 7.4 错误处理机制验证

**检查内容**：
```bash
# 查看错误处理代码
grep -B 3 -A 10 "try:\|except\|raise\|logging.error" \
  ~/xlerobot/src/modules/asr/*.py | head -50

# 检查是否有重试机制
grep -n "retry\|attempt\|max_retries" ~/xlerobot/src/modules/asr/*.py

# 查看错误日志
tail -100 ~/xlerobot/logs/asr_service.log | grep -i "error\|exception\|fail"
```

**必须验证的错误处理**：
```
1. Token过期处理
   - 自动检测Token过期
   - 自动刷新Token
   - 重试机制

2. 网络异常处理
   - WebSocket连接失败
   - 超时处理
   - 断线重连

3. ASR识别失败处理
   - 识别超时
   - 识别结果为空
   - 置信度过低

4. 状态恢复机制
   - 异常时返回IDLE状态
   - 清理音频缓冲区
   - 释放资源
```

**判断标准**：
- ✅ **Pass**: 
  - 所有关键环节都有try-except
  - 错误时有日志记录
  - 有明确的恢复策略
- ❌ **Fail**:
  - 缺少异常处理
  - 异常后程序崩溃
  - 无错误恢复机制

### 7.5 日志完整性验证

**检查内容**：
```bash
# 查看日志配置
cat ~/xlerobot/config/logging_config.yaml

# 检查日志文件存在性和大小
ls -lh ~/xlerobot/logs/

# 查看最近的日志条目
tail -50 ~/xlerobot/logs/asr_service.log
tail -50 ~/xlerobot/logs/tts_service.log
tail -50 ~/xlerobot/logs/llm_service.log
tail -50 ~/xlerobot/logs/main.log

# 搜索关键事件日志
grep -i "wake.*detect\|asr.*start\|recognition.*complete\|tts.*start" \
  ~/xlerobot/logs/*.log | tail -20
```

**必须存在的关键日志**：
```
启动日志：
- "ASR service started"
- "Token initialized"
- "WebSocket connected"
- "State: IDLE"

运行日志：
- "Wake word detected: 傻强"
- "State transition: IDLE -> WAKE_DETECTED"
- "Playing response: 傻强系度..."
- "State transition: WAKE_DETECTED -> LISTENING"
- "Recording started"
- "Audio data sent to ASR"
- "ASR result received: [文本]"
- "Calling LLM with: [文本]"
- "LLM response received"
- "TTS synthesis started"
- "TTS audio received: [大小]bytes"
- "Playing audio"
- "State transition: RESPONDING -> IDLE"

错误日志（如果有）：
- "Token refresh required"
- "WebSocket connection lost"
- "ASR recognition failed"
```

**判断标准**：
- ✅ **Pass**: 
  - 日志级别至少为INFO
  - 每个关键步骤都有日志
  - 日志包含时间戳和状态信息
- ❌ **Fail**:
  - 日志几乎为空
  - 缺少关键事件的日志
  - 日志级别设置为ERROR（太少）

### 7.6 资源管理验证

**检查内容**：
```bash
# 查看进程资源占用
ps aux | grep python | grep xlerobot

# 监控内存使用
watch -n 1 'ps aux | grep python | grep xlerobot | awk "{sum+=\$6} END {print sum/1024 \"MB\"}"'

# 检查是否有内存泄漏
# 运行前
free -h > /tmp/mem_before.txt

# 运行一段时间后
free -h > /tmp/mem_after.txt

# 对比
diff /tmp/mem_before.txt /tmp/mem_after.txt

# 检查文件描述符
lsof -p $(pgrep -f xlerobot) | wc -l

# 检查WebSocket连接数
netstat -anp | grep python | grep ESTABLISHED | wc -l
```

**判断标准**：
- ✅ **Pass**: 
  - 内存使用稳定（<150MB）
  - 文件描述符数量稳定（<100）
  - WebSocket连接数正常（<=5）
- ❌ **Fail**:
  - 内存持续增长
  - 文件描述符泄漏
  - WebSocket连接过多

---

## Phase 8: 完整链路端到端真实测试

**目的**：用真实语音输入验证完整流程，不允许任何模拟

### 8.1 测试前准备（必须）

**环境准备**：
```bash
# 1. 确保系统运行正常
cd ~/xlerobot
source config/.env.sprint1

# 2. 检查所有服务都在运行
ps aux | grep python | grep xlerobot
ros2 node list

# 3. 清空旧日志（可选，便于查看新测试的日志）
> ~/xlerobot/logs/asr_service.log
> ~/xlerobot/logs/main.log

# 4. 启动日志实时监控（新终端）
tail -f ~/xlerobot/logs/main.log | grep -i "wake\|asr\|llm\|tts\|state"
```

**测试前检查清单**：
- [ ] 麦克风已连接且工作正常（arecord测试）
- [ ] 音箱已连接且工作正常（aplay测试）
- [ ] 网络连接正常（ping测试）
- [ ] Token有效（Phase 2验证）
- [ ] 所有服务进程在运行
- [ ] ROS2 topics活跃

### 8.2 测试执行（严格按步骤）

#### 步骤1：唤醒词触发测试

**测试动作**：
```
1. 对着麦克风清晰地说："傻强"
2. 等待2秒
3. 观察是否有语音回应
```

**同时监控（在另一个终端）**：
```bash
# 监控wake_word_detected topic
ros2 topic echo /wake_word_detected
```

**必须验证的内容**：
- [ ] 听到粤语回应："傻强系度,老细有乜可以帮到你!"
- [ ] 终端显示收到/wake_word_detected消息
- [ ] 日志显示"Wake word detected: 傻强"
- [ ] 日志显示状态转换："IDLE -> WAKE_DETECTED"

**判断标准**：
- ✅ **Pass**: 
  - 300ms内检测到唤醒词
  - 语音回应播放清晰
  - topic消息发布成功
  - 状态转换正确
- ❌ **Fail**: 
  - 无任何响应 → 唤醒词检测未工作
  - 有响应但无topic消息 → ROS2集成问题
  - 状态未转换 → 状态机问题

**失败处理**：
- 如果此步骤失败，**必须停止测试**，返回Phase 6排查唤醒词问题

#### 步骤2：语音识别测试

**测试动作**：
```
1. 听到"傻强系度..."回应后
2. 等待0.5秒（程序准备录音）
3. 清晰说出粤语："今日天气点样？"
4. 停止说话，等待识别
```

**同时监控**：
```bash
# 监控ASR结果topic
ros2 topic echo /asr/result
```

**必须验证的内容**：
- [ ] 录音过程中看到日志："Recording started"
- [ ] 录音结束看到日志："Audio data sent to ASR"
- [ ] 收到ASR识别结果topic消息
- [ ] 日志显示识别文本："今日天氣點樣？"（繁体）
- [ ] 日志显示状态转换："LISTENING -> RECOGNIZING"

**判断标准**：
- ✅ **Pass**: 
  - 识别延迟<3秒
  - 识别文本基本准确（允许细微差异）
  - 识别结果是繁体字
  - topic消息包含text字段
- ❌ **Fail**:
  - 超过5秒无结果 → ASR超时
  - 识别文本完全错误 → 音频质量或ASR配置问题
  - 无topic消息 → ROS2或ASR服务问题

**失败处理**：
- 如果识别为空，检查音频是否真的传给了ASR
- 如果识别错误，尝试说更清晰的普通话再测试一次
- 如果完全无响应，返回Phase 3排查ASR服务

#### 步骤3：LLM调用测试

**无需人工操作**，程序自动执行

**同时监控**：
```bash
# 监控LLM响应topic
ros2 topic echo /llm_response
```

**必须验证的内容**：
- [ ] 日志显示："Calling LLM with: 今日天氣點樣？"
- [ ] 日志显示："LLM API request sent"
- [ ] 收到/llm_response topic消息
- [ ] 日志显示LLM回复内容
- [ ] 回复内容合理且相关

**判断标准**：
- ✅ **Pass**: 
  - LLM响应延迟<10秒
  - 回复内容与问题相关
  - 回复是中文
  - topic消息正常发布
- ❌ **Fail**:
  - 超过15秒无响应 → LLM超时
  - 回复不相关 → LLM配置或prompt问题
  - API错误 → 返回Phase 4排查

**失败处理**：
- 检查通义千问API Key是否有效
- 查看日志中的API错误信息
- 验证网络连接

#### 步骤4：TTS合成和播放测试

**无需人工操作**，程序自动执行

**同时监控**：
```bash
# 监控TTS状态
ros2 topic echo /tts/status
```

**必须验证的内容**：
- [ ] 日志显示："TTS synthesis started"
- [ ] 日志显示："TTS audio received: [数字]bytes"
- [ ] 听到清晰的粤语语音播放
- [ ] 语音内容与LLM回复一致
- [ ] 播放完成后看到："TTS playback completed"

**判断标准**：
- ✅ **Pass**: 
  - TTS合成延迟<3秒
  - 语音清晰且是粤语
  - 音量适中
  - 播放完整无中断
- ❌ **Fail**:
  - 无语音播放 → TTS或播放设备问题
  - 语音不是粤语 → TTS voice参数错误
  - 播放中断 → 音频设备问题

**失败处理**：
- 检查音箱连接
- 返回Phase 5排查TTS服务
- 检查voice参数是否为粤语发音人

#### 步骤5：状态重置验证

**无需人工操作**，自动验证

**必须验证的内容**：
- [ ] 播放完成后，日志显示："State transition: RESPONDING -> IDLE"
- [ ] 系统回到等待唤醒状态
- [ ] 可以再次说"傻强"触发新一轮对话

**判断标准**：
- ✅ **Pass**: 
  - 状态成功返回IDLE
  - 可以立即进行下一轮测试
  - 无资源泄漏
- ❌ **Fail**:
  - 状态卡住 → 状态机问题
  - 无法再次触发 → 资源未释放

### 8.3 连续对话测试（3轮）

**目的**：验证系统稳定性和状态管理

**测试动作**：
```
第1轮：
  说："傻强"
  等待回应
  说："你好呀"
  等待回复和播放
  
第2轮（立即开始）：
  说："傻强"
  等待回应
  说："而家几点？"
  等待回复和播放
  
第3轮（立即开始）：
  说："傻强"
  等待回应
  说："谢谢你"
  等待回复和播放
```

**判断标准**：
- ✅ **Pass**: 3轮全部成功，无卡顿或异常
- ⚠️ **Warning**: 第2或第3轮有延迟（>2秒等待）
- ❌ **Fail**: 第2轮开始失败

### 8.4 性能指标测量

**测量工具准备**：
```bash
# 创建性能测量脚本
cat > /tmp/measure_performance.py << 'EOF'
import re
import sys

log_file = sys.argv[1]

with open(log_file) as f:
    logs = f.read()

# 提取时间戳
wake_time = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}).*Wake word detected', logs)
asr_time = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}).*ASR result received', logs)
llm_time = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}).*LLM response received', logs)
tts_time = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}).*TTS synthesis started', logs)
play_time = re.search(r'(\d{2}:\d{2}:\d{2}\.\d{3}).*TTS playback completed', logs)

if all([wake_time, asr_time, llm_time, tts_time, play_time]):
    # 计算各阶段耗时
    from datetime import datetime
    fmt = "%H:%M:%S.%f"
    
    t_wake = datetime.strptime(wake_time.group(1), fmt)
    t_asr = datetime.strptime(asr_time.group(1), fmt)
    t_llm = datetime.strptime(llm_time.group(1), fmt)
    t_tts = datetime.strptime(tts_time.group(1), fmt)
    t_play = datetime.strptime(play_time.group(1), fmt)
    
    print(f"唤醒检测: {wake_time.group(1)}")
    print(f"ASR识别: {(t_asr - t_wake).total_seconds():.2f}秒")
    print(f"LLM生成: {(t_llm - t_asr).total_seconds():.2f}秒")
    print(f"TTS合成: {(t_tts - t_llm).total_seconds():.2f}秒")
    print(f"播放完成: {(t_play - t_tts).total_seconds():.2f}秒")
    print(f"总耗时: {(t_play - t_wake).total_seconds():.2f}秒")
else:
    print("日志不完整，无法测量")
EOF

chmod +x /tmp/measure_performance.py
```

**执行测量**：
```bash
# 完成一轮完整测试后
python3 /tmp/measure_performance.py ~/xlerobot/logs/main.log
```

**性能目标**：
```
唤醒检测: < 0.2秒
ASR识别: < 3秒
LLM生成: < 10秒
TTS合成: < 3秒
播放完成: < 5秒（视回复长度）
-------------------------
总耗时: < 25秒 ✅ 可接受
总耗时: 25-40秒 ⚠️ 需优化
总耗时: > 40秒 ❌ 不可接受
```

### 8.5 边界条件测试

#### 测试1：静音超时
```
说："傻强"
等待回应
不说话（静音30秒）

预期：
- 系统应在3-5秒后超时
- 日志显示："Silence timeout"
- 状态返回IDLE
- 可以再次触发
```

#### 测试2：噪音环境
```
播放背景音乐（音量适中）
说："傻强"
说："你好"

预期：
- 唤醒词识别率>70%
- ASR能识别基本内容
- 可能有误识别但系统不崩溃
```

#### 测试3：含糊不清
```
说："傻强"
说："啊啊啊啊"（含糊）

预期：
- ASR返回识别结果（可能为空）
- 系统不崩溃
- LLM有默认回复
- 状态正常返回IDLE
```

#### 测试4：长时间运行
```
连续测试10轮对话（每轮间隔1分钟）

监控：
- 内存使用是否稳定
- 响应时间是否变慢
- 是否有错误累积
```

### 8.6 测试记录表

**每次测试填写此表**：

```
测试时间: ______________________
测试人员: ______________________
环境描述: ______________________

| 测试项目 | 结果 | 耗时 | 问题描述 |
|---------|------|------|---------|
| 唤醒词触发 | □Pass □Fail | ___秒 | |
| 粤语回应播放 | □Pass □Fail | ___秒 | |
| ASR识别 | □Pass □Fail | ___秒 | |
| LLM生成 | □Pass □Fail | ___秒 | |
| TTS合成 | □Pass □Fail | ___秒 | |
| 音频播放 | □Pass □Fail | ___秒 | |
| 状态重置 | □Pass □Fail | ___秒 | |
| 连续对话(3轮) | □Pass □Fail | ___秒 | |
| 总耗时 | - | ___秒 | |

识别准确率: ___% (说的话vs识别结果)
系统稳定性: □稳定 □偶尔异常 □频繁异常

其他问题和观察:
________________________________
________________________________
________________________________
```

---

## 验收标准

### Phase 7验收标准

**必须全部通过**：
- [ ] 状态机定义完整，所有状态和转换都存在
- [ ] 所有ROS2 topics活跃且有消息流动
- [ ] 所有关键回调函数都已实现并有日志
- [ ] 错误处理完整，有try-except和恢复机制
- [ ] 日志包含所有关键事件，级别至少INFO
- [ ] 资源使用稳定，无内存泄漏

**如果有任何一项失败，Phase 7不通过**。

### Phase 8验收标准

**完整链路测试**：
- [ ] 唤醒词触发成功率>90% (10次测试)
- [ ] ASR识别出文本（准确度>70%）
- [ ] LLM生成相关回复
- [ ] TTS合成并播放清晰粤语
- [ ] 端到端耗时<30秒
- [ ] 连续3轮对话全部成功
- [ ] 系统无崩溃或卡死

**性能指标**：
- [ ] 唤醒检测<0.3秒
- [ ] ASR识别<4秒
- [ ] LLM生成<12秒
- [ ] TTS合成<4秒

**稳定性指标**：
- [ ] 连续10轮测试无异常
- [ ] 内存增长<20MB
- [ ] 无进程崩溃

**如果有任何一项失败，Phase 8不通过**。

---

## 常见失败原因和处理

| 失败现象 | 可能原因 | 排查步骤 |
|---------|---------|---------|
| 唤醒词无响应 | 1. 进程未运行<br>2. 麦克风故障<br>3. 检测阈值过高 | 1. `ps aux \| grep wake`<br>2. `arecord`测试<br>3. 调整sensitivity |
| ASR识别为空 | 1. 录音未成功<br>2. 音频格式错误<br>3. Token无效 | 1. 检查日志"Recording started"<br>2. 验证16kHz单声道<br>3. 重新获取Token |
| LLM无响应 | 1. API Key无效<br>2. 网络问题<br>3. 配额用尽 | 1. Phase 4测试<br>2. ping测试<br>3. 检查控制台 |
| TTS无声音 | 1. 音箱未连接<br>2. Voice参数错误<br>3. 音频数据为空 | 1. `aplay`测试<br>2. 检查voice="shanshan"<br>3. 查看日志音频大小 |
| 第2轮失败 | 1. 状态未重置<br>2. 资源未释放<br>3. Token过期 | 1. 查看状态机日志<br>2. 检查WebSocket连接<br>3. 刷新Token |

---

## 总结

这是**真正严谨的Phase 7和Phase 8测试SOP**：

**Phase 7核心**：
- ✅ 验证代码逻辑、状态机、回调函数
- ✅ 验证ROS2集成、错误处理、日志
- ✅ 验证资源管理、无内存泄漏

**Phase 8核心**：
- ✅ **真实语音输入**，不允许模拟
- ✅ **真实API调用**，验证完整链路
- ✅ **真实性能测量**，记录每个环节耗时
- ✅ **多轮连续测试**，验证稳定性

**禁止的行为**：
- ❌ 用预设文本代替真实语音
- ❌ 只测试单个环节就声称端到端通过
- ❌ 不记录性能数据就说测试完成
- ❌ 第一轮成功就不测试连续对话

**只有严格按照这个SOP执行，才能真正验证系统可用！** 🎯
