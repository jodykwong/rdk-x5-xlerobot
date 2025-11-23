# ASR API 400错误完整解决方案

## 问题识别
症状：API返回400错误，语音识别失败

## 根本原因
采样率不匹配：
- 麦克风输出: 44100 Hz或48000 Hz
- 阿里云期望: 16000 Hz
- 直接发送未重采样的音频 → 400错误

## 验证方法
```bash
# 检查麦克风采样率
arecord --dump-hw-params -D hw:0,0 | grep RATE

# 检查代码配置
grep -n "sample_rate" src/modules/asr/simple_aliyun_asr_service.py

# 检查是否启用重采样
grep -n "resample\|audio_processor" src/modules/asr/simple_aliyun_asr_service.py
```

## 解决方案
1. 启用音频重采样
2. 配置正确的采样率参数
3. 使用官方SDK获取Token

## 工作配置参考
参见：`config/snapshots/working_config_*.txt`

## 快速恢复
```bash
git checkout v1.0-asr-working -- src/modules/asr/
./start_voice_assistant.sh restart
```

## 修复代码示例
```python
# 音频处理器初始化
self.audio_processor = create_asr_audio_processor(
    sample_rate=16000,
    format="pcm",
    target_channels=1,
    target_bit_depth=16
)

# 重采样处理
if self.audio_processor and isinstance(audio_data, bytes):
    if len(audio_data) > 44 and audio_data[:4] == b'RIFF':
        import struct
        sample_rate_bytes = audio_data[24:28]
        original_sample_rate = struct.unpack('<I', sample_rate_bytes)[0]

        if original_sample_rate != 16000:
            processed_audio_data = self.audio_processor.convert_audio_format(
                audio_data, original_sample_rate
            )
```