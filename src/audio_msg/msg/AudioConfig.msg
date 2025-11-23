# 音频配置消息
std_msgs/Header header

# 音频配置参数
uint32 sample_rate     # 采样率 (Hz)
uint32 channels        # 通道数
uint32 chunk_size      # 块大小
string format         # 音频格式
bool enable_preprocessing    # 启用预处理
bool enable_wake_word        # 启用唤醒词检测