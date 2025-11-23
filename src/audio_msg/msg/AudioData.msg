# 音频数据消息
std_msgs/Header header

# 音频数据参数
uint32 sample_rate     # 采样率 (Hz)
uint32 channels        # 通道数
string format         # 音频格式 (S16_LE, PCM, WAV等)
uint32 frame_size      # 帧大小 (样本数)
bool is_processed      # 是否已处理

# 音频数据 (字节流)
byte[] data