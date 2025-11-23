# 音频状态消息
std_msgs/Header header

# 音频状态信息
bool is_recording     # 是否正在录制
string device_id      # 设备ID
uint32 sample_rate    # 采样率
uint32 channels       # 通道数
uint32 total_frames   # 总帧数
uint32 processed_frames   # 已处理帧数
uint32 wake_word_detections  # 唤醒词检测次数
uint32 errors         # 错误次数
float64 uptime       # 运行时间(秒)