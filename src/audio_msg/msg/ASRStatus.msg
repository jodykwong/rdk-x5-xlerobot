# ASR状态消息
std_msgs/Header header

# ASR状态信息
bool is_initialized     # 是否已初始化
bool is_processing      # 是否正在处理
string language         # 语言
uint32 sample_rate      # 采样率
string format           # 格式
uint32 total_requests   # 总请求数
uint32 successful_requests   # 成功请求数
uint32 failed_requests   # 失败请求数
float64 uptime          # 运行时间(秒)
uint32 audio_buffer_size  # 音频缓冲区大小
uint32 active_goals     # 活跃目标数

# 兼容性字段 - 用于协调器状态监控
int32 state               # 状态码: 0=idle, 1=processing, 2=error, 3=offline
float32 avg_response_time # 平均响应时间（秒）
string last_error         # 最后一次错误信息