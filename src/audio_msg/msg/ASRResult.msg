# ASR识别结果消息
std_msgs/Header header

# 识别结果
string text           # 识别文本
float32 confidence     # 置信度 (0.0-1.0)
uint32 begin_time      # 开始时间 (毫秒)
uint32 end_time        # 结束时间 (毫秒)
uint32 status_code     # 状态码
string message        # 消息