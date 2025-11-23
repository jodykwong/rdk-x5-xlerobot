# LLM响应消息
std_msgs/Header header

# 响应内容
string text                # LLM响应文本
string session_id          # 会话ID
float32 confidence         # 置信度 (0.0-1.0)
int32 status_code          # 状态码（0=成功）
string error_message       # 错误信息（如果有）

# 元数据
string user_input          # 用户原始输入
float32 response_time      # 响应时间（秒）
string model_name          # 使用的模型名称