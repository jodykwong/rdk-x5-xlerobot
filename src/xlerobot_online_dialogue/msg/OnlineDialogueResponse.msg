# OnlineDialogueResponse消息 - 在线对话响应
# Story 1.7: 多模态在线对话API集成
# 严格遵循Epic 1纯在线架构

# Header包含时间戳信息
std_msgs/Header header

# 会话ID
string session_id

# API响应文本
string text_response

# 响应状态
uint8 RESPONSE_STATUS_SUCCESS=1
uint8 RESPONSE_STATUS_ERROR=2
uint8 RESPONSE_STATUS_TIMEOUT=3
uint8 response_status

# 错误信息（如果有）
string error_message

# API调用延迟（毫秒）
uint32 response_latency_ms

# API元数据
string[] api_metadata

# 是否成功处理
bool success