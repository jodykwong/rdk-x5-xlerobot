# OnlineDialogueInput消息 - 在线对话输入
# Story 1.7: 多模态在线对话API集成
# 严格遵循Epic 1纯在线架构

# Header包含时间戳信息
std_msgs/Header header

# 会话ID
string session_id

# 用户输入文本
string user_input

# 音频数据 (可选，Base64编码)
string audio_base64

# 图像数据 (可选，Base64编码)
string image_base64

# 输入类型标识
uint8 INPUT_TYPE_TEXT_ONLY=1
uint8 INPUT_TYPE_AUDIO_ONLY=2
uint8 INPUT_TYPE_IMAGE_ONLY=3
uint8 INPUT_TYPE_MULTIMODAL=4
uint8 input_type

# 请求元数据
string[] metadata