# LLM服务状态消息
std_msgs/Header header

string node_name           # 节点名称
int32 state               # 0=idle, 1=processing, 2=error
float32 avg_response_time # 平均响应时间（秒）
int32 total_requests      # 总请求数
int32 failed_requests     # 失败请求数
float32 cpu_usage         # CPU使用率
float32 memory_usage      # 内存使用率
string last_error         # 最后一次错误信息