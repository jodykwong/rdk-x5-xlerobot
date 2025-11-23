# TTS服务状态消息
std_msgs/Header header

string node_name           # 节点名称
int32 state               # 0=idle, 1=synthesizing, 2=playing, 3=error
int32 queue_length        # 当前播放队列长度
float32 avg_synthesis_time # 平均合成时间（秒）
float32 avg_playback_time  # 平均播放时间（秒）
int32 total_syntheses     # 总合成数
int32 total_playbacks     # 总播放数
string last_error         # 最后一次错误信息