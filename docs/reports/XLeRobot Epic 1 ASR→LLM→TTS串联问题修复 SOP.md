XLeRobot Epic 1 ASR→LLM→TTS串联问题修复 SOP
📋 问题诊断总结
发现的9个主要问题及处理思路
🔴 问题1：服务间通信管道完全缺失（优先级⭐⭐⭐⭐⭐）
位置：src/start_epic1_services.py
现象：主启动脚本只创建ASR服务，没有调用LLM和TTS
处理思路：创建ROS2节点，通过话题订阅自动触发后续流程
🔴 问题2：集成层状态管理混乱（优先级⭐⭐⭐⭐）
位置：src/xlerobot_final_voice_assistant.py
现象：双重状态管理，会话上下文丢失
处理思路：在主控节点统一管理会话状态
🔴 问题3：ASR输出与LLM输入格式不匹配（优先级⭐⭐⭐⭐）
现象：ASR返回ASRResult对象，LLM期望List[Dict]格式
处理思路：在LLM节点中实现适配器转换格式
🔴 问题4：LLM异步方法在同步上下文调用（优先级⭐⭐⭐⭐）
位置：src/modules/llm/qwen_client.py
现象：RuntimeError: This event loop is already running
处理思路：ROS2节点统一使用异步回调机制
🔴 问题5：TTS服务未集成到通信网络（优先级⭐⭐⭐）
现象：TTS合成成功但没有自动触发
处理思路：创建TTS节点订阅LLM响应
🔴 问题6：音频信号衰减（优先级⭐⭐⭐）
位置：src/modules/asr/websocket_asr_service_final.py
现象：音频预处理后幅度<100，识别率极低
处理思路：禁用cantonese_optimizer，使用高质量重采样
🔴 问题7：异步/同步调用混用（优先级⭐⭐⭐）
处理思路：统一使用ROS2异步回调模式
🔴 问题8：Token管理器路径硬编码（优先级⭐⭐）
位置：src/aliyun_nls_token_manager.py
现象：配置文件路径硬编码为Linux路径
处理思路：使用环境变量或相对路径
🔴 问题9：缺少端到端错误传播（优先级⭐⭐）
处理思路：在主控节点实现统一错误处理
🎯 解决方案：完成ROS2节点架构
核心策略
基于现有的10个ROS2节点（已完成），补充3个关键节点：
LLMServiceNode - 订阅ASR结果，调用LLM，发布响应
TTSServiceNode - 订阅LLM响应，合成并播放语音
VoiceAssistantCoordinator - 主控节点，协调整体流程
📝 详细实施SOP
阶段1：核心节点开发（Day 1-2）
Step 1.1: 创建LLM服务节点（6小时）
文件：src/nodes/llm_service_node.py 功能清单：
 继承rclpy.node.Node类
 订阅/asr/result话题（ASRResult消息类型）
 集成现有QwenAPIClient类
 实现格式转换适配器（ASRResult → LLM消息格式）
 处理会话上下文（使用SiQiangIntelligentDialogue）
 发布/llm/response话题（LLMResponse消息类型）
 实现错误处理和重试机制
 添加状态发布（/llm/status）
 编写单元测试
关键代码结构：
class LLMServiceNode(Node):
    def __init__(self):
        super().__init__('llm_service_node')
        
        # 订阅ASR结果
        self.asr_subscription = self.create_subscription(
            ASRResult, '/asr/result', self.asr_callback, 10)
        
        # 发布LLM响应
        self.llm_publisher = self.create_publisher(
            LLMResponse, '/llm/response', 10)
        
        # 初始化LLM客户端
        self.qwen_client = QwenAPIClient()
        self.dialogue = SiQiangIntelligentDialogue()
    
    def asr_callback(self, msg: ASRResult):
        # 1. 检查ASR成功状态
        # 2. 转换格式
        # 3. 调用LLM（异步）
        # 4. 发布响应
        # 5. 错误处理
Step 1.2: 创建TTS服务节点（6小时）
文件：src/nodes/tts_service_node.py 功能清单：
 继承rclpy.node.Node类
 订阅/llm/response话题（LLMResponse消息类型）
 集成现有SimpleTTSService或AliyunTTSFinal
 实现异步音频合成
 实现音频播放（使用asyncio.create_subprocess_exec调用aplay）
 添加播放队列管理（避免重叠）
 实现错误处理和降级
 添加状态发布（/tts/status）
 编写单元测试
关键代码结构：
class TTSServiceNode(Node):
    def __init__(self):
        super().__init__('tts_service_node')
        
        # 订阅LLM响应
        self.llm_subscription = self.create_subscription(
            LLMResponse, '/llm/response', self.llm_callback, 10)
        
        # 初始化TTS客户端
        self.tts_service = SimpleTTSService()
        
        # 播放队列
        self.play_queue = asyncio.Queue()
    
    async def llm_callback(self, msg: LLMResponse):
        # 1. 提取文本
        # 2. 调用TTS合成
        # 3. 异步播放音频
        # 4. 错误处理
Step 1.3: 创建主控协调节点（8小时）
文件：src/nodes/voice_assistant_coordinator.py 功能清单：
 继承rclpy.node.Node类
 监控所有节点状态（订阅各节点的status话题）
 实现会话生命周期管理
 实现错误恢复和重试策略
 提供服务接口（启动/停止对话）
 记录完整对话日志
 实现性能监控（响应时间统计）
 发布系统状态（/system/status）
 编写单元测试
关键代码结构：
class VoiceAssistantCoordinator(Node):
    def __init__(self):
        super().__init__('voice_assistant_coordinator')
        
        # 订阅各模块状态
        self.create_subscription(ASRStatus, '/asr/status', ...)
        self.create_subscription(LLMStatus, '/llm/status', ...)
        self.create_subscription(TTSStatus, '/tts/status', ...)
        
        # 会话管理
        self.session_manager = SessionManager()
        
        # 创建服务
        self.create_service(StartDialogue, 'start_dialogue', ...)
阶段2：消息类型定义（Day 2，3小时）
Step 2.1: 定义LLM响应消息
文件：src/xlerobot_interfaces/msg/LLMResponse.msg
std_msgs/Header header
string text                # LLM响应文本
string session_id          # 会话ID
float32 confidence         # 置信度
int32 status_code          # 状态码（0=成功）
string error_message       # 错误信息（如果有）
Step 2.2: 定义LLM状态消息
文件：src/xlerobot_interfaces/msg/LLMStatus.msg
std_msgs/Header header
string node_name
int32 state               # 0=idle, 1=processing, 2=error
float32 avg_response_time
int32 total_requests
int32 failed_requests
Step 2.3: 定义TTS状态消息
文件：src/xlerobot_interfaces/msg/TTSStatus.msg
std_msgs/Header header
string node_name
int32 state               # 0=idle, 1=synthesizing, 2=playing, 3=error
int32 queue_length
float32 avg_synthesis_time
Step 2.4: 编译消息
cd /home/sunrise/xlerobot
colcon build --packages-select xlerobot_interfaces
source install/setup.bash
阶段3：Launch文件创建（Day 2，3小时）
Step 3.1: 创建主Launch文件
文件：launch/voice_assistant.launch.py 功能清单：
 启动所有核心节点
 配置命名空间
 设置环境变量
 配置日志级别
 设置节点启动顺序
 添加条件启动（可选节点）
代码：
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 环境变量
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', 
                             '[{severity}] [{name}]: {message}'),
        
        # 参数
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('enable_vision', default_value='false'),
        
        # 音频输入节点
        Node(
            package='xlerobot',
            executable='audio_input_node',
            name='audio_input_node',
            namespace='xlerobot',
            parameters=[{'log_level': LaunchConfiguration('log_level')}],
            output='screen'
        ),
        
        # ASR服务节点
        Node(
            package='xlerobot',
            executable='asr_service_node',
            name='asr_service_node',
            namespace='xlerobot',
            output='screen'
        ),
        
        # LLM服务节点（新建）
        Node(
            package='xlerobot',
            executable='llm_service_node',
            name='llm_service_node',
            namespace='xlerobot',
            output='screen'
        ),
        
        # TTS服务节点（新建）
        Node(
            package='xlerobot',
            executable='tts_service_node',
            name='tts_service_node',
            namespace='xlerobot',
            output='screen'
        ),
        
        # 主控协调节点（新建）
        Node(
            package='xlerobot',
            executable='voice_assistant_coordinator',
            name='voice_assistant_coordinator',
            namespace='xlerobot',
            output='screen'
        ),
    ])
阶段4：启动脚本修改（Day 2，1小时）
Step 4.1: 修改start_voice_assistant.sh
位置：第968行 修改前：
nohup python3.10 src/start_epic1_services.py > "$LOG_FILE" 2>&1 &
修改后：
# 启动ROS2 Launch系统
source /opt/ros/humble/setup.bash
source install/setup.bash
nohup ros2 launch xlerobot voice_assistant.launch.py > "$LOG_FILE" 2>&1 &
Step 4.2: 添加ROS2环境检查
在环境检查函数中添加：
# 检查ROS2节点是否编译
if [ ! -f "install/setup.bash" ]; then
    log_error "❌ ROS2包未编译，请运行: colcon build"
    ((errors++))
fi
阶段5：集成测试（Day 3，8小时）
Step 5.1: 单节点测试
测试LLM节点：
# 终端1：启动LLM节点
ros2 run xlerobot llm_service_node

# 终端2：模拟ASR结果发布
ros2 topic pub /asr/result xlerobot_interfaces/msg/ASRResult \
  "{success: true, text: '你好', confidence: 0.95}"

# 终端3：监听LLM响应
ros2 topic echo /llm/response
测试TTS节点：
# 终端1：启动TTS节点
ros2 run xlerobot tts_service_node

# 终端2：模拟LLM响应发布
ros2 topic pub /llm/response xlerobot_interfaces/msg/LLMResponse \
  "{text: '你好，我系傻强'}"

# 验证音频播放
Step 5.2: 端到端测试
完整流程测试：
# 启动完整系统
./start_voice_assistant.sh

# 检查节点状态
ros2 node list
# 应该看到：
# /xlerobot/audio_input_node
# /xlerobot/asr_service_node
# /xlerobot/llm_service_node
# /xlerobot/tts_service_node
# /xlerobot/voice_assistant_coordinator

# 检查话题
ros2 topic list
# 应该看到完整的话题链路

# 监控话题通信
ros2 topic hz /asr/result
ros2 topic hz /llm/response

# 测试唤醒词 → 对话 → 播放
# 说："傻强"
# 等待提示音
# 说："今日天气点样？"
# 验证回复播放
Step 5.3: 性能测试
测试指标：
 ASR识别延迟 < 2秒
 LLM响应延迟 < 3秒
 TTS合成延迟 < 1秒
 端到端延迟 < 6秒
 ASR识别准确率 > 80%（粤语）
 系统稳定运行 > 1小时
测试命令：
# 记录性能数据
ros2 topic echo /system/status | tee performance.log

# 分析响应时间
cat performance.log | grep "avg_response_time"
阶段6：问题修复（Day 3，4小时）
Step 6.1: 修复音频信号衰减
位置：src/modules/asr/websocket_asr_service_final.py 修改：
def __init__(self, enable_optimization: bool = False):  # 默认关闭优化
    ...
    if enable_optimization:  # 仅在明确启用时使用
        self.cantonese_optimizer = CantoneseAudioOptimizer()
Step 6.2: 修复Token管理器路径
位置：src/aliyun_nls_token_manager.py 修改：
def __init__(self, config_path=None):
    if config_path is None:
        # 智能查找配置文件
        candidates = [
            os.getenv("ALIYUN_CONFIG_PATH"),
            os.path.join(os.getcwd(), "config/aliyun_nls_config.yaml"),
            "/home/sunrise/xlerobot/config/aliyun_nls_config.yaml"
        ]
        for path in candidates:
            if path and os.path.exists(path):
                config_path = path
                break
阶段7：文档和清理（Day 4，4小时）
Step 7.1: 更新文档
文件：docs/ros2-nodes-architecture.md 内容包括：
 ROS2节点架构图
 话题通信关系图
 消息类型定义
 Launch文件使用说明
 故障排查指南
Step 7.2: 代码清理
 删除或标记废弃的纯Python启动方式
 统一代码风格（black格式化）
 添加类型提示
 完善注释和文档字符串
Step 7.3: 更新README
在README.md中添加：
## 🚀 启动方式（ROS2）

### 方式1：使用启动脚本（推荐）
```bash
./start_voice_assistant.sh
方式2：直接使用ROS2 Launch
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch xlerobot voice_assistant.launch.py
检查系统状态
# 查看运行的节点
ros2 node list

# 查看话题
ros2 topic list

# 监控ASR结果
ros2 topic echo /xlerobot/asr/result
✅ 验收标准
功能验收
 唤醒词"傻强"成功率 > 90%
 ASR识别粤语成功率 > 80%
 LLM响应合理且相关
 TTS语音清晰、自然
 端到端对话流程完整
性能验收
 端到端响应时间 < 6秒
 ASR识别延迟 < 2秒
 LLM响应延迟 < 3秒
 TTS合成延迟 < 1秒
 系统内存占用 < 2GB
 系统稳定运行 > 24小时
架构验收
 所有核心功能通过ROS2节点实现
 使用ROS2 Launch统一启动
 话题通信关系清晰
 错误处理完善
 日志记录完整
📊 工作量估算
阶段	任务	时间	负责人
阶段1	LLM节点开发	6小时	开发
阶段1	TTS节点开发	6小时	开发
阶段1	主控节点开发	8小时	开发
阶段2	消息定义和编译	3小时	开发
阶段3	Launch文件	3小时	开发
阶段4	启动脚本修改	1小时	开发
阶段5	集成测试	8小时	测试
阶段6	问题修复	4小时	开发
阶段7	文档和清理	4小时	开发
总计		43小时	约3个工作日
🎯 立即行动清单
今天（第1天）
 创建LLM服务节点（6小时）
 创建TTS服务节点（6小时）
明天（第2天）
 创建主控协调节点（8小时）
 定义消息类型（3小时）
 创建Launch文件（3小时）
后天（第3天）
 集成测试（8小时）
 问题修复（4小时）
 文档更新（4小时）
🔧 关键技术决策
统一使用ROS2节点架构 - 不再维护纯Python启动方式
保留现有服务类 - LLM节点封装QwenAPIClient，不重写算法
异步通信模型 - 所有节点回调使用异步处理
统一错误处理 - 主控节点负责错误恢复
配置外部化 - 使用环境变量和ROS2参数