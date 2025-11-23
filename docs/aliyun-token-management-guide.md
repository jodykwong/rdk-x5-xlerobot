# 阿里云NLS Token管理指南

## 📋 概述

本文档详细说明XleRobot项目中阿里云NLS Token的自动管理机制，包括Token获取、缓存、自动刷新和过期处理。

## 🔄 Token过期处理机制

### 1. **自动刷新机制** ✅ **推荐**

Token管理器采用**主动自动刷新**策略，无需手动干预：

```python
# 自动刷新逻辑
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   每分钟检查    │───▶│  Token即将过期? │───▶│   自动刷新Token │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                              │
                              ▼
                       ┌──────────────┐
                       │ 继续使用Token │
                       └──────────────┘
```

**关键特性：**
- **缓冲时间**: 提前5分钟自动刷新（可配置）
- **后台线程**: 独立线程负责自动刷新
- **无缝切换**: 不会影响正在进行的语音服务

### 2. **缓存机制**

Token信息会自动缓存到本地文件：

```yaml
cache:
  enabled: true
  cache_file: "/tmp/aliyun_nls_token.cache"
  buffer_seconds: 300  # 提前5分钟刷新
```

**缓存优势：**
- **持久化存储**: 程序重启后仍可使用缓存的Token
- **减少API调用**: 避免频繁请求阿里云API
- **离线启动**: 即使网络短暂中断也能使用缓存的Token

### 3. **过期检查策略**

```python
def is_token_valid():
    """智能Token有效性检查"""
    current_time = int(time.time())
    # 考虑缓冲时间，提前刷新
    return current_time < (expire_time - buffer_seconds)
```

## 🚀 使用方法

### 1. **基础使用**

```python
from aliyun_nls_token_manager import get_valid_token

# 获取有效Token（自动处理刷新）
token = get_valid_token()
if token:
    print(f"Token: {token}")
    # 使用Token调用阿里云NLS服务
else:
    print("Token获取失败")
```

### 2. **高级使用**

```python
from aliyun_nls_token_manager import AliyunNLSTokenManager

# 创建Token管理器实例
manager = AliyunNLSTokenManager()

# 获取Token信息
info = manager.get_token_info()
print(f"剩余时间: {info['remaining_hours']} 小时")
print(f"Token有效: {info['is_valid']}")

# 强制刷新Token
manager.refresh_token(force=True)

# 检查Token状态
if manager.is_token_valid():
    token = manager.get_token()
```

### 3. **ROS2集成示例**

```python
import rclpy
from rclpy.node import Node
from aliyun_nls_token_manager import get_valid_token

class AliyunNLSNode(Node):
    def __init__(self):
        super().__init__('aliyun_nls_node')
        self.timer = self.create_timer(0.1, self.process_audio)

    def process_audio(self):
        # 每次处理音频前获取有效Token
        token = get_valid_token()
        if token:
            # 使用Token进行语音识别/合成
            self.call_nls_service(token)
        else:
            self.get_logger().error("Token不可用")
```

## ⚙️ 配置参数

### Token缓存配置

```yaml
authentication:
  token:
    # Token缓存配置
    cache:
      enabled: true                      # 启用缓存
      cache_file: "/tmp/aliyun_nls_token.cache"  # 缓存文件路径
      buffer_seconds: 300               # 提前刷新时间（秒）
      expire_seconds: 3600              # Token过期时间（秒）
```

### 环境变量配置

```bash
# 可选：通过环境变量覆盖配置
export ALIYUN_NLS_ACCESS_KEY_ID="your_access_key_id"
export ALIYUN_NLS_ACCESS_KEY_SECRET="your_access_key_secret"
export ALIYUN_NLS_REGION="cn-shanghai"
```

## 🔧 运维管理

### 1. **监控Token状态**

```bash
# 查看Token管理器状态
python3 /home/sunrise/xlerobot/src/aliyun_nls_token_manager.py

# 输出示例：
# 📋 Token信息:
#   Token: 62a78450593e4fa3bd10...4fa3bd109cbb7045f081
#   剩余时间: 35 小时
#   是否有效: True
#   缓存文件: /tmp/aliyun_nls_token.cache
```

### 2. **手动刷新Token**

```python
from aliyun_nls_token_manager import get_token_manager

manager = get_token_manager()
success = manager.refresh_token(force=True)
print(f"刷新结果: {'成功' if success else '失败'}")
```

### 3. **清理缓存**

```bash
# 删除Token缓存文件
rm /tmp/aliyun_nls_token.cache

# 程序会在下次启动时自动重新获取Token
```

## 🛡️ 错误处理

### 常见问题和解决方案

#### 1. **Token获取失败**

**错误信息**: `❌ Token请求失败: InvalidAccessKeyId.NotFound`

**解决方案**:
```bash
# 1. 检查配置文件
cat /home/sunrise/xlerobot/config/aliyun_nls_config.yaml

# 2. 验证AccessKey配置
python3 -c "
import yaml
with open('/home/sunrise/xlerobot/config/aliyun_nls_config.yaml') as f:
    config = yaml.safe_load(f)
    print('AccessKeyId:', config['authentication']['token']['access_key_id'])
    print('AccessKeySecret:', '已配置' if config['authentication']['token']['access_key_secret'] else '未配置')
"

# 3. 检查阿里云服务状态
curl -s https://nls-meta.cn-shanghai.aliyuncs.com
```

#### 2. **网络连接问题**

**错误信息**: `❌ Token请求失败: ConnectTimeout`

**解决方案**:
```bash
# 1. 检查网络连接
ping nls-meta.cn-shanghai.aliyuncs.com

# 2. 检查防火墙设置
sudo ufw status

# 3. 测试HTTPS连接
curl -I https://nls-meta.cn-shanghai.aliyuncs.com
```

#### 3. **权限问题**

**错误信息**: `❌ 缓存保存失败: Permission denied`

**解决方案**:
```bash
# 1. 检查缓存目录权限
ls -la /tmp/

# 2. 创建专用缓存目录
sudo mkdir -p /var/cache/xlerobot
sudo chown $USER:$USER /var/cache/xlerobot
chmod 755 /var/cache/xlerobot

# 3. 更新配置文件中的缓存路径
# cache_file: "/var/cache/xlerobot/aliyun_nls_token.cache"
```

## 📊 性能优化

### 1. **Token刷新频率优化**

```python
# 根据使用频率调整检查间隔
class HighFrequencyTokenManager(AliyunNLSTokenManager):
    def _auto_refresh_worker(self):
        """高频使用场景：每30秒检查一次"""
        while True:
            time.sleep(30)  # 更频繁的检查
            # ... 刷新逻辑
```

### 2. **缓存策略优化**

```yaml
# 生产环境配置
authentication:
  token:
    cache:
      buffer_seconds: 1800  # 提前30分钟刷新（更保守）
      expire_seconds: 7200  # 2小时过期时间
```

## 🔍 调试模式

启用详细日志：

```python
import logging
logging.getLogger('AliyunNLSTokenManager').setLevel(logging.DEBUG)

# 运行Token管理器
python3 /home/sunrise/xlerobot/src/aliyun_nls_token_manager.py
```

## 📝 最佳实践

### 1. **程序启动时检查Token**

```python
def startup_token_check():
    """启动时Token检查"""
    manager = get_token_manager()
    if not manager.is_token_valid():
        if not manager.refresh_token():
            raise RuntimeError("Token初始化失败，请检查配置")
    print("✅ Token检查通过")
```

### 2. **优雅关闭处理**

```python
import signal
import sys

def signal_handler(signum, frame):
    """优雅关闭处理"""
    manager = get_token_manager()
    manager.cleanup()  # 清理缓存
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
```

### 3. **多线程安全**

Token管理器内置线程锁，支持多线程环境：

```python
import threading
from aliyun_nls_token_manager import get_valid_token

def worker_thread():
    """工作线程示例"""
    token = get_valid_token()  # 线程安全
    # 使用token...

# 创建多个工作线程
threads = []
for i in range(5):
    t = threading.Thread(target=worker_thread)
    threads.append(t)
    t.start()
```

## 🎯 总结

XleRobot的Token管理机制提供：

✅ **完全自动化**: 无需手动干预，自动处理Token刷新
✅ **高可用性**: 缓存机制确保服务连续性
✅ **线程安全**: 支持多线程并发访问
✅ **错误恢复**: 网络异常时自动重试
✅ **生产就绪**: 完整的监控和日志系统

**您无需担心Token过期问题**，系统会自动处理所有Token相关的管理工作！

---

**文档版本**: 1.0
**最后更新**: 2025-11-08
**适用项目**: XleRobot Story 1.1 - 语音唤醒和基础识别