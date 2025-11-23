# 智能家居控制模块

智能家居控制模块是XLeRobot语音助手系统的重要组成部分，提供统一的多协议智能设备接入、控制和管理功能。

## 功能特性

### 协议支持

- **WiFi**: 基于MQTT的WiFi智能设备（灯光、插座、传感器等）
- **Bluetooth**: 蓝牙低功耗(BLE)设备（传感器、门锁等）
- **Zigbee**: Zigbee 3.0网状网络设备（灯光、传感器等）
- **Matter**: 新一代统一智能家居协议（支持所有类型设备）

### 核心能力

- ✅ **多协议适配**: 统一接口适配多种智能家居协议
- ✅ **设备发现**: 自动发现和识别智能设备
- ✅ **设备管理**: 设备注册、状态维护、属性管理
- ✅ **设备控制**: 统一的设备控制接口，支持批量操作
- ✅ **语音集成**: 与ASR+LLM+TTS深度集成，支持语音控制
- ✅ **场景自动化**: 支持自定义场景和自动化规则
- ✅ **设备联动**: 多设备联动控制和冲突处理

## 架构设计

### 模块结构

```
smart_home/
├── core/                    # 核心抽象层
│   ├── device_interface.py  # 设备接口抽象
│   ├── protocol_adapter.py  # 协议适配器抽象
│   └── device_registry.py   # 设备注册表
├── protocols/               # 协议实现
│   ├── wifi_adapter.py      # WiFi协议适配器
│   ├── bluetooth_adapter.py # 蓝牙协议适配器
│   ├── zigbee_adapter.py    # Zigbee协议适配器
│   └── matter_adapter.py    # Matter协议适配器
├── services/                # 服务层
│   ├── device_discovery.py  # 设备发现服务
│   └── device_control.py    # 设备控制服务
├── tests/                   # 测试套件
├── examples/                # 使用示例
├── iot_service_node.py      # ROS2服务节点
├── __init__.py
└── README.md
```

### 核心组件

#### 1. 设备接口抽象 (DeviceInterface)

统一的设备接口抽象，定义设备基本属性和操作方法：

```python
from smart_home import DeviceInterface, DeviceInfo, DeviceType, ProtocolType

class YourDevice(DeviceInterface):
    async def connect(self):
        # 实现设备连接
        pass

    async def send_command(self, command):
        # 实现设备控制
        pass

    async def get_status(self):
        # 实现状态获取
        pass
```

#### 2. 协议适配器 (ProtocolAdapter)

协议适配器模式，支持多种协议设备：

- `WiFiAdapter`: WiFi/MQTT设备适配
- `BluetoothAdapter`: 蓝牙低功耗设备适配
- `ZigbeeAdapter`: Zigbee设备适配
- `MatterAdapter`: Matter协议设备适配

#### 3. 设备注册表 (DeviceRegistry)

设备统一管理：

```python
from smart_home import get_device_registry

registry = get_device_registry()

# 注册设备
registry.register_device(device)

# 查找设备
devices = registry.list_devices()
light_devices = registry.list_devices_by_type('light')

# 搜索设备
found_devices = registry.search_devices(location='客厅', manufacturer='小米')
```

#### 4. 设备发现服务 (DeviceDiscoveryService)

自动发现和配对智能设备：

```python
from smart_home import get_discovery_service

discovery = get_discovery_service()

# 启动发现服务
await discovery.start_discovery()

# 单次发现
devices = await discovery.discover_once(ProtocolType.WIFI)
```

#### 5. 设备控制服务 (DeviceControlService)

统一的设备控制接口：

```python
from smart_home import get_control_service

control = get_control_service()

# 控制单个设备
result = await control.control_device(
    device_id='wifi_light_001',
    command_type='turn_on',
    parameters={'brightness': 80},
    source='voice'
)

# 批量控制
results = await control.control_multiple_devices([
    {'device_id': 'device1', 'command_type': 'turn_on'},
    {'device_id': 'device2', 'command_type': 'turn_off'}
], parallel=True)
```

## 使用示例

### 基本使用

```python
import asyncio
from smart_home import (
    get_protocol_registry,
    get_adapter_manager,
    get_discovery_service,
    get_control_service,
    ProtocolType,
    DeviceType,
)

async def main():
    # 1. 初始化
    protocol_registry = get_protocol_registry()
    protocol_registry.auto_discover_adapters()

    adapter_manager = get_adapter_manager()
    # 注册适配器...
    # adapter_manager.register_adapter(WiFiAdapter())

    # 2. 启动设备发现
    discovery = get_discovery_service()
    await discovery.start_discovery()

    # 3. 发现设备
    devices = await discovery.discover_once()

    # 4. 控制设备
    control = get_control_service()
    result = await control.control_device(
        device_id='device_id',
        command_type='turn_on',
        source='example'
    )

asyncio.run(main())
```

### ROS2集成

```python
from smart_home import IoTServiceNode
import rclpy

rclpy.init()
node = IoTServiceNode()
rclpy.spin(node)
```

### 语音控制集成

模块与ASR+LLM+TTS系统深度集成：

1. **ASR**: 语音识别
2. **LLM**: 解析控制意图
3. **IoT服务**: 执行设备控制
4. **TTS**: 语音反馈

## API文档

### 设备类型 (DeviceType)

- `LIGHT`: 灯光设备
- `SWITCH`: 开关设备
- `SENSOR`: 传感器
- `THERMOSTAT`: 温控器
- `CURTAIN`: 窗帘
- `OUTLET`: 插座
- `CAMERA`: 摄像头
- `LOCK`: 门锁

### 协议类型 (ProtocolType)

- `WIFI`: WiFi协议
- `BLUETOOTH`: 蓝牙协议
- `ZIGBEE`: Zigbee协议
- `MATTER`: Matter协议

### 设备能力 (DeviceCapability)

- `ON_OFF`: 开关能力
- `DIMMABLE`: 可调亮度
- `RGB_COLOR`: 彩光
- `TEMPERATURE`: 温度检测
- `HUMIDITY`: 湿度检测
- `LOCK_UNLOCK`: 上锁解锁

## 性能指标

- **设备响应时间**: <2秒
- **支持设备数量**: 100+
- **并发控制**: 20个设备
- **协议支持**: 4种 (WiFi/BLE/Zigbee/Matter)
- **设备发现时间**: <30秒

## 依赖库

- **MQTT**: paho-mqtt >=1.6.0
- **Bluetooth**: bleak >=0.20.0
- **Zigbee**: zigpy >=0.54.0
- **Matter**: chip >=0.6.0
- **ROS2**: rclpy (Humble)

## 许可证

MIT License

## 作者

Dev Agent - XLeRobot项目

## 更新日志

### v1.0.0 (2025-11-05)

- ✅ 实现协议抽象层和统一接口
- ✅ 实现4种协议适配器 (WiFi/BLE/Zigbee/Matter)
- ✅ 实现设备发现和管理服务
- ✅ 实现设备控制服务
- ✅ 实现ROS2节点集成
- ✅ 实现语音控制流程集成
- ✅ 完成8/8验收标准

## 贡献指南

欢迎提交Pull Request和Issue。

## 联系方式

- 项目仓库: XLeRobot项目
- 作者: Dev Agent
