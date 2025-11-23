#!/bin/bash

# 测试摄像头检查功能修复效果
echo "🧪 测试摄像头检查功能修复效果..."

source ./xlerobot_env.sh

# 测试lsmod超时机制
echo "📋 测试1: lsmod超时机制"
driver_modules=("uvcvideo" "videobuf2_vmalloc" "videobuf2_memops" "videobuf2_v4l2" "videobuf2_common")

for module in "${driver_modules[@]}"; do
    echo -n "   检查 $module: "
    if timeout 1 bash -c "lsmod 2>/dev/null | grep -q '^$module'" 2>/dev/null; then
        echo "✅ 已加载"
    else
        echo "❌ 未加载"
    fi
done

echo ""
echo "📋 测试2: 设备文件检查"
camera_devices=0
for i in {0..9}; do
    device_path="/dev/video$i"
    if [ -e "$device_path" ]; then
        echo "   ✅ 找到设备: $device_path"
        ((camera_devices++))
    fi
done

if [ $camera_devices -eq 0 ]; then
    echo "   ℹ️ 未找到摄像头设备"
fi

echo ""
echo "📋 测试3: v4l2-ctl超时机制"
if command -v v4l2-ctl &> /dev/null; then
    echo "   v4l2-ctl工具可用"
    if compgen -G "/dev/video*" > /dev/null 2>&1; then
        echo "   有视频设备，测试v4l2-ctl..."
        if timeout 2 bash -c 'v4l2-ctl --list-devices 2>/dev/null' 2>/dev/null >/dev/null; then
            echo "   ✅ v4l2-ctl工作正常"
        else
            echo "   ⚠️ v4l2-ctl超时或失败"
        fi
    else
        echo "   无视频设备，跳过v4l2-ctl测试"
    fi
else
    echo "   ⚠️ v4l2-ctl工具不可用"
fi

echo ""
echo "✅ 摄像头检查功能测试完成"