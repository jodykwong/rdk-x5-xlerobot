# 唤醒词识别准确率优化计划
## Story 1.1 AC-2 高优先级行动项

**目标**: 将唤醒词识别准确率从50%提升至95%+
**唤醒词**: "傻强" (粤语)
**当前状态**: 模拟测试50%通过率

---

## 📊 问题分析

### 当前准确率低的原因
1. **训练数据不足**: 缺乏真实粤语"傻强"发音样本
2. **模型泛化能力差**: CNN模型在真实环境中表现不佳
3. **环境噪声影响**: 未充分考虑实际使用环境的噪声
4. **声调敏感性**: 粤语声调变化对识别准确率的影响

### 技术瓶颈
- 现有模型主要基于合成数据训练
- 缺乏多样化的发音者数据
- 环境适应性不足

---

## 🎯 优化策略

### 阶段1: 数据收集和增强 (1-2天)
**目标**: 建立高质量训练数据集

#### 1.1 真实语音数据收集
- **来源**: 团队成员录制"傻强"唤醒词
- **数量**: 每人录制50-100个样本
- **环境**: 多种环境（安静、背景噪声、不同距离）
- **设备**: 使用项目支持的USB音频设备

#### 1.2 数据增强
- **噪声注入**: 添加真实环境噪声
- **音调变化**: 模拟不同声调和语速
- **距离变化**: 模拟不同距离的录制
- **回声处理**: 添加适当的回声效果

### 阶段2: 模型优化和重训练 (2-3天)
**目标**: 提升模型在真实数据上的表现

#### 2.1 模型架构优化
- **输入特征**: 优化MFCC特征提取参数
- **网络结构**: 调整CNN层数和滤波器数量
- **正则化**: 添加Dropout和Batch Normalization
- **损失函数**: 使用Focal Loss处理类别不平衡

#### 2.2 迁移学习
- **预训练模型**: 使用大规模语音识别预训练模型
- **微调策略**: 在"傻强"数据上进行领域自适应
- **多任务学习**: 同时优化唤醒词检测和说话人识别

#### 2.3 集成学习
- **模型融合**: 结合多个模型的预测结果
- **投票机制**: 使用软投票提高准确率
- **置信度校准**: 优化预测置信度计算

### 阶段3: 实时优化 (1-2天)
**目标**: 优化实时检测性能

#### 3.1 滑动窗口优化
- **窗口大小**: 调整检测窗口大小
- **步长优化**: 优化滑动步长
- **重叠处理**: 优化窗口重叠策略

#### 3.2 后处理优化
- **阈值调整**: 动态调整检测阈值
- **平滑滤波**: 减少误检测
- **置信度累积**: 多帧置信度累积

---

## 🛠️ 实施计划

### 技术实施步骤

#### 步骤1: 数据收集工具开发
```python
# 数据收集脚本示例
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav

def record_wake_word_samples(filename_prefix, num_samples=100):
    """录制唤醒词样本"""
    for i in range(num_samples):
        print(f"请录制第 {i+1}/{num_samples} 个样本：说'傻强'")
        # 录制逻辑
        audio_data = record_audio(duration=2.0)
        wav.write(f"{filename_prefix}_{i:03d}.wav", 16000, audio_data)
        print("录制完成，按Enter继续...")
        input()
```

#### 步骤2: 数据预处理管道
```python
# 数据增强管道
def augment_audio_data(audio_data, sample_rate):
    """音频数据增强"""
    augmented_samples = []

    # 原始数据
    augmented_samples.append(audio_data)

    # 添加噪声
    noise = np.random.normal(0, 0.01, len(audio_data))
    augmented_samples.append(audio_data + noise)

    # 时间偏移
    shift = np.random.randint(-1000, 1000)
    augmented_samples.append(np.roll(audio_data, shift))

    # 音调变化
    pitched = change_pitch(audio_data, sample_rate, random_factor=True)
    augmented_samples.append(pitched)

    return augmented_samples
```

#### 步骤3: 模型训练优化
```python
# 优化的训练循环
class ImprovedWakeWordDetector:
    def __init__(self):
        self.model = self.build_improved_model()
        self.threshold = 0.5  # 初始阈值

    def build_improved_model(self):
        """构建改进的CNN模型"""
        model = Sequential([
            # 输入层
            Conv1D(32, 3, activation='relu', input_shape=(99, 13)),
            BatchNormalization(),
            MaxPooling1D(2),
            Dropout(0.2),

            # 中间层
            Conv1D(64, 3, activation='relu'),
            BatchNormalization(),
            MaxPooling1D(2),
            Dropout(0.3),

            # 深层特征
            Conv1D(128, 3, activation='relu'),
            BatchNormalization(),
            GlobalAveragePooling1D(),
            Dropout(0.4),

            # 输出层
            Dense(64, activation='relu'),
            Dropout(0.5),
            Dense(1, activation='sigmoid')
        ])

        model.compile(
            optimizer=Adam(learning_rate=0.001),
            loss='binary_crossentropy',
            metrics=['accuracy', 'precision', 'recall']
        )

        return model
```

---

## 📈 性能指标和验证

### 准确率提升目标
- **短期目标**: 50% → 75% (1周内)
- **中期目标**: 75% → 90% (2周内)
- **最终目标**: 90% → 95%+ (3周内)

### 验证方法
1. **交叉验证**: 5折交叉验证确保模型稳定性
2. **真实环境测试**: 在实际使用环境中测试
3. **多说话人测试**: 不同说话者的泛化能力
4. **噪声鲁棒性测试**: 不同噪声环境下的表现

### 成功标准
- **准确率**: ≥95% (在测试集上)
- **误检率**: ≤5% (非唤醒词被误识别)
- **响应时间**: ≤1秒 (保持现有性能)
- **内存占用**: ≤100MB (满足硬件约束)

---

## 🔄 持续改进机制

### 数据持续收集
- 建立用户反馈收集机制
- 定期更新训练数据集
- 在线学习算法实现

### 模型版本管理
- A/B测试框架
- 模型性能监控
- 自动回滚机制

### 性能监控
- 实时准确率监控
- 误检率统计
- 用户满意度调查

---

## 🎯 即时行动项

### 今天可以完成的任务
1. ✅ **librosa依赖问题已解决**
2. **数据收集脚本开发** - 创建录制工具
3. **团队样本收集** - 收集初始"傻强"样本
4. **数据增强管道** - 实现数据增强算法

### 本周目标
1. 收集至少200个"傻强"样本
2. 完成数据增强处理
3. 重训练并优化模型
4. 验证准确率提升至75%+

---

**责任人**: Developer Agent
**审核人**: Scrum Master
**完成时间**: 预计1-2周看到显著改善

这个优化计划将直接解决AC-2准确率问题，并为后续的Story 1.2 (粤语语音识别优化) 奠定坚实基础。