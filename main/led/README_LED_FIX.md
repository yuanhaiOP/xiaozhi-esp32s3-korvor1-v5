# LED效果冲突问题修复

## 问题描述
在idle模式下设置呼吸灯效果时，该效果会影响speaking模式和listening模式时LED的显示，导致不同模式之间的LED效果相互干扰。

## 根本原因
1. **任务管理不当**: 不同效果的任务没有正确的优先级管理
2. **状态管理缺失**: 没有统一的状态管理机制来协调不同效果
3. **资源竞争**: 多个任务同时操作同一个LED strip，导致显示冲突
4. **代码错误**: `BreathingEffectTask`函数实现有语法错误

## 修复方案

### 1. 添加状态管理机制
- 引入`LedEffectState`枚举，定义不同的LED效果状态
- 实现状态优先级管理：语音效果 > 呼吸灯效果 > 空闲状态
- 添加状态变化检测机制

### 2. 改进任务管理
- 实现`SetEffectState()`函数统一管理状态切换
- 实现`StopAllEffects()`函数确保效果切换时正确停止所有任务
- 添加`CanStartEffect()`函数检查效果启动权限

### 3. 修复代码错误
- 修复`BreathingEffectTask`函数的语法错误
- 添加状态检查机制到所有效果任务中
- 改进错误处理和RMT通道管理

### 4. 增强互斥性
- 使用`std::mutex`保护共享资源
- 添加`std::atomic<bool>`标志检测状态变化
- 实现优雅的任务停止机制

## 使用方法

### 基本使用流程
```cpp
// 1. 在idle模式下启动呼吸灯效果
led->StartBreathingEffect();

// 2. 进入speaking模式时，语音效果自动覆盖呼吸灯效果
led->StartVoiceOutputEffect();

// 3. 进入listening模式时，语音效果自动覆盖呼吸灯效果
led->StartVoiceInputEffect();

// 4. 语音模式结束时，可以重新启动呼吸灯效果
led->StopVoiceEffect();
led->StartBreathingEffect();
```

### 状态检查
```cpp
LedEffectState state = led->GetCurrentEffectState();
if (state == LedEffectState::IDLE) {
    // 可以启动任何效果
} else if (state == LedEffectState::BREATHING) {
    // 语音效果可以覆盖呼吸灯效果
}
```

## 状态优先级
- **VOICE_INPUT/VOICE_OUTPUT**: 最高优先级，可以覆盖其他所有效果
- **BREATHING**: 中等优先级，只能在IDLE状态下启动
- **IDLE**: 最低优先级，可以启动任何效果

## 技术细节

### 新增的成员变量
```cpp
// 状态管理
LedEffectState current_effect_state_;
std::atomic<bool> effect_state_changed_;
```

### 新增的成员函数
```cpp
void SetEffectState(LedEffectState new_state);
void StopAllEffects();
bool CanStartEffect(LedEffectState effect) const;
LedEffectState GetCurrentEffectState() const;
```

### 改进的效果任务
- 所有效果任务都会检查`effect_state_changed_`标志
- 状态变化时会自动停止当前任务
- 添加了RMT错误处理和重试机制

## 测试验证
参考`led_test_example.cc`文件中的测试用例，验证修复效果。

## 注意事项
1. 确保在切换效果前检查当前状态
2. 语音效果具有最高优先级，会自动覆盖其他效果
3. 效果切换时会有短暂的延迟，这是正常的
4. 如果遇到RMT错误，系统会自动重试初始化LED strip 