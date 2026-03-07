<img width="1919" height="976" alt="屏幕截图 2026-03-06 182109" src="https://github.com/user-attachments/assets/5d85747a-d171-4c24-a415-c6e3aec648a8" />
<img width="1919" height="972" alt="屏幕截图 2026-03-06 182051" src="https://github.com/user-attachments/assets/70a8914c-3467-4af5-a197-9d407dac41f6" />
<img width="1919" height="969" alt="屏幕截图 2026-03-06 181929" src="https://github.com/user-attachments/assets/6df359b9-60c3-4d10-964e-d18e59ebd7f7" />
<img width="1175" height="882" alt="屏幕截图 2026-03-05 172020" src="https://github.com/user-attachments/assets/d60032f2-3582-4f46-83b0-ef07c0930f2b" />
<img width="1919" height="970" alt="屏幕截图 2026-03-05 171529" src="https://github.com/user-attachments/assets/2c5504ca-543b-4fcd-9506-d53507f0b701" />



# 垂直着陆模拟器

一个基于物理的火箭垂直着陆模拟器，灵感来自 SpaceX Falcon 9 和 Kerbal Space Program。

## 🌿 分支说明

- **`master`** - 主分支（当前版本）
- **`feature/save-system`** - 保存系统功能演示分支
  - 包含完整的游戏保存/加载系统
  - 启动主菜单、自动保存、继续游戏等功能
  - 基于旧项目结构，作为功能演示保留
  - [查看分支](https://github.com/zhuzixuan2007/vertical-landing-test-v1.0/tree/feature/save-system)
  - [查看文档](https://github.com/zhuzixuan2007/vertical-landing-test-v1.0/blob/feature/save-system/SAVE_SYSTEM_README.md)

## 新功能：游戏保存系统 ✨

### 主要特性
- **启动菜单**: 程序启动时显示主菜单，可选择继续游戏或开始新游戏
- **自动保存**: 游戏每10秒自动保存一次，左下角显示保存提示
- **存档信息**: 菜单中显示存档的游戏时间和火箭零件数量
- **无缝恢复**: 加载存档后直接恢复到上次的游戏状态

### 使用方法
1. 启动游戏后会看到主菜单
2. 如果有存档，选择"CONTINUE GAME"继续游戏
3. 选择"NEW GAME"开始新游戏（会删除旧存档）
4. 游戏中每10秒自动保存，无需手动操作
5. 退出游戏时会自动保存最终状态

详细说明请查看 [SAVE_SYSTEM_README.md](SAVE_SYSTEM_README.md)

## 核心功能

### 火箭构建系统
- KSP风格的零件装配界面
- 多种零件类型：鼻锥、指令舱、燃料箱、引擎、助推器、结构件
- 实时计算：质量、Delta-V、推重比、阻力系数
- 可视化预览：2D示意图显示火箭结构

### 物理模拟
- 真实的轨道力学和重力模型
- 大气阻力和气压模拟
- 太阳系天体运动（太阳、行星、卫星）
- 影响球（SOI）转换
- 日食遮挡计算

### 控制系统
- 自动驾驶：PID控制器实现精确着陆
- 手动控制：WASD控制姿态，Shift/Ctrl控制油门
- 时间加速：1x到10,000,000x（真空中）

### 3D渲染
- 程序化行星渲染（地球、气态巨行星、荒芜星球）
- 真实的大气层效果
- 动态光照和阴影
- 火箭尾焰粒子效果
- 轨道预测线显示

## 操作指南

### 构建阶段
- **方向键**: 浏览零件目录
- **Tab**: 切换目录/装配模式
- **Enter**: 添加零件
- **Delete**: 删除零件
- **Page Up/Down**: 调整零件顺序
- **Space**: 发射火箭

### 飞行阶段
- **Space**: 发射
- **Tab**: 切换自动/手动模式
- **WASD**: 姿态控制（手动模式）
- **Shift/Ctrl**: 增加/减少油门
- **Z/X**: 最大/最小油门
- **1-8**: 时间加速（1x, 10x, 100x, 1000x, ...）
- **C**: 切换相机模式（轨道/跟踪/全景）
- **H**: 显示/隐藏HUD
- **O**: 显示/隐藏轨道线
- **R**: 切换轨道参考系（地球/太阳）
- **,/.**: 切换焦点天体

## 编译说明

### Windows (MSVC)
```cmd
build.cmd
```

### 依赖项
- GLFW 3.x
- GLAD (OpenGL 3.3+)
- C++11 或更高版本

## 文件结构

```
src/
├── main.cpp            # 主程序
├── core/rocket_state.h # 火箭状态定义
├── simulation/rocket_builder.h # 火箭构建系统
├── physics/physics_system.h/cpp # 物理引擎
├── control/control_system.h/cpp # 控制系统
├── render/renderer3d.h # 3D渲染引擎
├── math/math3d.h      # 3D数学库
├── save_system.h      # 保存系统 (新增)
└── menu_system.h      # 菜单系统 (新增)
```

## 技术亮点

- **双精度物理**: 使用双精度浮点数避免大尺度下的精度损失
- **浮动原点**: 动态调整渲染中心，避免远距离网格撕裂
- **程序化渲染**: 使用噪声函数生成真实的行星表面
- **多层大气**: 体积散射实现真实的大气效果
- **开普勒轨道**: 精确的轨道预测和可视化

## 已知问题

- 高速时间加速可能导致物理不稳定
- 极端轨道可能出现数值误差
- 部分行星纹理需要优化

## 未来计划

- [ ] 多级火箭支持
- [ ] 更多行星和卫星
- [ ] 任务系统
- [ ] 重返大气层效果
- [ ] 多人模式
- [x] 游戏保存/加载系统

## 致谢

灵感来源：
- SpaceX Falcon 9 着陆技术
- Kerbal Space Program
- RSS (Real Solar System) Mod

## 许可证

本项目仅供学习和研究使用。
