# RocketSim3D 🚀

火箭飞行模拟器 —— 基于 C++ / Vulkan / OpenGL 的 3D 实时模拟。

---

## 环境要求

| 依赖 | 版本要求 | 下载 |
|------|---------|------|
| **Visual Studio** | 2022 或 2026（含"使用 C++ 的桌面开发"） | [visualstudio.microsoft.com](https://visualstudio.microsoft.com/downloads/) |
| **Vulkan SDK** | 1.3+ | [vulkan.lunarg.com](https://vulkan.lunarg.com/) |
| **Git LFS**（可选） | 用于大文件 | [git-lfs.com](https://git-lfs.com) |

> ⚠️ 如果未安装 Vulkan SDK，项目会回退到 OpenGL 模式编译。

---

## 快速开始

### 1. 克隆项目

```powershell
git clone <仓库地址>
cd RocketSim3D
```

### 2. 编译着色器（仅 Vulkan 模式）

```powershell
.\compile_shaders.bat
```

### 3. 一键编译

```powershell
.\build.cmd
```

### 4. 运行

```powershell
.\vertical-landing-test.exe
```

---

## 三种构建方式

### 方式一：命令行一键构建（推荐 ✅）

```powershell
.\build.cmd
```

- 自动检测 VS 2022/2026 和 Vulkan SDK
- 自动生成 `build_report.txt`（编译报告）和 `build_report_YYYYMMDD_HHMMSS.log`（详细日志）
- 适用于 **你、AI、外部用户**

### 方式二：Visual Studio IDE

```powershell
# 双击或在 VS 中打开
start "vertical-landing-test v1.0.slnx"
```

然后按 `Ctrl+Shift+B` 构建，`F5` 调试运行。

### 方式三：VS Code（Ctrl+Shift+B）

已在 `.vscode/tasks.json` 中配置，直接按 `Ctrl+Shift+B` 触发。

---

## 编译报告

每次运行 `build.cmd` 会在项目根目录生成两份报告：

| 文件 | 内容 |
|------|------|
| `build_report.txt` | 概要报告（环境、源文件列表、成功/失败） |
| `build_report_YYYYMMDD_HHMMSS.log` | 完整编译日志 |

示例报告内容：
```
================================================================
  RocketSim3D - Build Report
================================================================
  Date:    2026/06/15 14:30:00
  Machine: MY-PC
  User:    魏DELL

  --- Environment ---
  MSVC:    D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat
  Vulkan:  D:\Vulkan SDK\Include

  --- Compiler Flags ---
  /nologo /utf-8 /std:c++17 /EHsc /MD /O2 /W3 /D USE_VULKAN

  --- Source Files ---
  src\main.cpp
  src\physics\physics_system.cpp
  ...

================================================================
  RESULT: SUCCESS
================================================================
```

---

## AI / 自动化构建

对于 AI Agent 或 CI/CD，直接调用：

```powershell
.\build.cmd
if %ERRORLEVEL% neq 0 (
    echo Build failed
    type build_report.txt
    exit /b 1
)
```

可解析 `build_report.txt` 中的 `RESULT: SUCCESS` 或 `RESULT: FAILED` 来判断构建状态。

---

## 项目结构

```
RocketSim3D/
├── build.cmd              ← 一键构建（推荐入口）
├── compile_shaders.bat    ← 着色器编译
├── src/                   ← 源代码
│   ├── main.cpp
│   ├── physics/           ← 物理系统
│   ├── control/           ← 控制系统
│   ├── render/            ← 渲染（Vulkan/OpenGL）
│   ├── simulation/        ← 模拟（分级、重心等）
│   └── scene/             ← 场景管理
├── vendor/                ← 第三方库（GLAD, ImGui, EnTT, VMA）
├── dependencires/         ← 外部依赖（GLFW）
├── assets/                ← 游戏资源（模型、贴图、字体）
└── vertical-landing-test v1.0.slnx  ← VS 解决方案
```

---

## 常见问题

### Q: 提示找不到 `vcvars64.bat`？
请安装 Visual Studio 2022+ 并勾选"使用 C++ 的桌面开发"工作负载。

### Q: 提示找不到 `vulkan-1.lib`？
安装 [Vulkan SDK](https://vulkan.lunarg.com/)，或直接不带 Vulkan 编译（自动回退 OpenGL）。

### Q: `build_fast.bat` 运行报错？
该项目暂无 `CMakeLists.txt`，请使用 `build.cmd` 直接编译。

### Q: 如何贡献代码？
1. 从 `main` 分支创建 `feature/xxx`
2. 小步提交，一个 commit 做一件事
3. 确保 `.\build.cmd` 通过后再提交
4. 提 Pull Request 合并回 `main`
