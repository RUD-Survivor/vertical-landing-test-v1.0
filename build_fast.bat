@echo off
setlocal enabledelayedexpansion

:: 设置 Visual Studio 环境路径 (根据您的系统环境调整)
set "VCVARS=D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat"

if not exist "%VCVARS%" (
    echo [错误] 找不到 vcvars64.bat，请检查路径：%VCVARS%
    exit /b 1
)

:: 初始化环境
if "%VSCMD_ARG_TGT_ARCH%"=="" (
    call "%VCVARS%"
)

:: 创建构建目录
if not exist build (
    mkdir build
)

cd build

:: 配置项目
echo [1/2] 正在配置 CMake...
cmake -G "Visual Studio 18 2026" -A x64 ..
if %ERRORLEVEL% neq 0 (
    echo [错误] CMake 配置失败
    exit /b 1
)

:: 执行增量构建
echo [2/2] 正在执行增量构建...
cmake --build . --config Release --parallel %NUMBER_OF_PROCESSORS%
if %ERRORLEVEL% neq 0 (
    echo [错误] 构建失败
    exit /b 1
)

:: 将生成的可执行文件复制到根目录 (可选，保持原来的运行习惯)
copy /Y Release\RocketSim3D.exe ..\vertical-landing-test.exe

echo.
echo [成功] 构建完成！现在您可以运行 vertical-landing-test.exe 了。
cd ..
pause
