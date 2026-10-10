@echo off
setlocal
cd /d "%~dp0.."
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist build\tests mkdir build\tests
cl /nologo /utf-8 /std:c++17 /EHsc /MD /Od /DROCKET_VK_VALIDATION /I src /I vendor /I dependencires\include /I "D:\Vulkan SDK\Include" /Fo"build\tests\\" tools\vulkan_infrastructure_smoke.cpp src\render\vulkan\vk_allocator.cpp /Fe:build\tests\vulkan_infrastructure_smoke.exe /link dependencires\lib\glfw3.lib "D:\Vulkan SDK\Lib\vulkan-1.lib" user32.lib gdi32.lib shell32.lib
if errorlevel 1 exit /b 1
rem 将注入故障产生的测试日志放在测试目录，避免混入用户故障日志。
cd build\tests
vulkan_infrastructure_smoke.exe
