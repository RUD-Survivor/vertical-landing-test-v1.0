@echo off
setlocal
cd /d "%~dp0.."
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist build\tests mkdir build\tests
cl /nologo /utf-8 /std:c++17 /EHsc /MD /O2 /DROCKET_VK_VALIDATION /I src /I vendor /I dependencires\include /I "D:\Vulkan SDK\Include" /Fo"build\tests\\" tools\ocean_gpu_validation.cpp src\render\vulkan\vk_allocator.cpp src\render\stb_impl.cpp src\render\data\blue_noise_sobol.cpp src\render\data\blue_noise_scrambling.cpp src\render\data\blue_noise_ranking.cpp /Fe:build\tests\sky_gpu_validation.exe /link dependencires\lib\glfw3.lib "D:\Vulkan SDK\Lib\vulkan-1.lib" user32.lib gdi32.lib shell32.lib

