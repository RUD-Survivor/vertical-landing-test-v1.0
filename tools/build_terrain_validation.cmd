@echo off
setlocal
cd /d "%~dp0.."
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist build\tests mkdir build\tests
for %%S in (terrain.vert terrain.frag terrain_shadow.vert ocean.frag terrain_validate.comp) do (
    "D:\Vulkan SDK\Bin\glslc.exe" src\render\shaders\%%S -o src\render\shaders\spirv\%%S.spv
    if errorlevel 1 exit /b 1
)
"D:\Vulkan SDK\Bin\glslc.exe" -O -DOCEAN_SURFACE=1 src\render\shaders\terrain.vert -o src\render\shaders\spirv\ocean.vert.spv
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /std:c++17 /EHsc /MD /O2 /fp:precise /DROCKET_VK_VALIDATION /I src /I vendor /I dependencires\include /I "D:\Vulkan SDK\Include" /Fo"build\tests\\" tools\terrain_validation.cpp src\render\vulkan\vk_allocator.cpp src\render\stb_impl.cpp src\render\data\blue_noise_sobol.cpp src\render\data\blue_noise_scrambling.cpp src\render\data\blue_noise_ranking.cpp /Fe:build\tests\terrain_validation.exe /link dependencires\lib\glfw3.lib "D:\Vulkan SDK\Lib\vulkan-1.lib" user32.lib gdi32.lib shell32.lib
