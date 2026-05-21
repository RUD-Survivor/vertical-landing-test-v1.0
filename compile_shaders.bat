@echo off
setlocal

if "%VULKAN_SDK%"=="" (
    echo [Error] VULKAN_SDK environment variable not set.
    echo         Install Vulkan SDK and reopen this terminal.
    exit /b 1
)

set GLSLC="%VULKAN_SDK%\Bin\glslc.exe"
set SHADER_DIR=src\render\shaders
set OUT_DIR=src\render\shaders\spirv

if not exist %OUT_DIR% mkdir %OUT_DIR%

echo === Compiling Vulkan shaders ===

echo [1/4] mesh.vert
%GLSLC% %SHADER_DIR%\mesh.vert -o %OUT_DIR%\mesh.vert.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [2/4] mesh.frag
%GLSLC% %SHADER_DIR%\mesh.frag -o %OUT_DIR%\mesh.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [3/4] taa.vert
%GLSLC% %SHADER_DIR%\taa.vert -o %OUT_DIR%\taa.vert.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [4/4] taa.frag
%GLSLC% %SHADER_DIR%\taa.frag -o %OUT_DIR%\taa.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo.
echo === All shaders compiled successfully ===
echo Output: %OUT_DIR%
exit /b 0

:fail
echo.
echo [Error] Shader compilation failed (see output above).
exit /b 1
