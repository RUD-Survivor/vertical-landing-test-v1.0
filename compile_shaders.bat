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

echo [ 1/21] mesh.vert
%GLSLC% %SHADER_DIR%\mesh.vert -o %OUT_DIR%\mesh.vert.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 2/21] mesh.frag
%GLSLC% %SHADER_DIR%\mesh.frag -o %OUT_DIR%\mesh.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 3/21] earth.frag
%GLSLC% %SHADER_DIR%\earth.frag -o %OUT_DIR%\earth.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 4/21] gas_giant.frag
%GLSLC% %SHADER_DIR%\gas_giant.frag -o %OUT_DIR%\gas_giant.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 5/21] barren.frag
%GLSLC% %SHADER_DIR%\barren.frag -o %OUT_DIR%\barren.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 6/21] mercury.frag
%GLSLC% %SHADER_DIR%\mercury.frag -o %OUT_DIR%\mercury.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 7/21] moon.frag
%GLSLC% %SHADER_DIR%\moon.frag -o %OUT_DIR%\moon.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 8/21] venus.frag
%GLSLC% %SHADER_DIR%\venus.frag -o %OUT_DIR%\venus.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [ 9/21] mars.frag
%GLSLC% %SHADER_DIR%\mars.frag -o %OUT_DIR%\mars.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [10/21] jupiter.frag
%GLSLC% %SHADER_DIR%\jupiter.frag -o %OUT_DIR%\jupiter.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [11/21] saturn.frag
%GLSLC% %SHADER_DIR%\saturn.frag -o %OUT_DIR%\saturn.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [12/21] uranus.frag
%GLSLC% %SHADER_DIR%\uranus.frag -o %OUT_DIR%\uranus.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [13/21] neptune.frag
%GLSLC% %SHADER_DIR%\neptune.frag -o %OUT_DIR%\neptune.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [14/21] ring.frag
%GLSLC% %SHADER_DIR%\ring.frag -o %OUT_DIR%\ring.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [15/21] skybox.vert
%GLSLC% %SHADER_DIR%\skybox.vert -o %OUT_DIR%\skybox.vert.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [16/21] skybox.frag
%GLSLC% %SHADER_DIR%\skybox.frag -o %OUT_DIR%\skybox.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [17/21] exhaust.vert
%GLSLC% %SHADER_DIR%\exhaust.vert -o %OUT_DIR%\exhaust.vert.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [18/21] exhaust.frag
%GLSLC% %SHADER_DIR%\exhaust.frag -o %OUT_DIR%\exhaust.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [19/21] ribbon.vert
%GLSLC% %SHADER_DIR%\ribbon.vert -o %OUT_DIR%\ribbon.vert.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [20/21] ribbon.frag
%GLSLC% %SHADER_DIR%\ribbon.frag -o %OUT_DIR%\ribbon.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [21/21] billboard.vert + billboard.frag + lens_flare.vert + lens_flare.frag
%GLSLC% %SHADER_DIR%\billboard.vert -o %OUT_DIR%\billboard.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\billboard.frag -o %OUT_DIR%\billboard.frag.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\lens_flare.vert -o %OUT_DIR%\lens_flare.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\lens_flare.frag -o %OUT_DIR%\lens_flare.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [TAA] taa.vert + taa.frag
%GLSLC% %SHADER_DIR%\taa.vert -o %OUT_DIR%\taa.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\taa.frag -o %OUT_DIR%\taa.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [SVO/Atmo/Veg/Terrain/HUD] svo.vert + svo.frag + atmo.vert + atmo.frag + vegetation.vert + vegetation.frag + terrain.vert + terrain.frag + hud2d.vert + hud2d.frag
%GLSLC% %SHADER_DIR%\svo.vert -o %OUT_DIR%\svo.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\svo.frag -o %OUT_DIR%\svo.frag.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\atmo.vert -o %OUT_DIR%\atmo.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\atmo.frag -o %OUT_DIR%\atmo.frag.spv
if %ERRORLEVEL% neq 0 goto :fail
echo [Cloud] cloud.vert + cloud.frag + cloud bake compute
%GLSLC% %SHADER_DIR%\cloud.vert -o %OUT_DIR%\cloud.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\cloud.frag -o %OUT_DIR%\cloud.frag.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\cloud\cloud_basicnoise.glsl -o %OUT_DIR%\cloud_basicnoise.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\cloud\cloud_detailnoise.glsl -o %OUT_DIR%\cloud_detailnoise.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\cloud\cloud_curlnoise.glsl -o %OUT_DIR%\cloud_curlnoise.comp.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [Atmosphere LUT] transmittance/multiScatter/skyView/skyIrradiance (real-time per-frame bake)
%GLSLC% -fshader-stage=compute %SHADER_DIR%\atmosphere\transmittance_lut.glsl -o %OUT_DIR%\transmittance_lut.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\atmosphere\multi_scatter_lut.glsl -o %OUT_DIR%\multi_scatter_lut.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\atmosphere\skyview_lut.glsl -o %OUT_DIR%\skyview_lut.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\atmosphere\sky_irradiance_capture.glsl -o %OUT_DIR%\sky_irradiance_capture.comp.spv
if %ERRORLEVEL% neq 0 goto :fail

echo [Cloud Phase1] real flower cloud compute passes (raymarch/reconstruct/composite)
%GLSLC% -fshader-stage=compute %SHADER_DIR%\cloud\cloud_raymarching.glsl -o %OUT_DIR%\cloud_raymarching.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\cloud\cloud_reconstruct.glsl -o %OUT_DIR%\cloud_reconstruct.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% -fshader-stage=compute %SHADER_DIR%\cloud\cloud_composite.glsl -o %OUT_DIR%\cloud_composite.comp.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\vegetation.vert -o %OUT_DIR%\vegetation.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\vegetation.frag -o %OUT_DIR%\vegetation.frag.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\terrain.vert -o %OUT_DIR%\terrain.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\terrain.frag -o %OUT_DIR%\terrain.frag.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\hud2d.vert -o %OUT_DIR%\hud2d.vert.spv
if %ERRORLEVEL% neq 0 goto :fail
%GLSLC% %SHADER_DIR%\hud2d.frag -o %OUT_DIR%\hud2d.frag.spv
if %ERRORLEVEL% neq 0 goto :fail

echo.
echo === All shaders compiled successfully ===
echo Output: %OUT_DIR%
exit /b 0

:fail
echo.
echo [Error] Shader compilation failed (see output above).
exit /b 1
