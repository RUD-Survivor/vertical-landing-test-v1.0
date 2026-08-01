@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

:: Create build output directory
if not exist "build" mkdir "build"

:: Get timestamp safely (works on any locale)
for /f "tokens=2 delims==" %%I in ('wmic os get localdatetime /value') do set "TS=%%I"
set "TIMESTAMP=%TS:~0,4%%TS:~4,2%%TS:~6,2%_%TS:~8,2%%TS:~10,2%%TS:~12,2%"

echo ================================================================
echo   RocketSim3D - Build Script
echo   Compiler: MSVC ^(cl.exe^) ^| Graphics: Vulkan
echo ================================================================
echo.

:: ================================================================
:: Step 0: Detect MSVC environment
:: ================================================================
set "VCVARS_DEFAULT=D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat"

if exist "%VCVARS_DEFAULT%" (
    set "VCVARS=%VCVARS_DEFAULT%"
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat"
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat" (
    set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Enterprise\VC\Auxiliary\Build\vcvars64.bat"
)

if not defined VCVARS (
    echo [ERROR] Cannot find Visual Studio.
    echo   Tried: VS 2026, VS 2022 Community/Professional/Enterprise
    echo   Install: https://visualstudio.microsoft.com/downloads/
    exit /b 1
)

call "%VCVARS%" > nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Failed to initialize MSVC environment.
    exit /b 1
)
echo [OK] MSVC: %VCVARS%
echo.

:: ================================================================
:: Step 1: Detect Vulkan SDK
:: ================================================================
set "VK_INCLUDE="
set "VK_LIB="

if defined VULKAN_SDK (
    set "VK_INCLUDE=%VULKAN_SDK%\Include"
    set "VK_LIB=%VULKAN_SDK%\Lib\vulkan-1.lib"
) else if exist "D:\Vulkan SDK\Include\vulkan\vulkan.h" (
    set "VK_INCLUDE=D:\Vulkan SDK\Include"
    set "VK_LIB=D:\Vulkan SDK\Lib\vulkan-1.lib"
) else if exist "C:\VulkanSDK\1.3.296.0\Include\vulkan\vulkan.h" (
    set "VK_INCLUDE=C:\VulkanSDK\1.3.296.0\Include"
    set "VK_LIB=C:\VulkanSDK\1.3.296.0\Lib\vulkan-1.lib"
)

if not "!VK_INCLUDE!"=="" (
    set "VK_FLAG=/D USE_VULKAN"
    set "VK_INC=/I "!VK_INCLUDE!""
    set "VK_LINK="!VK_LIB!""
    echo [OK] Vulkan SDK: !VK_INCLUDE!
) else (
    set "VK_FLAG="
    set "VK_INC="
    set "VK_LINK="
    echo [WARN] Vulkan SDK not found - building with OpenGL fallback.
    echo        Install: https://vulkan.lunarg.com/
)
echo.

:: ================================================================
:: Step 2: Compile shaders (Vulkan only)
:: ================================================================
if not "!VK_INCLUDE!"=="" (
    if exist "compile_shaders.bat" (
        echo [2/3] Compiling shaders...
        call compile_shaders.bat
        if !ERRORLEVEL! neq 0 (
            echo [WARN] Shader compilation had errors, continuing...
        )
    ) else (
        echo [2/3] Skipping shaders ^(compile_shaders.bat not found^)
    )
) else (
    echo [2/3] Skipping shaders ^(no Vulkan^)
)
echo.

:: ================================================================
:: Step 3: Compile C++ source
:: ================================================================
echo [3/3] Compiling C++ source...

:: --- Config: debug (this script) | release via build_release.cmd ---
:: Args: [debug|release] [force]
::   force = rebuild all .cpp (ignore timestamps / headers)
set "CFG=debug"
set "FORCE_REBUILD=0"
if /I "%~1"=="release" set "CFG=release"
if /I "%~1"=="debug" set "CFG=debug"
if /I "%~1"=="force" set "FORCE_REBUILD=1"
if /I "%~2"=="force" set "FORCE_REBUILD=1"
if /I "%~2"=="release" set "CFG=release"
if /I "%~2"=="debug" set "CFG=debug"

if /I "%CFG%"=="release" (
    set "CFLAGS=/nologo /utf-8 /std:c++17 /EHsc /MD /O2 /W3 /DNDEBUG"
) else (
    :: Use /MD (same as glfw3.lib). Do not use /MDd or /D_DEBUG.
    set "CFLAGS=/nologo /utf-8 /std:c++17 /EHsc /MD /Od /Zi /FS /W3"
)

:: Separate obj/exe trees so debug/release never share stale objects
set "OUTPUT_DIR=build\%CFG%"
set "OBJ_DIR=build\obj\%CFG%"
set "OUTPUT_EXE=%OUTPUT_DIR%\Stella_Ascend.exe"
set "BUILD_LOG=%OUTPUT_DIR%\build_%TIMESTAMP%.log"
if not exist "%OUTPUT_DIR%" mkdir "%OUTPUT_DIR%"
if not exist "%OBJ_DIR%" mkdir "%OBJ_DIR%"

:: --- Source files ---
set "SRC=src\main.cpp"
set "SRC=%SRC% src\physics\physics_system.cpp"
set "SRC=%SRC% src\physics\aerodynamics_system.cpp"
set "SRC=%SRC% src\physics\ground_collision_system.cpp"
set "SRC=%SRC% src\physics\chunk_body_collision.cpp"
set "SRC=%SRC% src\physics\voxel\voxel_types.cpp"
set "SRC=%SRC% src\physics\voxel\vessel_voxel_model.cpp"
set "SRC=%SRC% src\physics\voxel\vessel_voxelizer.cpp"
set "SRC=%SRC% src\control\control_system.cpp"
set "SRC=%SRC% src\render\stb_impl.cpp"
set "SRC=%SRC% src\render\data\blue_noise_sobol.cpp"
set "SRC=%SRC% src\render\data\blue_noise_scrambling.cpp"
set "SRC=%SRC% src\render\data\blue_noise_ranking.cpp"
set "SRC=%SRC% src\simulation\stage_manager.cpp"
set "SRC=%SRC% src\simulation\structural_state.cpp"
set "SRC=%SRC% src\simulation\center_calculator.cpp"
set "SRC=%SRC% src\simulation\center_visualizer.cpp"
set "SRC=%SRC% src\simulation\predictor.cpp"
set "SRC=%SRC% src\render\vulkan\vk_allocator.cpp"
set "SRC=%SRC% vendor\imgui\imgui.cpp"
set "SRC=%SRC% vendor\imgui\imgui_draw.cpp"
set "SRC=%SRC% vendor\imgui\imgui_widgets.cpp"
set "SRC=%SRC% vendor\imgui\imgui_tables.cpp"
set "SRC=%SRC% vendor\imgui\backends\imgui_impl_glfw.cpp"
set "SRC=%SRC% vendor\imgui\backends\imgui_impl_vulkan.cpp"

:: --- Includes ---
set "INCLUDES=/I dependencires\include /I vendor /I vendor\imgui /I vendor\imgui\backends /I src"

:: --- Libraries ---
set "LIBS=dependencires\lib\glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib"

echo   Config:   %CFG%
if "!FORCE_REBUILD!"=="1" (echo   Force:    ON ^(rebuild all .cpp^)) else (echo   Force:    off)
echo   Compiler:  cl
echo   Flags:     %CFLAGS% %VK_FLAG%
echo   .exe:      %OUTPUT_EXE%
echo   .obj dir:  %OBJ_DIR%\
echo   Log:       %BUILD_LOG%
echo.

:: Stale units = missing .obj, OR .cpp newer, OR included headers newer.
:: Logic lives in build_find_stale.ps1 (main.cpp also watches src\ + vendor\imgui\ headers).
:: Use: build_debug.cmd force   to rebuild every .cpp.
set "STALE="
set "SOURCES_LST=%OBJ_DIR%\sources.lst"
> "%SOURCES_LST%" (
    for %%F in (%SRC%) do echo %%F
)
set "FORCE_PS="
if "!FORCE_REBUILD!"=="1" set "FORCE_PS=-Force"
for /f "usebackq delims=" %%S in (`powershell -NoProfile -ExecutionPolicy Bypass -File "build_find_stale.ps1" -ObjDir "%OBJ_DIR%" -SourcesFile "%SOURCES_LST%" !FORCE_PS!`) do set "STALE=%%S"

echo. > "%BUILD_LOG%"
set "BUILD_RESULT=0"
if not "!STALE!"=="" (
    if "!FORCE_REBUILD!"=="1" (
        echo   Force rebuild all units:!STALE!
    ) else (
        echo   Compiling stale units:!STALE!
    )
    cl %CFLAGS% /c /MP %VK_FLAG% %INCLUDES% %VK_INC% /Fo"%OBJ_DIR%\\" /Fd"%OBJ_DIR%\vc.pdb" !STALE! >> "%BUILD_LOG%" 2>&1
    set BUILD_RESULT=!ERRORLEVEL!
) else (
    echo   All .obj up to date ^(cpp + headers^), skip compile.
)

:: Assemble OBJS in SRC order; abort link if any .obj is missing.
set "OBJS="
set "MISSING_OBJ="
for %%F in (%SRC%) do (
    set "OBJS=!OBJS! "%OBJ_DIR%\%%~nF.obj""
    if not exist "%OBJ_DIR%\%%~nF.obj" set "MISSING_OBJ=1"
)
if defined MISSING_OBJ (
    echo   [ERROR] Missing .obj files - compile did not produce all units. See log.
    if !BUILD_RESULT! equ 0 set BUILD_RESULT=1
)
if !BUILD_RESULT! equ 0 (
    echo   Linking...
    link /nologo /OUT:"%OUTPUT_EXE%" !OBJS! %LIBS% %VK_LINK% >> "%BUILD_LOG%" 2>&1
    set BUILD_RESULT=!ERRORLEVEL!
)
:: ================================================================
:: Generate build report
:: ================================================================
set "REPORT=%OUTPUT_DIR%\build_report.txt"

(
    echo ================================================================
    echo   RocketSim3D - Build Report
    echo ================================================================
    echo   Time:    %TIMESTAMP%
    echo   Machine: %COMPUTERNAME%
    echo   User:    %USERNAME%
    echo.
    echo --- Environment ---
    echo   MSVC:    %VCVARS%
    if not "!VK_INCLUDE!"=="" (
        echo   Vulkan:  !VK_INCLUDE!
    ) else (
        echo   Vulkan:  Not detected ^(OpenGL fallback^)
    )
    echo.
    echo --- Compiler ---
    echo   %CFLAGS% %VK_FLAG%
    echo.
    echo --- Source Files ---
    for %%f in (%SRC%) do echo   %%f
    echo.
    echo --- Libraries ---
    echo   %LIBS% %VK_LINK%
    echo.
    echo --- Output ---
    echo   .exe: %OUTPUT_EXE%
    echo   .obj: %OBJ_DIR%\
    echo.
    echo ================================================================
    if %BUILD_RESULT% equ 0 (
        echo   RESULT: SUCCESS
        echo ================================================================
    ) else (
        echo   RESULT: FAILED ^(exit code: %BUILD_RESULT%^)
        echo ================================================================
        echo.
        echo --- Last 30 lines of build log ---
        type "%BUILD_LOG%" 2>nul | powershell -Command "$input | Select-Object -Last 30"
    )
) > "%REPORT%"

type "%REPORT%"

:: ================================================================
:: Final
:: ================================================================
echo.
if %BUILD_RESULT% equ 0 (
    echo ================================================================
    echo   BUILD SUCCESSFUL!
    echo   Run: %OUTPUT_EXE%
    echo ================================================================
    :: Copy exe to root for convenience
    copy /Y "%OUTPUT_EXE%" "Stella_Ascend.exe" > nul
    echo   ^(also copied to root: Stella_Ascend.exe^)
) else (
    echo ================================================================
    echo   BUILD FAILED!
    echo   Full log: %BUILD_LOG%
    echo   Report:   %REPORT%
    echo ================================================================
    :: If the window closes immediately when double-clicked, use: cmd /k build_debug.cmd
    exit /b %BUILD_RESULT%
)

endlocal
