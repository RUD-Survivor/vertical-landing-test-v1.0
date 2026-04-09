@echo off
set "VCVARS=D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" (
    echo Cannot find vcvars64.bat
    exit /b 1
)
call "%VCVARS%"
cl /std:c++17 /utf-8 /EHsc /MD /O2 src/main.cpp src/physics/physics_system.cpp src/control/control_system.cpp src/simulation/stage_manager.cpp src/simulation/center_calculator.cpp src/simulation/center_visualizer.cpp src/simulation/predictor.cpp vendor/glad/glad.c src/render/stb_impl.cpp -I dependencires/include -I vendor -I src /link /LIBPATH:dependencires/lib glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib /OUT:stellar_ascend.exe
