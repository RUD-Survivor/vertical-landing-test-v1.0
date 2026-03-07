@echo off
set "VCVARS=D:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" (
    echo Cannot find vcvars64.bat
    exit /b 1
)
call "%VCVARS%"
cl /EHsc /MD /O2 src/main.cpp vendor/glad/glad.c src/physics/physics_system.cpp src/control/control_system.cpp src/simulation/stage_manager.cpp -I src -I dependencies/include /link /LIBPATH:dependencies/lib glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib /OUT:vertical-landing-test.exe
