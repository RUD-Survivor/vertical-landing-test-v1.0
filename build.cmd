@echo off
set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" (
    echo Cannot find vcvars64.bat
    exit /b 1
)
call "%VCVARS%"
cl /EHsc /MD /O2 src/project.cpp src/glad.c src/physics_system.cpp src/control_system.cpp -I dependencires/include /link /LIBPATH:dependencires/lib glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib /OUT:vertical-landing-test.exe
