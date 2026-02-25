@echo off
set "VCVARS=C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
if not exist "%VCVARS%" (
    echo Cannot find vcvars64.bat
    exit /b 1
)
call "%VCVARS%"
cl /EHsc /MD /O2 src/project.cpp dependencires/library/glad.c -I dependencires/include /link /LIBPATH:dependencires/library glfw3dll.lib opengl32.lib /OUT:space_game.exe
