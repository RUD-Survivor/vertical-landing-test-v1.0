@echo off
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat"
cl /EHsc /MD /O2 /I dependencires\include src\project.cpp src\glad.c /link /OUT:vertical-landing-test.exe dependencires\lib\glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib
