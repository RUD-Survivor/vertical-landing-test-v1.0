@echo off
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat"
cl /EHsc /MD /O2 /I include src\project.cpp src\glad.c /link /OUT:vertical-landing-test.exe lib\glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib
