@echo off
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat"
cl /utf-8 /std:c++17 /EHsc /MD /O2 /I dependencires\include /I vendor /I src src\main.cpp src\physics\physics_system.cpp src\control\control_system.cpp src\render\stb_impl.cpp src\simulation\stage_manager.cpp src\simulation\center_calculator.cpp src\simulation\center_visualizer.cpp src\simulation\predictor.cpp vendor\glad\glad.c /link /OUT:vertical-landing-test.exe dependencires\lib\glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib
