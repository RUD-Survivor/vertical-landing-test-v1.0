@echo off
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat"
cl /EHsc /MD /O2 /I dependencies/include /I vendor /I src src/main.cpp src/physics/physics_system.cpp src/control/control_system.cpp src/simulation/stage_manager.cpp vendor/glad/glad.c /link /OUT:vertical-landing-test.exe dependencies/lib/glfw3.lib opengl32.lib user32.lib gdi32.lib shell32.lib
