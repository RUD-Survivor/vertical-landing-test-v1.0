@echo off
setlocal
cd /d "%~dp0.."
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
cl /nologo /utf-8 /std:c++17 /EHsc /MD /O2 /I src /Fo"build\tests\solar_optics_numerics.obj" tools\solar_optics_numerics.cpp /Fe:build\tests\solar_optics_numerics.exe
if errorlevel 1 exit /b 1
build\tests\solar_optics_numerics.exe
exit /b %errorlevel%
