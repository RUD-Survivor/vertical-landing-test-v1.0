@echo off
setlocal
cd /d "%~dp0.."
call "D:\Microsoft Visual Studio 2026\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
if not exist build\tests mkdir build\tests
cl /nologo /utf-8 /std:c++17 /EHsc /MD /O2 /I src tests\ocean_numerics.cpp /Fo:build\tests\ocean_numerics.obj /Fe:build\tests\ocean_numerics.exe
if errorlevel 1 exit /b 1
build\tests\ocean_numerics.exe
