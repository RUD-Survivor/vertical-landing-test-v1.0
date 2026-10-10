@echo off
setlocal
cd /d "%~dp0"
set "ROCKET_DISABLE_CLOUD=1"
set "ROCKET_DISABLE_ATMO="
build\debug\Stella_Ascend.exe > gpu_console_no_cloud.log 2>&1
