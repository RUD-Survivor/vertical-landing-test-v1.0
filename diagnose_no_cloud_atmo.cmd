@echo off
setlocal
cd /d "%~dp0"
set "ROCKET_DISABLE_CLOUD=1"
set "ROCKET_DISABLE_ATMO=1"
build\debug\Stella_Ascend.exe > gpu_console_no_cloud_atmo.log 2>&1
