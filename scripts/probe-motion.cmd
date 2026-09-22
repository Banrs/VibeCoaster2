@echo off
call "%~dp0build-native.cmd"
if errorlevel 1 exit /b 1
cd /d "%~dp0.."
build\vibe.exe probe recipes\default.vcr
exit /b %errorlevel%
