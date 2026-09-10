@echo off
setlocal
set "game=%~dp0releases\VibeCoaster-0.8.2-terrain.1-6f23a9e-Windows\Play VibeCoaster.cmd"
if exist "%game%" (
    call "%game%" %*
    exit /b
)
echo Download and extract the Windows ZIP from:
echo https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.2-terrain.1-6f23a9e
echo Then open Play VibeCoaster.cmd inside the extracted game folder.
pause
