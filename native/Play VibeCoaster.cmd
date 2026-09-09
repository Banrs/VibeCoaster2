@echo off
setlocal
set "game=%~dp0releases\VibeCoaster-0.8.0-flow.1-Windows\Play VibeCoaster.cmd"
if exist "%game%" (
    call "%game%" %*
    exit /b
)
echo Download and extract the Windows ZIP from:
echo https://github.com/Banrs/VibeCoaster2/releases/tag/native-v0.8.0-flow.1
echo Then open Play VibeCoaster.cmd inside the extracted game folder.
pause
