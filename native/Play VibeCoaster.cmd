@echo off
setlocal
set "game=%~dp0releases\VibeCoaster-0.7.3-review.4-Windows\Play VibeCoaster.cmd"
if exist "%game%" (
    call "%game%"
    exit /b
)
echo Download and extract the Windows ZIP from:
echo https://github.com/Banrs/OpenVibeCoaster/releases/tag/native-v0.7.3-review.4
echo Then open Play VibeCoaster.cmd inside the extracted game folder.
pause
