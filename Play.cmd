@echo off
pwsh -NoProfile -File "%~dp0scripts\play.ps1" %*
if errorlevel 1 pause
