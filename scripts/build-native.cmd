@echo off
call "D:\Toolchains\VS2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -vcvars_ver=14.38
if errorlevel 1 exit /b 1
cd /d "%~dp0..\build"
cl /nologo /std:c++20 /O2 /W4 /EHsc /fp:strict /I"..\core\include" /c "..\core\src\acceleration.cpp" "..\core\src\motion.cpp" "..\core\src\track.cpp" "..\core\src\recipe.cpp" "..\core\src\simulation.cpp" "..\core\src\persistence.cpp"
if errorlevel 1 exit /b 1
lib /nologo /out:vibe_core.lib acceleration.obj motion.obj track.obj recipe.obj simulation.obj persistence.obj
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\src\main.cpp" vibe_core.lib /Fe:vibe.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\core_tests.cpp" vibe_core.lib /Fe:core_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\acceleration_tests.cpp" vibe_core.lib /Fe:acceleration_tests.exe
if errorlevel 1 exit /b 1
core_tests.exe
if errorlevel 1 exit /b 1
acceleration_tests.exe
exit /b %errorlevel%
