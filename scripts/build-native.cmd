@echo off
call "D:\Toolchains\VS2022\Common7\Tools\VsDevCmd.bat" -arch=x64 -vcvars_ver=14.38
if errorlevel 1 exit /b 1
cd /d "%~dp0..\build"
cl /nologo /std:c++20 /O2 /W4 /EHsc /fp:strict /I"..\core\include" /c "..\core\src\acceleration.cpp" "..\core\src\motion.cpp" "..\core\src\track.cpp" "..\core\src\spline.cpp" "..\core\src\elements.cpp" "..\core\src\journey.cpp" "..\core\src\recipe.cpp" "..\core\src\simulation.cpp" "..\core\src\clearance.cpp" "..\core\src\terrain.cpp" "..\core\src\persistence.cpp" "..\core\src\validation.cpp"
if errorlevel 1 exit /b 1
lib /nologo /out:vibe_core.lib acceleration.obj motion.obj track.obj spline.obj elements.obj journey.obj recipe.obj simulation.obj clearance.obj terrain.obj persistence.obj validation.obj
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\src\main.cpp" vibe_core.lib /Fe:vibe.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\core_tests.cpp" vibe_core.lib /Fe:core_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\acceleration_tests.cpp" vibe_core.lib /Fe:acceleration_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\clearance_tests.cpp" vibe_core.lib /Fe:clearance_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\generator_tests.cpp" vibe_core.lib /Fe:generator_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\persistence_tests.cpp" vibe_core.lib /Fe:persistence_tests.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\src\audit.cpp" vibe_core.lib /Fe:vibe_audit.exe
if errorlevel 1 exit /b 1
cl /nologo /std:c++20 /O2 /W4 /EHsc /I"..\core\include" "..\core\tests\terrain_tests.cpp" vibe_core.lib /Fe:terrain_tests.exe
if errorlevel 1 exit /b 1
terrain_tests.exe
if errorlevel 1 exit /b 1
clearance_tests.exe
if errorlevel 1 exit /b 1
core_tests.exe
if errorlevel 1 exit /b 1
acceleration_tests.exe
exit /b %errorlevel%
