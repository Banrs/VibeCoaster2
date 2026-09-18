# Build the core, package the game, and point "Play VibeCoaster.lnk" at the result.
# The shortcut only moves once the package exists and its executable runs, so a
# failed build leaves the last working game in place.
#   powershell -ExecutionPolicy Bypass -File build-and-play.ps1 [-SkipTests] [-Verify]
[CmdletBinding()]
param([switch]$SkipTests, [switch]$Verify)
$ErrorActionPreference = 'Stop'
$Source = $PSScriptRoot
$Tools = 'D:\Coding\Codex\vibecoasterlegacy\archive-2026-09-13\removed\native\build\tools'
$CMake = Join-Path $Tools 'cmake\data\bin\cmake.exe'
$CTest = Join-Path $Tools 'cmake\data\bin\ctest.exe'
$BuildDir = 'D:\Coding\Codex\vibecoasterlegacy\ws-build-fresh'
$Unreal = 'D:\Games\Epic Games\UE_5.8'
$Stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$Frozen = "D:\Coding\Codex\vibecoasterlegacy\graded-082-source\$Stamp"
$Package = "D:\Coding\Codex\vibecoasterlegacy\graded-082-game\run-$Stamp"

Write-Host "== building core"
cmd /c "call D:\Toolchains\VS2022\VC\Auxiliary\Build\vcvars64.bat >nul && set PATH=$Tools\bin;%PATH% && `"$CMake`" --build `"$BuildDir`" --parallel 6"
if ($LASTEXITCODE -ne 0) { throw 'Core build failed; shortcut left untouched.' }

if (-not $SkipTests) {
    Write-Host "== core tests"
    & $CTest --test-dir $BuildDir --output-on-failure --parallel 3 --timeout 1800
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed; shortcut left untouched.' }
}

Write-Host "== freezing source"
New-Item -ItemType Directory -Path $Frozen -Force | Out-Null
foreach ($f in Get-ChildItem -LiteralPath (Join-Path $Source 'native') -Recurse -File) {
    $rel = $f.FullName.Substring($Source.Length + 1)
    $dst = Join-Path $Frozen $rel
    New-Item -ItemType Directory -Path ([System.IO.Path]::GetDirectoryName($dst)) -Force | Out-Null
    Copy-Item -LiteralPath $f.FullName -Destination $dst
}

Write-Host "== packaging game (several minutes)"
& (Join-Path $Frozen 'native\unreal\scripts\package.ps1') -UnrealRoot $Unreal -OutputDirectory $Package
if ($LASTEXITCODE -ne 0) { throw 'Packaging failed; shortcut left untouched.' }

$exe = Get-ChildItem -LiteralPath $Package -Recurse -Filter 'VibeCoaster.exe' |
    Where-Object { $_.FullName -like '*\Binaries\Win64\*' } | Select-Object -First 1
if (-not $exe) { throw 'Packaged executable not found; shortcut left untouched.' }

if ($Verify) {
    Write-Host "== packaged runtime check"
    $evidence = "D:\Coding\Codex\vibecoasterlegacy\task-20260917-graded-flats\runtime-$Stamp"
    & 'D:\Coding\Codex\vibecoasterlegacy\task-20260917-graded-flats\run-runtime-check.ps1' -Executable $exe.FullName -Output $evidence
    $result = Join-Path $evidence 'events\result.json'
    while (-not (Test-Path $result)) { Start-Sleep -Seconds 5 }
    $status = (Get-Content $result -Raw | ConvertFrom-Json).status
    if ($status -ne 'automated-smoke-passed') { throw "Runtime check said '$status'; shortcut left untouched." }
    Write-Host "   runtime check passed"
}

$shell = New-Object -ComObject WScript.Shell
$link = $shell.CreateShortcut((Join-Path $Source 'Play VibeCoaster.lnk'))
$link.TargetPath = $exe.FullName
$link.WorkingDirectory = (Get-Item $exe.FullName).Directory.Parent.Parent.Parent.FullName
$link.Description = "VibeCoaster - packaged $Stamp"
$link.Save()
Write-Host "== Play VibeCoaster.lnk now opens $($exe.FullName)"
