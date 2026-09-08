<# Build native C++ coaster core with portable Zig (no auto download).
Usage:
  powershell -ExecutionPolicy Bypass -File native/tools/build.ps1
  powershell -ExecutionPolicy Bypass -File native/tools/build.ps1 -Compiler "C:\path\to\zig.exe"
  powershell -ExecutionPolicy Bypass -File native/tools/build.ps1 -Test
  powershell -ExecutionPolicy Bypass -File native/tools/build.ps1 -Clean -Test
#>
[CmdletBinding()]
param(
  [string]$Compiler = "",
  [ValidateSet("Release", "Debug")]
  [string]$Configuration = "Release",
  [switch]$Test,
  [switch]$Clean
)

$ErrorActionPreference = "Stop"

$ToolsDir = $PSScriptRoot
$NativeDir = Split-Path -Parent $ToolsDir
$IncludeDir = Join-Path $NativeDir "core\include"
$SrcDir = Join-Path $NativeDir "core\src"
$TestsDir = Join-Path $NativeDir "core\tests"
$BuildDir = Join-Path $NativeDir "build"

if ([string]::IsNullOrWhiteSpace($Compiler)) {
  $Compiler = Join-Path $NativeDir ".tools\zig-x86_64-windows-0.14.1\zig.exe"
}

if (-not (Test-Path -LiteralPath $Compiler)) {
  Write-Error "Missing compiler: '$Compiler' not found. This script never downloads toolchains automatically. Place the portable Zig 0.14.1 there or pass -Compiler <path-to-zig.exe>."
  exit 1
}

if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
  # Boundary safeguard: only delete the resolved native/build directory.
  $ResolvedNative = [System.IO.Path]::GetFullPath($NativeDir).TrimEnd([System.IO.Path]::DirectorySeparatorChar)
  $ResolvedBuild = [System.IO.Path]::GetFullPath($BuildDir).TrimEnd([System.IO.Path]::DirectorySeparatorChar)
  $ExpectedBuild = Join-Path $ResolvedNative "build"
  if ($ResolvedBuild -ne $ExpectedBuild) {
    Write-Error "Refusing -Clean: resolved build dir '$ResolvedBuild' is not '$ExpectedBuild'."
    exit 1
  }
  $Item = Get-Item -LiteralPath $BuildDir -Force
  if ($Item.Attributes -band [System.IO.FileAttributes]::ReparsePoint) {
    # Reparse point (symlink/junction): remove the link only, never recurse.
    Write-Host "Build path is a reparse point; removing link only."
    Remove-Item -LiteralPath $BuildDir -Force
  } else {
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
  }
}
New-Item -ItemType Directory -Path $BuildDir -Force | Out-Null

$CoreSources = @(
  (Join-Path $SrcDir "track.cpp"),
  (Join-Path $SrcDir "frame.cpp"),
  (Join-Path $SrcDir "drive_profile.cpp"),
  (Join-Path $SrcDir "convergence.cpp"),
  (Join-Path $SrcDir "simulation.cpp"),
  (Join-Path $SrcDir "generation.cpp"),
  (Join-Path $SrcDir "layout_modules.cpp"),
  (Join-Path $SrcDir "fvd.cpp"),
  (Join-Path $SrcDir "persistence.cpp"),
  (Join-Path $SrcDir "reference.cpp"),
  (Join-Path $SrcDir "supports.cpp"),
  (Join-Path $SrcDir "clearance.cpp"),
  (Join-Path $SrcDir "dimensions.cpp"),
  (Join-Path $SrcDir "station.cpp"),
  (Join-Path $SrcDir "structures.cpp")
)
$MainSource = Join-Path $SrcDir "main.cpp"
$TestSource = Join-Path $TestsDir "tests.cpp"

$Missing = @()
foreach ($f in ($CoreSources + @($MainSource))) {
  if (-not (Test-Path -LiteralPath $f)) { $Missing += $f }
}
if ($Missing.Count -gt 0) {
  Write-Error ("Missing core sources (owned by core agent, still in progress?):`n  " + ($Missing -join "`n  "))
  exit 1
}

if ($Configuration -eq "Debug") {
  $OptFlags = @("-O0", "-g")
} else {
  # Pinned release flags: deterministic FP, no contracted FMA.
  $OptFlags = @("-O2")
}

# Pinned warning + FP flags shared by both binaries.
$CommonFlags = @("-std=c++20", "-ffp-contract=off", "-Wall", "-Wextra") + $OptFlags

$CliOut = Join-Path $BuildDir "coaster_cli.exe"
$TestsOut = Join-Path $BuildDir "coaster_tests.exe"

function Invoke-ZigCxx {
  param([string[]]$ZigArgs, [string]$Label)
  & $Compiler @ZigArgs
  if ($LASTEXITCODE -ne 0) {
    Write-Error "$Label failed (zig c++ exit=$LASTEXITCODE). See output above."
    exit 1
  }
}

Write-Host "Compiler : $Compiler"
Write-Host "Config   : $Configuration"
Write-Host "BuildDir : $BuildDir"

Invoke-ZigCxx -Label "coaster_cli build" -ZigArgs (@("c++") + $CommonFlags + @("-I", $IncludeDir) + $CoreSources + @($MainSource, "-o", $CliOut))
Write-Host "Built $CliOut"

$NeedTests = $Test -or (Test-Path -LiteralPath $TestSource)
if (Test-Path -LiteralPath $TestSource) {
  Invoke-ZigCxx -Label "coaster_tests build" -ZigArgs (@("c++") + $CommonFlags + @("-I", $IncludeDir) + $CoreSources + @($TestSource, "-o", $TestsOut))
  Write-Host "Built $TestsOut"
} elseif ($Test) {
  Write-Error "Requested -Test but '$TestSource' is missing (test executable still in progress by core agent)."
  exit 1
} else {
  Write-Host "Skipped tests (no $TestSource yet)."
}

if ($Test) {
  if (-not (Test-Path -LiteralPath $TestsOut)) {
    Write-Error "Test binary missing: $TestsOut"
    exit 1
  }
  & $TestsOut
  if ($LASTEXITCODE -ne 0) {
    Write-Error "coaster_tests failed (exit=$LASTEXITCODE)."
    exit 1
  }
  Write-Host "coaster_tests passed."
}

# Independent experiment suites preserve the scientific/property tests from each input.
$AdapterInclude = Join-Path $NativeDir "unreal/Source/VibeCoaster/Public"
$Historical = Join-Path $TestsDir "fixtures/historical"
foreach ($Name in @("persistence_cancel", "baseline_jets", "flow_bridge", "connector_profile", "reference", "support", "support_family", "layout_modules", "organic_generation", "terrain_profile", "fvd", "dimensions", "clearance", "track_web", "station", "structures", "geometry", "frame_force", "terrain", "convergence", "drive_profile")) {
  $Source = Join-Path $TestsDir ($Name + "_tests.cpp")
  $Binary = Join-Path $BuildDir ($Name + "_tests.exe")
  $SuiteSources = $CoreSources
  Invoke-ZigCxx -Label ($Name + " tests build") -ZigArgs (@("c++") + $CommonFlags + @("-I", $IncludeDir, "-I", $AdapterInclude) + $SuiteSources + @($Source, "-o", $Binary))
  if ($Test) {
    $TestArgs = @()
    if ($Name -eq "support") { $TestArgs = @($Historical) }
    & $Binary @TestArgs
    if ($LASTEXITCODE -ne 0) { throw "$Name tests failed (exit=$LASTEXITCODE)" }
  }
}

# Explicit acceptance evidence tool; deliberately not run by -Test.
$ConvergenceSource = Join-Path $ToolsDir "convergence/audit.cpp"
$ConvergenceOut = Join-Path $BuildDir "coaster_convergence.exe"
Invoke-ZigCxx -Label "coaster_convergence build" -ZigArgs (@("c++") + $CommonFlags + @("-I", $IncludeDir) + $CoreSources + @($ConvergenceSource, "-o", $ConvergenceOut))
