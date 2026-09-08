[CmdletBinding()]
param([string]$ZigPath)
$ErrorActionPreference = 'Stop'
$Root = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$NativeRoot = (Resolve-Path -LiteralPath (Join-Path $Root '..')).Path
if (-not $ZigPath) { $ZigPath = Join-Path $NativeRoot '.tools/zig-x86_64-windows-0.14.1/zig.exe' }
$ZigPath = (Resolve-Path -LiteralPath $ZigPath).Path
$Project = Get-Content -LiteralPath (Join-Path $Root 'VibeCoaster.uproject') -Raw | ConvertFrom-Json
if ($Project.Modules[0].Name -ne 'VibeCoaster') { throw 'Project module does not match targets.' }
foreach ($Script in (Get-ChildItem -LiteralPath $PSScriptRoot -Filter '*.ps1')) {
    $Tokens = $null
    $Errors = $null
    [System.Management.Automation.Language.Parser]::ParseFile($Script.FullName, [ref]$Tokens, [ref]$Errors) | Out-Null
    if ($Errors) { throw ($Errors | Out-String) }
}
$CoreFiles = @('track', 'frame', 'drive_profile', 'convergence', 'simulation', 'generation', 'layout_modules', 'fvd', 'persistence', 'supports', 'reference', 'clearance', 'dimensions', 'station', 'structures')
foreach ($CoreFile in $CoreFiles) {
    $WrapperName = if ($CoreFile -eq 'drive_profile') { 'CoreDriveProfile.cpp' } elseif ($CoreFile -eq 'layout_modules') { 'CoreLayoutModules.cpp' } else { 'Core' + [char]::ToUpperInvariant($CoreFile[0]) + $CoreFile.Substring(1) + '.cpp' }
    $Wrapper = Join-Path $Root "Source/VibeCoaster/Private/$WrapperName"
    if (-not (Test-Path -LiteralPath $Wrapper)) { throw "Missing core source wrapper: $CoreFile" }
    if (-not (Test-Path -LiteralPath (Join-Path $NativeRoot "core/src/$CoreFile.cpp"))) { throw "Missing canonical core source: $CoreFile" }
    $Include = '#include "' + $CoreFile + '.cpp"'
    if (-not (Get-Content -LiteralPath $Wrapper -Raw).Contains($Include)) { throw "Wrapper does not include the canonical source: $CoreFile" }
}
$Output = Join-Path $Root 'Saved/Validation'
New-Item -ItemType Directory -Path $Output -Force | Out-Null
$Executable = Join-Path $Output 'coordinate-test.exe'
$IncludeCore = Join-Path $NativeRoot 'core/include'
$IncludeAdapter = Join-Path $Root 'Source/VibeCoaster/Public'
& $ZigPath 'c++' '-std=c++20' '-O2' "-I$IncludeCore" "-I$IncludeAdapter" (Join-Path $PSScriptRoot 'coordinate_test.cpp') '-o' $Executable
if ($LASTEXITCODE -ne 0) { throw 'Portable coordinate contract compilation failed.' }
& $Executable
if ($LASTEXITCODE -ne 0) { throw 'Portable coordinate contract test failed.' }
# These source-only wrappers do not depend on Unreal headers. Compile them as
# portable translation units; this does not pretend to compile the UE module.
foreach ($Name in @('Frame', 'Convergence', 'DriveProfile', 'LayoutModules')) {
    $Wrapper = Join-Path $Root "Source/VibeCoaster/Private/Core$Name.cpp"
    $Object = Join-Path $Output "Core$Name.obj"
    & $ZigPath 'c++' '-std=c++20' '-O2' '-ffp-contract=off' "-I$IncludeCore" ("-I" + (Join-Path $NativeRoot 'core/src')) '-c' $Wrapper '-o' $Object
    if ($LASTEXITCODE -ne 0) { throw "Portable canonical wrapper compilation failed: $Name" }
}
Write-Host 'PASS: project JSON, PowerShell parsing, exact core wrappers, compiled coordinates and portable frame/convergence/drive-profile translation units.' 
Write-Host 'This is not an Unreal build or a rendering test. Unreal installation is still required.'

