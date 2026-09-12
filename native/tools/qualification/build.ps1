<# CI qualification build. Uses the installed Zig compiler; never downloads it.
   Run component contracts before integrations using the CMake test adapter.
#>
[CmdletBinding()]
param(
  [Parameter(Mandatory)][string]$Compiler,
  [Parameter(Mandatory)][string]$Output
)
$ErrorActionPreference = 'Stop'
$Root = (Resolve-Path (Join-Path $PSScriptRoot '../../..')).Path
$Native = Join-Path $Root 'native'
$Output = [IO.Path]::GetFullPath($Output)
if (Test-Path -LiteralPath $Output) { throw "Qualification output must be fresh: $Output" }
New-Item -ItemType Directory -Path $Output | Out-Null
$Compiler = (Resolve-Path -LiteralPath $Compiler).Path
$Version = & $Compiler version
if ($LASTEXITCODE -ne 0 -or $Version -ne '0.14.1') { throw 'Qualification requires Zig 0.14.1' }

# This builds the core archive once and links every portable test against it.
& (Join-Path $Native 'tools/build.ps1') -Compiler $Compiler -Configuration Release
if ($LASTEXITCODE -ne 0) { throw 'Portable build failed' }
$Build = Join-Path $Native 'build'
$Flags = @('-std=c++20', '-O2', '-ffp-contract=off', '-Wall', '-Wextra')
$Includes = @('-I', (Join-Path $Native 'core/include'), '-I', (Join-Path $Native 'core/src'))
$Drivers = @(
  @{ Source = 'matrix.cpp'; Binary = 'coaster_matrix.exe'; Defines = @() },
  @{ Source = 'fixture.cpp'; Binary = 'crossover_fixture.exe'; Defines = @('-DCLI_SOURCE="' + (Join-Path $Native 'core/src/main.cpp').Replace('\', '/') + '"') },
  @{ Source = 'organic.cpp'; Binary = 'replay_organic.exe'; Defines = @('-DORGANIC_TEST_SOURCE="' + (Join-Path $Native 'core/tests/organic_generation_tests.cpp').Replace('\', '/') + '"') }
)
foreach ($Driver in $Drivers) {
  & $Compiler c++ @Flags @Includes @($Driver.Defines) (Join-Path $PSScriptRoot $Driver.Source) (Join-Path $Build 'coaster_core.lib') -o (Join-Path $Output $Driver.Binary)
  if ($LASTEXITCODE -ne 0) { throw "Driver build failed: $($Driver.Source)" }
}
foreach ($Name in @('coaster_cli.exe', 'coaster_convergence.exe')) {
  Copy-Item -LiteralPath (Join-Path $Build $Name) -Destination (Join-Path $Output $Name)
}

# CMake registers the same 36 contracts against the Zig executables. It does
# not compile a second core or run tests during this configuration step.
$Python = (Get-Command python -CommandType Application).Source
$Adapter = Join-Path $Native 'build-qualification-tests'
& cmake -S $Native -B $Adapter "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=$Build" "-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=$Build" "-DPython3_EXECUTABLE=$Python"
if ($LASTEXITCODE -ne 0) { throw 'Portable CTest adapter configuration failed' }
$InventoryText = & ctest --test-dir $Adapter -C Release --show-only=json-v1
if ($LASTEXITCODE -ne 0) { throw 'Portable CTest inventory failed' }
$Inventory = ($InventoryText -join "`n") | ConvertFrom-Json
if ($Inventory.tests.Count -ne 36 -or 'cli_arguments' -notin $Inventory.tests.name) { throw 'Expected all 36 tests including real CLI contracts' }
$Testing = Join-Path $Adapter 'Testing'
New-Item -ItemType Directory -Path $Testing -Force | Out-Null
$InventoryText | Set-Content -LiteralPath (Join-Path $Testing 'inventory.json') -Encoding utf8
& ctest --test-dir $Adapter -C Release -L component --output-on-failure --stop-on-failure --parallel 2 --timeout 600 --output-junit (Join-Path $Testing 'components.xml')
if ($LASTEXITCODE -ne 0) { throw 'Portable component contracts failed' }
& ctest --test-dir $Adapter -C Release -L integration --output-on-failure --stop-on-failure --parallel 2 --timeout 600 --output-junit (Join-Path $Testing 'integrations.xml')
if ($LASTEXITCODE -ne 0) { throw 'Portable integration contracts failed' }

$Commit = & git -C $Root rev-parse HEAD
if ($LASTEXITCODE -ne 0) { throw 'Cannot identify checkout commit' }
$Source = [ordered]@{}
$Files = & git -C $Root ls-files -- native/core native/tools native/unreal/Source native/unreal/scripts native/unreal/VibeCoaster.uproject native/CMakeLists.txt .github/workflows/native.yml
if ($LASTEXITCODE -ne 0) { throw 'Cannot enumerate maintained build inputs' }
foreach ($File in ($Files | Sort-Object)) {
  $Source[$File] = (Get-FileHash -LiteralPath (Join-Path $Root $File) -Algorithm SHA256).Hash.ToLowerInvariant()
}
$Binaries = [ordered]@{}
foreach ($File in (Get-ChildItem -LiteralPath $Output -Filter '*.exe' | Sort-Object Name)) {
  $Binaries[$File.Name] = (Get-FileHash -LiteralPath $File.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
}
$Manifest = [ordered]@{
  commit = $Commit
  compiler = @{ version = $Version; sha256 = (Get-FileHash -LiteralPath $Compiler -Algorithm SHA256).Hash.ToLowerInvariant(); flags = $Flags }
  binaries = $Binaries
  source = $Source
}
$Manifest | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Output 'manifest.json') -Encoding utf8
$PortableBinaries = [ordered]@{}
foreach ($File in (Get-ChildItem -LiteralPath $Build -Filter '*.exe' | Sort-Object Name)) {
  $PortableBinaries[$File.Name] = (Get-FileHash -LiteralPath $File.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
}
@{ commit = $Commit; compiler = $Manifest.compiler; source = $Source; binaries = $PortableBinaries } |
  ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $Testing 'portable-identity.json') -Encoding utf8
