param([switch]$Test)
$ErrorActionPreference='Stop'
$repo=Split-Path $PSScriptRoot -Parent
if ($env:OS -eq 'Windows_NT') {
    New-Item -ItemType Directory -Path "$repo/build" -Force | Out-Null
    & "$PSScriptRoot/build-native.cmd"
    if ($LASTEXITCODE -ne 0) { throw 'Native build or tests failed' }
    if ($Test) {
        & "$repo/build/generator_tests.exe" --extended
        if ($LASTEXITCODE -ne 0) { throw 'Generator regression failed' }
        & "$repo/build/persistence_tests.exe"
        if ($LASTEXITCODE -ne 0) { throw 'Persistence regression failed' }
    }
} else {
    cmake -S $repo -B "$repo/build" -DCMAKE_BUILD_TYPE=Release
    if ($LASTEXITCODE -ne 0) { throw 'Configure failed' }
    cmake --build "$repo/build" --parallel 1
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    if ($Test) {
        ctest --test-dir "$repo/build" --output-on-failure --parallel 1
        if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
    }
}
