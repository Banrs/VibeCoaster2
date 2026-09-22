param([Parameter(Mandatory)][string]$Manifest,[ValidateRange(1,500)][int]$Processes=300,[ValidatePattern('^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,60}$')][string]$Series='packaged-tail')
$ErrorActionPreference='Stop'
$repo=[IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$package=Split-Path -Parent $Manifest
$identity=Get-Content -LiteralPath $Manifest -Raw | ConvertFrom-Json
$exe=Join-Path $package $identity.executable
$design=Join-Path $package $identity.fixture
$seriesOutput=Join-Path $repo "out/$Series-summary.jsonl"
if (Test-Path -LiteralPath $seriesOutput) { throw 'A tail series must have a new output name.' }
$manifestHash=(Get-FileHash -LiteralPath $Manifest -Algorithm SHA256).Hash
$batchTimer=[Diagnostics.Stopwatch]::StartNew()
foreach ($number in 1..$Processes) {
    $runName='{0}-{1:000}' -f $Series,$number
    & (Join-Path $repo 'scripts/verify-runtime.ps1') -Mode Loads -Cycles 2 -Packaged -Executable $exe -Design $design -RunName $runName -Width 2560 -Height 1440 -MaxFps 60 -DisableVSync -Benchmark
    $sample=Get-Content -LiteralPath (Join-Path $repo "out/$runName/summary.json") -Raw | ConvertFrom-Json
    if ($sample.samples.Count -ne 2 -or $sample.exitCode -ne 0) { throw 'Incomplete tail sample.' }
    [ordered]@{run=$runName;process=$number;startupSeconds=$sample.initialUiGpuSeconds;freshProcessLoadSeconds=$sample.samples[0].seconds;warmLoadSeconds=$sample.samples[1].seconds;processSeconds=$sample.processSeconds;elapsedSeconds=$batchTimer.Elapsed.TotalSeconds;manifestSha256=$manifestHash} | ConvertTo-Json -Compress | Add-Content -Encoding utf8 -LiteralPath $seriesOutput
    Write-Output "Tail progress: $number/$Processes processes, $($number*2) loads"
}
if ((Get-FileHash -LiteralPath $Manifest -Algorithm SHA256).Hash -ne $manifestHash) { throw 'Package identity changed during measurement.' }
Write-Output "PASS complete tail series: $seriesOutput"
