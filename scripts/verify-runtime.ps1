param(
    [ValidateSet('Loads','Generate','Ride','Flow')][string]$Mode = 'Loads',
    [ValidateRange(1,2000)][int]$Cycles = 3,
    [string]$Executable = 'D:\Games\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe',
    [string]$Design,
    [ValidatePattern('^[a-zA-Z0-9][a-zA-Z0-9_.-]{0,79}$')][string]$RunName,
    [ValidateRange(640,7680)][int]$Width = 1600,
    [ValidateRange(480,4320)][int]$Height = 900,
    [ValidateRange(0,240)][int]$MaxFps = 0,
    [switch]$DisableVSync,
    [switch]$Benchmark,
    [switch]$Packaged
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $RunName) { $RunName = $Mode.ToLowerInvariant() + '-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') }
if (-not $Design) { $Design = Join-Path $repo 'out/validation-fixture.vcd' }
$Design = (Get-Item -LiteralPath $Design).FullName
$Executable = (Get-Item -LiteralPath $Executable).FullName
$verifyOutput = Join-Path $repo ('out/' + $RunName)
$verifyProfile = Join-Path $repo ('profiles/' + $RunName)
if ((Test-Path -LiteralPath $verifyOutput) -or (Test-Path -LiteralPath $verifyProfile)) {
    throw 'Verification requires new output and profile directories.'
}
New-Item -ItemType Directory -Path $verifyOutput,$verifyProfile | Out-Null
$launchArgs = @()
if (-not $Packaged) { $launchArgs += (Join-Path $repo 'unreal/VibeCoaster.uproject'); $launchArgs += '-game' }
$launchArgs += @('-windowed',"-ResX=$Width","-ResY=$Height",'-ForceRes','-NoShaderWorker','-unattended',
    "-UserDir=$verifyProfile","-VibeVerify=$verifyOutput","-VibeLoad=$Design",'-VibeQuit',
    ('-abslog=' + (Join-Path $verifyOutput 'unreal.log')))
if ($Mode -eq 'Ride') { $launchArgs += '-VibeRideVerify' }
elseif ($Mode -eq 'Flow') { $launchArgs += '-VibeFlowVerify' }
else {
    $launchArgs += "-VibeCycles=$Cycles"
    if ($Mode -eq 'Generate') { $launchArgs += '-VibeGenerateVerify' }
}
if ($Benchmark) {
    if ($Mode -notin 'Loads','Generate') { throw 'Benchmark mode applies only to timing runs.' }
    $launchArgs += '-VibeBenchmark'
}
$consoleSettings = @()
if ($DisableVSync) { $launchArgs += '-NoVSync'; $consoleSettings += 'r.VSync 0' }
if ($MaxFps -gt 0) { $consoleSettings += "t.MaxFPS $MaxFps" }
if ($consoleSettings.Count) { $launchArgs += ('-ExecCmds=' + ($consoleSettings -join ',')) }
function Quote-ProcessArgument([string]$Value) {
    # Quote assignment values for Unreal's raw command-line parser as well as the CRT.
    if ($Value -match '^(-[^=]+)=(.*)$') {
        $prefix = $Matches[1]; $argumentValue = $Matches[2]
        return $prefix + '=' + (Quote-ProcessArgument $argumentValue)
    }
    if ($Value -notmatch '[\s"]') { return $Value }
    return '"' + (($Value -replace '(\\*)"', '$1$1\"') -replace '(\\+)$', '$1$1') + '"'
}
$commit = (& git -c "safe.directory=$repo" -C $repo rev-parse HEAD).Trim()
if ($LASTEXITCODE -ne 0) { throw 'Cannot record source identity.' }
$dirty = [bool](& git -c "safe.directory=$repo" -C $repo status --porcelain)
$identity = [ordered]@{
    sourceCommit=$commit; workingTreeChanges=$dirty; mode=$Mode; cycles=$Cycles
    executable=$Executable; executableSha256=(Get-FileHash -LiteralPath $Executable -Algorithm SHA256).Hash
    fixture=$Design; fixtureSha256=(Get-FileHash -LiteralPath $Design -Algorithm SHA256).Hash
    resolution="${Width}x${Height}"; benchmark=[bool]$Benchmark; maxFps=$MaxFps; disableVSync=[bool]$DisableVSync; cacheState='Fresh process/profile; OS file and driver caches are uncontrolled'
}
$modulePath = Join-Path $repo 'unreal/Binaries/Win64/UnrealEditor-VibeCoaster.dll'
if (-not $Packaged) { $identity['moduleSha256'] = (Get-FileHash -LiteralPath $modulePath -Algorithm SHA256).Hash }
if (-not $Packaged) {
    $identity['materialSha256'] = (Get-FileHash -LiteralPath (Join-Path $repo 'unreal/Content/Materials/VertexSurface.uasset') -Algorithm SHA256).Hash
}
if ($Packaged) {
    $packageDir = Split-Path -Parent $Executable
    $manifestPath = $null
    for ($level=0; $level -lt 7 -and $packageDir; $level++) {
        $candidate = Join-Path $packageDir 'package-manifest.json'
        if (Test-Path -LiteralPath $candidate) { $manifestPath=$candidate; break }
        $packageDir = Split-Path -Parent $packageDir
    }
    if (-not $manifestPath) { throw 'Packaged verification requires its build manifest.' }
    $manifest = Get-Content -LiteralPath $manifestPath -Raw | ConvertFrom-Json
    $declaredExe = [IO.Path]::GetFullPath((Join-Path $packageDir $manifest.executable))
    if ($declaredExe -ne $Executable -or $manifest.executableSha256 -ne $identity.executableSha256) {
        throw 'Executable does not match its packaged source identity.'
    }
    $identity['checkoutCommit'] = $commit
    $identity['sourceCommit'] = $manifest.sourceCommit
    $identity['version'] = $manifest.version
    $identity['packageManifestSha256'] = (Get-FileHash -LiteralPath $manifestPath -Algorithm SHA256).Hash
}
$identity | ConvertTo-Json | Set-Content -Encoding utf8 (Join-Path $verifyOutput 'identity.json')
$startTicks = [DateTime]::UtcNow.Ticks
$elapsed = [Diagnostics.Stopwatch]::StartNew()
$process = Start-Process -FilePath $Executable -ArgumentList ($launchArgs | ForEach-Object { Quote-ProcessArgument $_ }) `
    -WorkingDirectory $repo -WindowStyle Normal -PassThru
$process.Id | Set-Content (Join-Path $verifyOutput 'process-id.txt')
Write-Output "Verification PID $($process.Id): $verifyOutput"
$process.WaitForExit()
$elapsed.Stop()
$process.ExitCode | Set-Content (Join-Path $verifyOutput 'exit-code.txt')
$events = @(Get-Content -LiteralPath (Join-Path $verifyOutput 'events.jsonl') | ForEach-Object { $_ | ConvertFrom-Json })
$ui = $events | Where-Object event -eq 'ui-gpu-ready' | Select-Object -First 1
if (-not $ui -or -not $ui.utc_ticks) { throw 'Initial GPU readiness/startup timestamp is missing.' }
$samples = @()
$request = $null
foreach ($event in $events) {
    if ($event.event -eq 'request') { $request = $event }
    if ($event.event -eq 'gpu-ready') {
        if (-not $request -or $event.rendered_frames -lt 3) { throw 'Unpaired or unrendered readiness event.' }
        if ($event.viewport_width -ne $Width -or $event.viewport_height -ne $Height) {
            throw 'Actual viewport differs from the requested benchmark configuration.'
        }
        $observed = $event.wall - $request.wall
        if ([Math]::Abs($observed - $event.seconds) -gt .005) { throw 'Completion timing excludes work before its marker.' }
        $samples += [ordered]@{ load=$request.load; seconds=$event.seconds; observedSeconds=$observed; frames=$event.rendered_frames }
        $request = $null
    }
}
$summary = [ordered]@{
    exitCode=$process.ExitCode; processSeconds=$elapsed.Elapsed.TotalSeconds
    processStartUtcTicks="$startTicks"; initialUiGpuSeconds=([long]$ui.utc_ticks - $startTicks) / 10000000.0
    samples=$samples
}
$summary | ConvertTo-Json -Depth 5 | Set-Content -Encoding utf8 (Join-Path $verifyOutput 'summary.json')
if ($process.ExitCode -ne 0) { throw "Runtime failed with exit $($process.ExitCode)." }
if ($Mode -eq 'Loads' -and @($samples | Where-Object load).Count -ne $Cycles) { throw 'Not every requested load completed.' }
if ($Mode -eq 'Generate' -and @($samples | Where-Object { -not $_.load }).Count -ne $Cycles) {
    throw 'Not every requested generation completed.'
}
if ($Mode -eq 'Ride') {
    $passes = @($events | Where-Object event -eq 'ride-complete')
    if ($passes.Count -ne 2 -or @($events | Where-Object event -eq 'ride-capture').Count -ne 30) {
        throw 'Both complete traversals and all captures are required.'
    }
    foreach ($pass in $passes) {
        if ($pass.rendered_frames -lt 20 * $pass.ride_time -or $pass.wall_seconds -lt $pass.ride_time - .1) {
            throw 'Traversal lacks sufficient actual rendered frames or elapsed time.'
        }
    }
}
if ($Mode -eq 'Flow') {
    foreach ($required in @('flow-generate-pass','flow-save-pass','flow-roundtrip-pass','flow-rejection-pass','flow-pass')) {
        if (@($events | Where-Object event -eq $required).Count -ne 1) { throw "Missing $required." }
    }
    foreach ($stage in @('cpu','upload','gpu')) {
        if (@($events | Where-Object { $_.event -eq 'flow-cancel-pass' -and $_.stage -eq $stage }).Count -ne 1) {
            throw "Missing retained-ride cancellation at $stage."
        }
    }
}
Write-Output "PASS $Mode verification: $verifyOutput"