param([string]$Manifest, [switch]$CheckOnly, [switch]$Verify)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
if (-not $Manifest) {
    $current = Get-Content -LiteralPath (Join-Path $repo 'dist/current.json') -Raw | ConvertFrom-Json
    $Manifest = Join-Path $repo $current.manifest
    if ((Get-FileHash -LiteralPath $Manifest -Algorithm SHA256).Hash -ne $current.manifestSha256) {
        throw 'The verified package manifest changed.'
    }
}
$Manifest = (Get-Item -LiteralPath $Manifest).FullName
$package = Split-Path -Parent $Manifest
$identity = Get-Content -LiteralPath $Manifest -Raw | ConvertFrom-Json
if ($identity.schemaVersion -ne 1 -or $identity.sourceCommit -notmatch '^[0-9a-f]{40}$') { throw 'Invalid package identity.' }
function Package-File([string]$Relative) {
    $path = [IO.Path]::GetFullPath((Join-Path $package $Relative))
    if (-not $path.StartsWith($package + [IO.Path]::DirectorySeparatorChar, [StringComparison]::OrdinalIgnoreCase)) {
        throw 'Package path escapes its directory.'
    }
    return $path
}
foreach ($file in $identity.payload) {
    if ((Get-FileHash -LiteralPath (Package-File $file.path) -Algorithm SHA256).Hash -ne $file.sha256) {
        throw "Package payload changed: $($file.path)"
    }
}
$exe = Package-File $identity.executable
$default = Package-File $identity.fixture
if ((Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash -ne $identity.executableSha256 -or
    (Get-FileHash -LiteralPath $default -Algorithm SHA256).Hash -ne $identity.fixtureSha256) {
    throw 'The tested executable or default design changed.'
}
Write-Output "$($identity.version) | $($identity.sourceCommit) | $($identity.executableSha256)"
if ($CheckOnly) { return }
$userDir = Join-Path $repo 'profiles/play'
New-Item -ItemType Directory -Path $userDir -Force | Out-Null
# The runtime prefers the user's saved design. The shipped default is only a fallback.
$launchArgs = @('-windowed','-ResX=1600','-ResY=900','-NoVSync','-ExecCmds="r.VSync 0,t.MaxFPS 60"','-VibeAutoLoad',"-VibeDefault=`"$default`"", "-UserDir=`"$userDir`"")
if ($Verify) {
    $verifyDir = Join-Path $repo ('out/play-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fff'))
    if (Test-Path -LiteralPath $verifyDir) { throw 'Play verification needs a new directory.' }
    New-Item -ItemType Directory -Path $verifyDir | Out-Null
    $launchArgs += @("-VibeVerify=`"$verifyDir`"",'-VibeQuit','-VibeCycles=1')
}
$process = Start-Process -FilePath $exe -ArgumentList $launchArgs -WorkingDirectory $repo -WindowStyle Normal -PassThru
if ($Verify) {
    $process.WaitForExit()
    if ($process.ExitCode -ne 0) { throw "Play exited $($process.ExitCode)." }
    $events = @(Get-Content -LiteralPath (Join-Path $verifyDir 'events.jsonl') | ForEach-Object { $_ | ConvertFrom-Json })
    $ready = @($events | Where-Object event -eq 'gpu-ready')
    $requests = @($events | Where-Object event -eq 'request')
    if ($ready.Count -ne 1 -or $ready[0].rendered_frames -lt 3 -or $requests.Count -ne 1 -or -not $requests[0].load) {
        throw 'Play did not load a saved design through actual GPU readiness.'
    }
    if ($ready[0].viewport_width -ne 1600 -or $ready[0].viewport_height -ne 900) {
        throw 'Play viewport differs from the expected default window.'
    }
    [ordered]@{sourceCommit=$identity.sourceCommit;executableSha256=$identity.executableSha256;
        launcherSha256=(Get-FileHash -LiteralPath $PSCommandPath -Algorithm SHA256).Hash;
        resolution='1600x900';maxFps=60;vsync=$false;renderedFrames=$ready[0].rendered_frames;
        manifestSha256=(Get-FileHash -LiteralPath $Manifest -Algorithm SHA256).Hash;
        processId=$process.Id;exitCode=$process.ExitCode;gpuReadySeconds=$ready[0].seconds} |
        ConvertTo-Json | Set-Content -Encoding utf8 -LiteralPath (Join-Path $verifyDir 'identity.json')
    Write-Output "PASS exact Play verification: $verifyDir"
}
