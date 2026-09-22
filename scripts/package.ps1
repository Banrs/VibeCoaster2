param(
    [string]$UnrealRoot = 'D:\Games\Epic Games\UE_5.8',
    [string]$Design
)
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$gitRoot = (& git -c "safe.directory=$repo" -C $repo rev-parse --show-toplevel).Trim()
if ($LASTEXITCODE -ne 0 -or [IO.Path]::GetFullPath($gitRoot) -ne $repo) { throw 'Packaging requires the independent VibeCoaster2 repository.' }
$commit = (& git -c "safe.directory=$repo" -C $repo rev-parse HEAD).Trim()
if ($commit -notmatch '^[0-9a-f]{40}$') { throw 'Missing source commit.' }
if (@(& git -c "safe.directory=$repo" -C $repo status --porcelain --untracked-files=all).Count) {
    throw 'Commit the reviewed source before packaging.'
}
$config = Get-Content -LiteralPath (Join-Path $repo 'unreal/Config/DefaultGame.ini') -Raw
$version = [regex]::Match($config, '(?m)^ProjectVersion=([a-zA-Z0-9.-]+)').Groups[1].Value
if (-not $version) { throw 'Missing project version.' }
if (-not $Design) { $Design = Join-Path $repo 'out/validation-fixture.vcd' }
$Design = (Get-Item -LiteralPath $Design).FullName
$project = Join-Path $repo 'unreal/VibeCoaster.uproject'
$uat = Join-Path $UnrealRoot 'Engine/Build/BatchFiles/RunUAT.bat'
$engine = Get-Content -LiteralPath (Join-Path $UnrealRoot 'Engine/Build/Build.version') -Raw | ConvertFrom-Json
if ($engine.MajorVersion -ne 5 -or $engine.MinorVersion -ne 8) { throw 'UE 5.8 is required.' }
$run = $commit.Substring(0,12) + '-' + [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss')
$archive = Join-Path $repo "dist/$version/$run"
$logDir = Join-Path $repo "out/package-$run"
if ((Test-Path -LiteralPath $archive) -or (Test-Path -LiteralPath $logDir)) { throw 'Package paths must be new.' }
New-Item -ItemType Directory -Path $archive,$logDir | Out-Null
$log = Join-Path $logDir 'build-cook-run.log'
Write-Output "Packaging $version at $commit; log: $log"
& $uat BuildCookRun "-project=$project" -nop4 -installed -nocompileeditor -platform=Win64 `
    -clientconfig=Development -build '-ubtargs=-MaxParallelActions=1 -NoUBA' -cook '-map=/Game/Maps/Ride' `
    '-AdditionalCookerOptions=-NoShaderWorker' -stage -pak -iostore -archive "-archivedirectory=$archive" `
    -utf8output -unattended *> $log
if ($LASTEXITCODE -ne 0) { Get-Content -LiteralPath $log -Tail 45; throw "Packaging failed ($LASTEXITCODE)." }
if ((& git -c "safe.directory=$repo" -C $repo rev-parse HEAD).Trim() -ne $commit -or
    @(& git -c "safe.directory=$repo" -C $repo status --porcelain --untracked-files=all).Count) {
    throw 'Source changed during packaging; this build cannot be promoted.'
}
$exeRelative = 'Windows/VibeCoaster/Binaries/Win64/VibeCoaster.exe'
$exe = Join-Path $archive $exeRelative
if (-not (Test-Path -LiteralPath $exe -PathType Leaf)) { throw 'Packaged executable missing.' }
$defaults = Join-Path $archive 'Defaults'
New-Item -ItemType Directory -Path $defaults | Out-Null
Copy-Item -LiteralPath $Design -Destination (Join-Path $defaults 'Default.vcd')
$payload = @(Get-ChildItem -LiteralPath (Join-Path $archive 'Windows') -Recurse -File | Where-Object {
    $_.Extension -in '.exe','.dll','.pak','.utoc','.ucas','.ini'
} | ForEach-Object {
    [ordered]@{ path=[IO.Path]::GetRelativePath($archive,$_.FullName).Replace('\','/'); sha256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
})
$manifest = [ordered]@{
    schemaVersion=1; version=$version; sourceCommit=$commit; configuration='Development'; compilerWorkers=1
    createdUtc=[DateTime]::UtcNow.ToString('o'); engine=$engine
    executable=$exeRelative; executableSha256=(Get-FileHash -LiteralPath $exe -Algorithm SHA256).Hash
    fixture='Defaults/Default.vcd'; fixtureSha256=(Get-FileHash -LiteralPath (Join-Path $defaults 'Default.vcd') -Algorithm SHA256).Hash
    payload=$payload; buildLog=[IO.Path]::GetRelativePath($repo,$log).Replace('\','/')
}
$manifestPath = Join-Path $archive 'package-manifest.json'
$manifest | ConvertTo-Json -Depth 6 | Set-Content -Encoding utf8 -LiteralPath $manifestPath
Write-Output "Package created, pending runtime verification: $manifestPath"
