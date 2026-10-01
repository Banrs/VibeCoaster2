[CmdletBinding()]
param([string]$UnrealRoot = 'D:/Games/Epic Games/UE_5.8')
$ErrorActionPreference = 'Stop'
$Repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../../..')).Path
$Project = Join-Path $Repository 'native/unreal/VibeCoaster.uproject'
$Editor = Join-Path $UnrealRoot 'Engine/Binaries/Win64/UnrealEditor.exe'
if (-not (Test-Path -LiteralPath $Editor -PathType Leaf)) { throw "Unreal editor missing: $Editor" }
if (-not (Test-Path -LiteralPath (Join-Path $Repository 'native/unreal/Binaries/Win64/UnrealEditor-VibeCoaster.dll'))) {
    throw 'Build VibeCoasterEditor Win64 Development before opening the geometry review.'
}
$Profile = Join-Path $Repository 'UserData-GeometryReview'
$Save = Join-Path $Profile 'Saved/VibeCoaster2/Designs/Accepted.vcdesign'
if (-not (Test-Path -LiteralPath $Save)) {
    $Selector = Get-Content -Raw -LiteralPath (Join-Path $Repository 'dist/current.json') | ConvertFrom-Json
    $Source = Join-Path $Repository ($Selector.profile + '/Saved/VibeCoaster2/Designs/Accepted.vcdesign')
    if ((Get-FileHash -LiteralPath $Source -Algorithm SHA256).Hash -ne $Selector.acceptedDesignSha256) { throw 'Baseline save hash does not match the playable selector.' }
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Save) | Out-Null
    Copy-Item -LiteralPath $Source -Destination $Save
}
# This is an explicit interactive review launch, isolated from the frozen Play profile.
Start-Process -FilePath $Editor -ArgumentList @("`"$Project`"", '-game', "-UserDir=`"$Profile`"", '-CoasterLoad', '-windowed', '-ResX=1600', '-ResY=900') -WorkingDirectory $Repository
