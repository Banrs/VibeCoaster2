[CmdletBinding()]
param([string]$UnrealRoot = 'D:/Games/Epic Games/UE_5.8', [switch]$Baseline, [UInt64]$Seed = 42)
$ErrorActionPreference = 'Stop'
$Repository = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../../..')).Path
$Project = Join-Path $Repository 'native/unreal/VibeCoaster.uproject'
$Editor = Join-Path $UnrealRoot 'Engine/Binaries/Win64/UnrealEditor.exe'
if (-not (Test-Path -LiteralPath $Editor -PathType Leaf)) { throw "Unreal editor missing: $Editor" }
if (-not (Test-Path -LiteralPath (Join-Path $Repository 'native/unreal/Binaries/Win64/UnrealEditor-VibeCoaster.dll'))) {
    throw 'Build VibeCoasterEditor Win64 Development before opening the geometry review.'
}
if ($Baseline) {
    $Profile = Join-Path $Repository 'UserData-GeometryReview'
    $Save = Join-Path $Profile 'Saved/VibeCoaster2/Designs/Accepted.vcdesign'
    if (-not (Test-Path -LiteralPath $Save)) {
        $Selector = Get-Content -Raw -LiteralPath (Join-Path $Repository 'dist/current.json') | ConvertFrom-Json
        $Source = Join-Path $Repository ($Selector.profile + '/Saved/VibeCoaster2/Designs/Accepted.vcdesign')
        if ((Get-FileHash -LiteralPath $Source -Algorithm SHA256).Hash -ne $Selector.acceptedDesignSha256) { throw 'Baseline save hash does not match the playable selector.' }
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Save) | Out-Null
        Copy-Item -LiteralPath $Source -Destination $Save
    }
    $Startup = @('-CoasterLoad')
} else {
    $Profile = Join-Path $Repository 'UserData-GeneratorDevelopment'
    New-Item -ItemType Directory -Force -Path $Profile | Out-Null
    $Startup = @('-CoasterGenerate', "-CoasterSeed=$Seed")
}
# Fresh generation is isolated from the frozen Play and historical review profiles.
$Arguments = @("`"$Project`"", '-game', "-UserDir=`"$Profile`"", '-windowed', '-ResX=1600', '-ResY=900') + $Startup
Start-Process -FilePath $Editor -ArgumentList $Arguments -WorkingDirectory $Repository
