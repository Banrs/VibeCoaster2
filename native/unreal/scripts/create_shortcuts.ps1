[CmdletBinding()]
param([switch]$Desktop)
$ErrorActionPreference = 'Stop'
$RepositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '../../..')).Path
$Current = Get-Content -LiteralPath (Join-Path $RepositoryRoot 'dist/current.json') -Raw | ConvertFrom-Json
$ManifestPath = Join-Path $RepositoryRoot $Current.manifest
if ((Get-FileHash -LiteralPath $ManifestPath -Algorithm SHA256).Hash -ne $Current.manifestSha256) { throw 'The current package manifest hash does not match.' }
$Manifest = Get-Content -LiteralPath $ManifestPath -Raw | ConvertFrom-Json
$PackageRoot = Join-Path (Split-Path -Parent $ManifestPath) 'Windows'
$Target = Join-Path $PackageRoot 'VibeCoaster/Binaries/Win64/VibeCoaster.exe'
$Record = @($Manifest.Executables | Where-Object { [IO.Path]::GetFullPath($_.Path) -eq [IO.Path]::GetFullPath($Target) })
if ($Record.Count -ne 1 -or (Get-FileHash -LiteralPath $Target -Algorithm SHA256).Hash -ne $Record[0].Sha256) { throw 'The packaged game executable is missing or does not match its manifest.' }
$Profile = (Resolve-Path -LiteralPath (Join-Path $RepositoryRoot $Current.profile)).Path
if (-not (Test-Path -LiteralPath (Join-Path $Profile 'Saved/VibeCoaster2/Designs/Accepted.vcdesign') -PathType Leaf)) { throw 'The current playable profile has no accepted ride.' }
$Destinations = @((Join-Path $RepositoryRoot 'Play VibeCoaster2.lnk'))
if ($Desktop) { $Destinations += Join-Path ([Environment]::GetFolderPath('Desktop')) 'VibeCoaster2.lnk' }
$Shell = New-Object -ComObject WScript.Shell
foreach ($Destination in $Destinations) {
    $Link = $Shell.CreateShortcut($Destination)
    $Link.TargetPath = $Target
    $Link.Arguments = '-UserDir="' + $Profile + '"'
    $Link.WorkingDirectory = $PackageRoot
    $Link.IconLocation = $Target + ',0'
    $Link.Description = 'Play VibeCoaster2 - ' + $Manifest.Release
    $Link.WindowStyle = 1
    $Link.Save()
    $Verified = $Shell.CreateShortcut($Destination)
    if ($Verified.TargetPath -ne $Target -or $Verified.Arguments -ne $Link.Arguments) { throw 'The launch shortcut did not retain its target and profile.' }
    [pscustomobject]@{ Shortcut = $Destination; Target = $Verified.TargetPath; Arguments = $Verified.Arguments; Release = $Manifest.Release }
}
