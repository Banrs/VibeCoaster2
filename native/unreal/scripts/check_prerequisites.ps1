[CmdletBinding()]
param([string]$UnrealRoot, [ValidateSet('5.6', '5.8')][string]$TargetVersion = '5.8', [switch]$AsJson)
# Read-only prerequisite inventory. This does not install software or accept terms.
$ErrorActionPreference = 'Stop'
$TargetEngine = [version]$TargetVersion
# Epic's versioned Windows setup tables; recommendations are not acceptance minima.
$Requirements = if ($TargetVersion -eq '5.8') {
    @{ VisualStudioRange = '[17.14,19.0)'; VisualStudio = 'Visual Studio 2022 17.14+ or Visual Studio 2026'; MinimumMsvc = '14.38.0'; RecommendedMsvc = '14.50'; MinimumWindowsSdk = '10.0.22621.0'; RecommendedWindowsSdk = '10.0.26100.0' }
} else {
    @{ VisualStudioRange = '[17.8,18.0)'; VisualStudio = 'Visual Studio 2022 17.8+ (17.14 recommended)'; MinimumMsvc = '14.38.33130'; RecommendedMsvc = '14.38.33130'; MinimumWindowsSdk = '10.0.19041.0'; RecommendedWindowsSdk = '10.0.22621.0' }
}
$Candidates = @()
if ($UnrealRoot) { $Candidates += $UnrealRoot }
foreach ($Registry in @('HKLM:\SOFTWARE\EpicGames\Unreal Engine', 'HKLM:\SOFTWARE\WOW6432Node\EpicGames\Unreal Engine')) {
    if (Test-Path -LiteralPath $Registry) {
        foreach ($Key in Get-ChildItem -LiteralPath $Registry) { $Candidates += (Get-ItemProperty -LiteralPath $Key.PSPath).InstalledDirectory }
    }
}
foreach ($Registry in @('HKCU:\SOFTWARE\Epic Games\Unreal Engine\Builds', 'HKCU:\SOFTWARE\EpicGames\Unreal Engine\Builds')) {
    if (Test-Path -LiteralPath $Registry) {
        foreach ($Property in (Get-ItemProperty -LiteralPath $Registry).PSObject.Properties) { if ($Property.Name -notmatch '^PS') { $Candidates += $Property.Value } }
    }
}
$LauncherFile = Join-Path $env:ProgramData 'Epic/UnrealEngineLauncher/LauncherInstalled.dat'
if (Test-Path -LiteralPath $LauncherFile) {
    foreach ($Install in (Get-Content -LiteralPath $LauncherFile -Raw | ConvertFrom-Json).InstallationList) {
        if ($Install.AppName -match '^UE_') { $Candidates += $Install.InstallLocation }
    }
}
foreach ($Parent in @('C:\Program Files\Epic Games', 'D:\Epic Games', 'D:\Games\Epic Games', 'D:\UnrealEngine', 'C:\UnrealEngine')) {
    if (Test-Path -LiteralPath $Parent) { $Candidates += $Parent; $Candidates += @(Get-ChildItem -LiteralPath $Parent -Directory | Select-Object -ExpandProperty FullName) }
}
$Engines = @()
foreach ($Candidate in @($Candidates | Where-Object { $_ -is [string] -and $_ } | Sort-Object -Unique)) {
    $VersionFile = Join-Path $Candidate 'Engine/Build/Build.version'
    if (-not (Test-Path -LiteralPath $VersionFile -PathType Leaf)) { continue }
    $Version = Get-Content -LiteralPath $VersionFile -Raw | ConvertFrom-Json
    $MissingFiles = @('Engine/Build/BatchFiles/Build.bat','Engine/Build/BatchFiles/RunUAT.bat','Engine/Binaries/Win64/UnrealEditor-Cmd.exe') | Where-Object { -not (Test-Path -LiteralPath (Join-Path $Candidate $_) -PathType Leaf) }
    $Engines += [pscustomobject]@{ Root = $Candidate; Version = "$($Version.MajorVersion).$($Version.MinorVersion).$($Version.PatchVersion)"; MatchesTarget = ($Version.MajorVersion -eq $TargetEngine.Major -and $Version.MinorVersion -eq $TargetEngine.Minor); MissingFiles = @($MissingFiles) }
}
$VsWhere = 'C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe'
$VisualStudio = @()
if (Test-Path -LiteralPath $VsWhere) {
    # Version-specific toolsets need not register the latest C++ tools component.
    # Inspect actual toolset files below rather than filtering those installations out.
    $Raw = & $VsWhere -all -products '*' -version $Requirements.VisualStudioRange -format json
    if ($LASTEXITCODE -ne 0) { throw 'vswhere could not inspect Visual Studio.' }
    if ($Raw) { $VisualStudio = @($Raw | ConvertFrom-Json | Where-Object { $_.isComplete } | Select-Object installationPath,installationVersion) }
}
$Compilers = @()
foreach ($Vs in $VisualStudio) {
    $ToolsetRoot = Join-Path $Vs.installationPath 'VC/Tools/MSVC'
    if (-not (Test-Path -LiteralPath $ToolsetRoot -PathType Container)) { continue }
    foreach ($Toolset in Get-ChildItem -LiteralPath $ToolsetRoot -Directory) {
        $Parsed = $null
        if (-not [version]::TryParse($Toolset.Name, [ref]$Parsed)) { continue }
        $MissingFiles = @('bin/Hostx64/x64/cl.exe', 'bin/Hostx64/x64/link.exe', 'include/vector', 'lib/x64/libcmt.lib') | Where-Object { -not (Test-Path -LiteralPath (Join-Path $Toolset.FullName $_) -PathType Leaf) }
        $Compilers += [pscustomobject]@{ Root = $Toolset.FullName; Version = $Toolset.Name; Compiler = (Join-Path $Toolset.FullName 'bin/Hostx64/x64/cl.exe'); MeetsDocumentedMinimum = ($Parsed -ge [version]$Requirements.MinimumMsvc); MissingFiles = @($MissingFiles) }
    }
}
$Sdks = @()
foreach ($Registry in @('HKLM:\SOFTWARE\Microsoft\Windows Kits\Installed Roots','HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows Kits\Installed Roots')) {
    if (-not (Test-Path -LiteralPath $Registry)) { continue }
    $SdkRoot = (Get-ItemProperty -LiteralPath $Registry).KitsRoot10
    if (-not $SdkRoot -or -not (Test-Path -LiteralPath (Join-Path $SdkRoot 'Include'))) { continue }
    foreach ($Sdk in Get-ChildItem -LiteralPath (Join-Path $SdkRoot 'Include') -Directory) {
        $Parsed = $null
        if (-not [version]::TryParse($Sdk.Name, [ref]$Parsed) -or $Parsed -lt [version]$Requirements.MinimumWindowsSdk) { continue }
        $MissingFiles = @("Include/$($Sdk.Name)/um/Windows.h", "Include/$($Sdk.Name)/ucrt/stdio.h", "Lib/$($Sdk.Name)/um/x64/kernel32.lib", "Lib/$($Sdk.Name)/ucrt/x64/ucrt.lib", "bin/$($Sdk.Name)/x64/rc.exe") | Where-Object { -not (Test-Path -LiteralPath (Join-Path $SdkRoot $_) -PathType Leaf) }
        if (-not @($MissingFiles).Count) { $Sdks += [pscustomobject]@{ Root = $SdkRoot; Version = $Sdk.Name } }
    }
}
$Missing = @()
$Sdks = @($Sdks | Sort-Object Root, Version -Unique)
if (-not @($Engines | Where-Object { $_.MatchesTarget -and $_.MissingFiles.Count -eq 0 }).Count) { $Missing += "UE $TargetVersion installed engine with Build.version, Build.bat, RunUAT.bat, and UnrealEditor-Cmd.exe. Finish installation or pass -UnrealRoot for an existing authorized installation." }
if (-not $VisualStudio.Count) { $Missing += "$($Requirements.VisualStudio), with C++ development components or equivalent C++ Build Tools." }
if (-not @($Compilers | Where-Object { $_.MeetsDocumentedMinimum -and $_.MissingFiles.Count -eq 0 }).Count) { $Missing += "MSVC $($Requirements.MinimumMsvc)+ x64 toolset with compiler, linker, headers and libraries; $($Requirements.RecommendedMsvc) is recommended for UE $TargetVersion." }
if (-not $Sdks.Count) { $Missing += "Windows SDK $($Requirements.MinimumWindowsSdk)+ with headers, UCRT, x64 libraries and rc.exe; $($Requirements.RecommendedWindowsSdk)+ is recommended for UE $TargetVersion." }
$Report = [pscustomobject]@{ CheckedAtUtc = [DateTime]::UtcNow.ToString('o'); Target = "UE $TargetVersion Win64"; Requirements = $Requirements; ReadyForDocumentedBuildPrerequisites = ($Missing.Count -eq 0); Engines = $Engines; VisualStudio = $VisualStudio; MsvcToolsets = $Compilers; WindowsSdks = $Sdks; Missing = $Missing; Limits = 'Read-only bounded discovery of documented C++ prerequisites. File presence does not verify a complete engine download, bundled .NET dependencies, UnrealBuildTool toolchain exclusions, compilation or rendering. Finish engine installation before building. No install or license action is performed.' }
if ($AsJson) { $Report | ConvertTo-Json -Depth 6 } else { $Report }
