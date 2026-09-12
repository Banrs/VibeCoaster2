[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$UnrealRoot,
    [ValidateSet('Development', 'Shipping')][string]$Configuration = 'Development',
    [string]$OutputDirectory,
    [switch]$PrepareOnly,
    [switch]$SkipAutomation
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Project = Join-Path $ProjectRoot 'VibeCoaster.uproject'
$EngineRoot = (Resolve-Path -LiteralPath $UnrealRoot).Path
$Build = Join-Path $EngineRoot 'Engine/Build/BatchFiles/Build.bat'
$UAT = Join-Path $EngineRoot 'Engine/Build/BatchFiles/RunUAT.bat'
$Editor = Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
foreach ($Required in @($Build, $UAT, $Editor)) {
    if (-not (Test-Path -LiteralPath $Required -PathType Leaf)) { throw "Unreal installation missing: $Required" }
}
$Version = Get-Content -LiteralPath (Join-Path $EngineRoot 'Engine/Build/Build.version') -Raw | ConvertFrom-Json
if ($Version.MajorVersion -ne 5 -or $Version.MinorVersion -lt 8) { throw 'This source uses UE 5.8 build settings and requires UE 5.8 or newer.' }
if (-not $OutputDirectory) { $OutputDirectory = Join-Path $ProjectRoot 'Packaged' }
$OutputDirectory = [System.IO.Path]::GetFullPath($OutputDirectory)
$RunId = [DateTime]::UtcNow.ToString('yyyyMMdd-HHmmss-fff')
$RunLogs = Join-Path $ProjectRoot "Saved/BuildRuns/$RunId"
if (Test-Path -LiteralPath $RunLogs) { throw 'Build log directory must be fresh.' }
New-Item -ItemType Directory -Path $RunLogs -Force | Out-Null
$TempDirectory = Join-Path $ProjectRoot 'Saved/Temp'
$CacheDirectory = Join-Path $ProjectRoot 'Saved/DerivedDataCache'
New-Item -ItemType Directory -Path $TempDirectory, $CacheDirectory -Force | Out-Null
$env:TEMP = $TempDirectory
$env:TMP = $TempDirectory
[Environment]::SetEnvironmentVariable('UE-LocalDataCachePath', $CacheDirectory, 'Process')
$ZenDirectory = Join-Path $CacheDirectory 'Zen'
[Environment]::SetEnvironmentVariable('UE-ZenDataPath', $ZenDirectory, 'Process')
[Environment]::SetEnvironmentVariable('UE-ZenSubprocessDataPath', $ZenDirectory, 'Process')
$env:uebp_LogFolder = Join-Path $RunLogs 'UAT'
$env:uebp_FinalLogFolder = $env:uebp_LogFolder
$env:DOTNET_CLI_HOME = Join-Path $ProjectRoot 'Saved/DotNet'
$env:NUGET_PACKAGES = Join-Path $ProjectRoot 'Saved/NuGetPackages'
New-Item -ItemType Directory -Path $ZenDirectory, $env:uebp_LogFolder, $env:DOTNET_CLI_HOME, $env:NUGET_PACKAGES -Force | Out-Null
$Receipt = Join-Path $RunLogs 'ContentBootstrap.json'
$env:VIBECOASTER_BOOTSTRAP_RECEIPT = $Receipt
Write-Host "Build evidence: $RunLogs"

function Invoke-HiddenEditor([string[]]$Arguments, [string]$LogName) {
    $Log = Join-Path $RunLogs $LogName
    $Parameters = @{
        FilePath = $Editor
        ArgumentList = $Arguments + @("-abslog=`"$Log.engine.log`"")
        WindowStyle = 'Hidden'
        Wait = $true
        PassThru = $true
        RedirectStandardOutput = "$Log.stdout.log"
        RedirectStandardError = "$Log.stderr.log"
    }
    $Process = Start-Process @Parameters
    if ($Process.ExitCode -ne 0) { throw "Unreal editor exited $($Process.ExitCode). See $Log.stdout.log and $Log.stderr.log" }
}

Write-Host 'Building the actual Unreal editor target...'
# Bound compiler memory while other applications are open.
& $Build 'VibeCoasterEditor' 'Win64' 'Development' $Project '-WaitMutex' '-NoHotReloadFromIDE' '-MaxParallelActions=2' "-Log=$RunLogs/UnrealBuildTool.log" 2>&1 | Tee-Object -FilePath (Join-Path $RunLogs 'EditorBuild.log')
if ($LASTEXITCODE -ne 0) { throw "Unreal editor build failed ($LASTEXITCODE)." }

Write-Host 'Creating the minimal cooked map and materials through Unreal editor APIs...'
$ContentScript = Join-Path $ProjectRoot 'scripts/create_content.py'
# Full editor startup ensures the level subsystem is ready.
Invoke-HiddenEditor -Arguments @("`"$Project`"", '/Engine/Maps/Entry', "-ExecutePythonScript=`"$ContentScript`"", '-unattended', '-nop4', '-NullRHI', '-nosplash', '-stdout', '-FullStdOutLogOutput') -LogName 'ContentBootstrap'
if (-not (Test-Path -LiteralPath $Receipt -PathType Leaf)) { throw 'Editor did not produce its content bootstrap receipt. Inspect the bootstrap log.' }
foreach ($Asset in @('Content/Maps/Ride.umap', 'Content/Materials/M_Rail.uasset', 'Content/Materials/M_Ground.uasset', 'Content/Materials/M_Ground_Plain.uasset', 'Content/Materials/M_Ground_Relief.uasset', 'Content/Materials/M_Structure.uasset', 'Content/Materials/M_Train.uasset', 'Content/Materials/M_Footing.uasset', 'Content/Art/V072/Conventional2/SM_TrainCar.uasset', 'Content/Art/V072/TrackWeb1/SM_TrackTieWeb.uasset', 'Content/Art/V072/Import1/SM_StationPlatformPanel.uasset', 'Content/Art/V072/Import1/SM_StationPlatformEndPanel.uasset', 'Content/Art/V072/Import1/SM_StationRoofPanel.uasset', 'Content/Art/V072/Import1/SM_StationPost.uasset')) {
    if (-not (Test-Path -LiteralPath (Join-Path $ProjectRoot $Asset))) { throw "Generated asset missing: $Asset" }
}
if (-not $SkipAutomation) {
    Write-Host 'Running Unreal coordinate, hardware, mesh and terrain contract automation...'
    Invoke-HiddenEditor -Arguments @("`"$Project`"", '-unattended', '-nop4', '-NullRHI', '-nosplash', '-stdout', '-FullStdOutLogOutput', '-ExecCmds="Automation RunTests VibeCoaster"', '-TestExit="Automation Test Queue Empty"', "-ReportExportPath=`"$RunLogs/Automation`"") -LogName 'Automation'
    $AutomationLog = Get-Content -LiteralPath (Join-Path $RunLogs 'Automation.stdout.log') -Raw
    if ($AutomationLog -match 'Result=\{Fail') {
        throw "Automation success was not confirmed. Inspect $RunLogs/Automation and $RunLogs/Automation.stdout.log."
    }
    foreach ($Contract in @('CoordinateContract', 'MeshContract', 'TerrainBackdropContract', 'CanyonRenderBudgetContract', 'StationArtContract', 'ImportedArtContract', 'SeedInputContract', 'OperationHardware')) {
        if ($AutomationLog -notmatch ('Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.' + $Contract + '\}')) {
            throw "Automation success missing for VibeCoaster.$Contract. Inspect $RunLogs/Automation.stdout.log."
        }
    }
}
if ($PrepareOnly) { Write-Host "Prepared Unreal project: $Project"; return }

$RunArchive = Join-Path $OutputDirectory ('run-' + $RunId)
if (Test-Path -LiteralPath $RunArchive) { throw 'Packaging archive must be fresh.' }
Write-Host 'Building, cooking, staging, and packaging the native Win64 game...'
& $UAT 'BuildCookRun' "-project=$Project" '-noP4' '-platform=Win64' "-clientconfig=$Configuration" '-build' '-ubtargs=-MaxParallelActions=2' '-cook' '-map=/Game/Maps/Ride' '-stage' '-pak' '-iostore' '-archive' "-archivedirectory=$RunArchive" '-prereqs' '-utf8output' 2>&1 | Tee-Object -FilePath (Join-Path $RunLogs 'Package.log')
if ($LASTEXITCODE -ne 0) { throw "Unreal packaging failed ($LASTEXITCODE)." }
$Executables = Get-ChildItem -LiteralPath $RunArchive -Filter 'VibeCoaster.exe' -File -Recurse
if (-not $Executables) { throw 'BuildCookRun returned success but no packaged VibeCoaster.exe was found.' }
$Executables | ForEach-Object { Write-Host "Packaged executable: $($_.FullName)" }
Write-Host 'Packaging does not verify rendering. Complete README manual acceptance checks on the target GPU.'
