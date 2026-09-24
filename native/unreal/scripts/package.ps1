[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$UnrealRoot,
    [ValidateSet('Development', 'Shipping')][string]$Configuration = 'Development',
    [string]$OutputDirectory,
    [ValidateRange(1, 1)][int]$MaxParallelActions = 1,
    [switch]$PrepareOnly,
    [switch]$SkipAutomation,
    [switch]$SkipEditorPreparation
)
$ErrorActionPreference = 'Stop'
$ProjectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$Project = Join-Path $ProjectRoot 'VibeCoaster.uproject'
$VersionHeader = Join-Path $ProjectRoot '../core/include/coaster/version.hpp'
$ReleaseMatch = [regex]::Match((Get-Content -LiteralPath $VersionHeader -Raw), '#define COASTER_GENERATOR_VERSION "([^"]+)"')
if (-not $ReleaseMatch.Success) { throw 'Canonical release version is missing.' }
$ReleaseVersion = $ReleaseMatch.Groups[1].Value
$GameConfig = Join-Path $ProjectRoot 'Config/DefaultGame.ini'
$ConfigText = Get-Content -LiteralPath $GameConfig -Raw
if ($ConfigText -notmatch '(?m)^ProjectVersion=') { throw "ProjectVersion is missing from $GameConfig." }
$UpdatedConfig = [regex]::Replace($ConfigText, '(?m)^ProjectVersion=[^\r\n]*', ('ProjectVersion=' + $ReleaseVersion))
if ($UpdatedConfig -ne $ConfigText) { throw "ProjectVersion must equal canonical release $ReleaseVersion before packaging. Commit the aligned config first." }
$RepositoryRoot = (Resolve-Path -LiteralPath (Join-Path $ProjectRoot '../..')).Path
$SourceCommit = & git -c "safe.directory=$RepositoryRoot" -C $RepositoryRoot rev-parse HEAD
if ($LASTEXITCODE -ne 0 -or $SourceCommit -notmatch '^[0-9a-f]{40}$') { throw 'Packaging needs an identified source commit.' }
$SourceStatus = @(& git -c "safe.directory=$RepositoryRoot" -C $RepositoryRoot status --porcelain --untracked-files=all)
if ($LASTEXITCODE -ne 0) { throw "Could not inspect source status." }
if (-not $PrepareOnly -and $SourceStatus.Count -ne 0) { throw 'Commit the reviewed source before producing a versioned package.' }
$EngineRoot = (Resolve-Path -LiteralPath $UnrealRoot).Path
$Build = Join-Path $EngineRoot 'Engine/Build/BatchFiles/Build.bat'
$UAT = Join-Path $EngineRoot 'Engine/Build/BatchFiles/RunUAT.bat'
$Editor = Join-Path $EngineRoot 'Engine/Binaries/Win64/UnrealEditor-Cmd.exe'
if ($PrepareOnly -and $SkipEditorPreparation) { throw '-PrepareOnly cannot be combined with -SkipEditorPreparation.' }
$RequiredTools = @($UAT)
if (-not $SkipEditorPreparation) { $RequiredTools += @($Build, $Editor) }
foreach ($Required in $RequiredTools) {
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

if (-not $SkipEditorPreparation) {
    Write-Host 'Building the actual Unreal editor target...'
    # Bound compiler memory while other applications are open.
    & $Build 'VibeCoasterEditor' 'Win64' 'Development' $Project '-WaitMutex' '-NoHotReloadFromIDE' "-MaxParallelActions=$MaxParallelActions" "-Log=$RunLogs/UnrealBuildTool.log" 2>&1 | Tee-Object -FilePath (Join-Path $RunLogs 'EditorBuild.log')
    if ($LASTEXITCODE -ne 0) { throw "Unreal editor build failed ($LASTEXITCODE)." }

    Write-Host 'Creating the minimal cooked map and materials through Unreal editor APIs...'
    $ContentScript = Join-Path $ProjectRoot 'scripts/create_content.py'
    # Full editor startup with the real RHI also primes preview material shaders.
    Invoke-HiddenEditor -Arguments @("`"$Project`"", '/Engine/Maps/Entry', "-ExecutePythonScript=`"$ContentScript`"", '-unattended', '-nop4', '-nosplash', '-stdout', '-FullStdOutLogOutput') -LogName 'ContentBootstrap'
    if (-not (Test-Path -LiteralPath $Receipt -PathType Leaf)) { throw 'Editor did not produce its content bootstrap receipt. Inspect the bootstrap log.' }
    Write-Host 'Importing the reviewed runtime art manifest through Unreal editor APIs...'
    $ArtReceipt = Join-Path $ProjectRoot 'Saved/V3ArtImport.json'
    if (Test-Path -LiteralPath $ArtReceipt) { Remove-Item -LiteralPath $ArtReceipt -Force }
    $ArtScript = Join-Path $ProjectRoot 'scripts/import_v2_art.py'
    Invoke-HiddenEditor -Arguments @("`"$Project`"", '/Engine/Maps/Entry', "-ExecutePythonScript=`"$ArtScript`"", '-unattended', '-nop4', '-nosplash', '-stdout', '-FullStdOutLogOutput') -LogName 'ArtImport'
    if (-not (Test-Path -LiteralPath $ArtReceipt -PathType Leaf)) { throw 'Editor did not produce its art import receipt. Inspect the import log.' }
}
$ArtManifestPath = Join-Path $RepositoryRoot 'native/art/exports/manifest.json'
$ArtManifest = Get-Content -LiteralPath $ArtManifestPath -Raw | ConvertFrom-Json
$RuntimeArt = @($ArtManifest.assets | Where-Object { $_.runtime -ne $false })
if ($RuntimeArt.Count -eq 0) { throw 'Art manifest has no runtime assets.' }
$RequiredAssets = @('Content/Maps/Ride.umap', 'Content/Materials/M_Rail.uasset',
    'Content/Materials/M_LSM.uasset', 'Content/Materials/M_Brake.uasset',
    'Content/Materials/M_Ground_Highlands.uasset', 'Content/Materials/M_Structure.uasset',
    'Content/Materials/M_Train.uasset', 'Content/Materials/M_Footing.uasset')
$RequiredAssets += @($RuntimeArt | ForEach-Object {
    if ($_.name -notmatch '^SM_[A-Za-z0-9_]+$') { throw "Invalid runtime art identity in manifest: $($_.name)" }
    "Content/Art/V3/$($_.name).uasset"
})
$RequiredAssets += @($RuntimeArt | ForEach-Object { $_.materials } | Sort-Object -Unique | ForEach-Object {
    if ($_ -notmatch '^VC2_[A-Za-z0-9_]+$') { throw "Invalid runtime material identity in manifest: $_" }
    "Content/Art/V3/Materials/M_$($_).uasset"
})
foreach ($Asset in $RequiredAssets) {
    $AssetPath = Join-Path $ProjectRoot $Asset
    if (-not (Test-Path -LiteralPath $AssetPath -PathType Leaf) -or (Get-Item -LiteralPath $AssetPath).Length -eq 0) {
        throw "Generated asset missing or empty: $Asset"
    }
}
if (-not $SkipAutomation -and -not $SkipEditorPreparation) {
    Write-Host 'Running Unreal coordinate, mesh and ground contract automation...'
    Invoke-HiddenEditor -Arguments @("`"$Project`"", '-unattended', '-nop4', '-NullRHI', '-nosplash', '-stdout', '-FullStdOutLogOutput', '-ExecCmds="Automation RunTests VibeCoaster"', '-TestExit="Automation Test Queue Empty"', "-ReportExportPath=`"$RunLogs/Automation`"") -LogName 'Automation'
    $AutomationLog = Get-Content -LiteralPath (Join-Path $RunLogs 'Automation.stdout.log') -Raw
    if ($AutomationLog -notmatch 'Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.CoordinateContract\}' -or $AutomationLog -notmatch 'Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.MeshContract\}' -or $AutomationLog -notmatch 'Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.GroundContract\}' -or $AutomationLog -notmatch 'Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.StationArtContract\}' -or $AutomationLog -notmatch 'Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.ImportedArtContract\}' -or $AutomationLog -notmatch 'Result=\{Success\}[^\r\n]*Path=\{VibeCoaster\.SeedInputContract\}' -or $AutomationLog -match 'Result=\{Fail') {
        throw "Automation success was not confirmed. Inspect $RunLogs/Automation and $RunLogs/Automation.stdout.log."
    }
}
if ($PrepareOnly) { Write-Host "Prepared Unreal project: $Project"; return }
$PostPreparationStatus = @(& git -c "safe.directory=$RepositoryRoot" -C $RepositoryRoot status --porcelain --untracked-files=all)
if ($LASTEXITCODE -ne 0 -or $PostPreparationStatus.Count -ne 0) {
    throw 'Editor preparation changed reviewed source or assets. Review and commit them, then package with -SkipEditorPreparation.'
}

$RunArchive = Join-Path $OutputDirectory ('run-' + $RunId)
if (Test-Path -LiteralPath $RunArchive) { throw 'Packaging archive must be fresh.' }
Write-Host 'Building, cooking, staging, and packaging the native Win64 game...'
& $UAT 'BuildCookRun' "-project=$Project" '-noP4' '-platform=Win64' "-clientconfig=$Configuration" '-build' '-skipbuildeditor' "-ubtargs=-MaxParallelActions=$MaxParallelActions" '-cook' '-map=/Game/Maps/Ride' '-stage' '-pak' '-iostore' '-archive' "-archivedirectory=$RunArchive" '-prereqs' '-utf8output' 2>&1 | Tee-Object -FilePath (Join-Path $RunLogs 'Package.log')
if ($LASTEXITCODE -ne 0) { throw "Unreal packaging failed ($LASTEXITCODE)." }
$Executables = Get-ChildItem -LiteralPath $RunArchive -Filter 'VibeCoaster.exe' -File -Recurse
if (-not $Executables) { throw 'BuildCookRun returned success but no packaged VibeCoaster.exe was found.' }
$Executables | ForEach-Object { Write-Host "Packaged executable: $($_.FullName)" }
$Manifest = [ordered]@{
    SchemaVersion = 1; Release = $ReleaseVersion; Commit = $SourceCommit; Configuration = $Configuration
    Engine = $Version; CompilerWorkers = $MaxParallelActions; CreatedUtc = [DateTime]::UtcNow.ToString('o')
    BuildEvidence = $RunLogs
    ArtManifestSha256 = (Get-FileHash -LiteralPath $ArtManifestPath -Algorithm SHA256).Hash
    RuntimeArt = @($RuntimeArt | ForEach-Object { $_.name })
    Executables = @($Executables | ForEach-Object {
        [ordered]@{ Path = $_.FullName; Bytes = $_.Length; Sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
    })
}
$Manifest | ConvertTo-Json -Depth 10 | Set-Content -LiteralPath (Join-Path $RunArchive 'package-manifest.json') -Encoding utf8
Write-Host 'Packaging does not verify rendering. Check the packaged game on the target GPU.'
