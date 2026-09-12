#!/bin/bash
# Run on a real Mac. Bash 3.2-compatible; no Windows-to-Mac cross compilation.
set -euo pipefail

usage() {
    cat <<'USAGE'
Usage: bash package_macos.sh --unreal-root PATH [options]
  --configuration Development|Shipping  (default: Development)
  --architecture arm64|universal        (default: arm64; universal = arm64+x64)
  --output-directory PATH              (default: native/unreal/Packaged/Mac)
  --prepare-only                       Build editor, create assets, run UE tests
  --help
All builds run all three named UE automation contracts. Nothing is notarized or submitted.
USAGE
}
fail() {
    if [[ -n "${run_dir:-}" ]]; then
        printf 'ERROR: %s\n' "$*" | tee -a "$run_dir/Failure.txt" >&2
    else
        printf 'ERROR: %s\n' "$*" >&2
    fi
    exit 1
}
require_value() { [[ $# -ge 2 && -n "$2" ]] || fail "Missing value for $1"; }

unreal_root=''
configuration=Development
architecture=arm64
output_directory=''
prepare_only=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --unreal-root) require_value "$@"; unreal_root=$2; shift 2 ;;
        --configuration) require_value "$@"; configuration=$2; shift 2 ;;
        --architecture) require_value "$@"; architecture=$2; shift 2 ;;
        --output-directory) require_value "$@"; output_directory=$2; shift 2 ;;
        --prepare-only) prepare_only=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) fail "Unknown argument: $1" ;;
    esac
done
[[ -n "$unreal_root" ]] || { usage >&2; fail '--unreal-root is required'; }
case "$configuration" in Development|Shipping) ;; *) fail 'Configuration must be Development or Shipping' ;; esac
case "$architecture" in arm64) game_arch=arm64 ;; universal) game_arch=arm64+x64 ;; *) fail 'Architecture must be arm64 or universal' ;; esac
[[ $(uname -s) == Darwin ]] || fail 'A real macOS host with Xcode and the macOS Unreal Engine installation is required. No Mac build was attempted.'

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
project_root=$(cd -- "$script_dir/.." && pwd -P)
project="$project_root/VibeCoaster.uproject"
[[ -f "$project" ]] || fail "Project missing: $project"
mkdir -p "$project_root/Saved/MacPackaging"
run_dir=$(mktemp -d "$project_root/Saved/MacPackaging/run-$(date -u +%Y%m%d-%H%M%S)-XXXXXX")
trap 'result=$?; if [[ $result -ne 0 ]]; then printf "Failed (exit %s); retained evidence: %s\n" "$result" "$run_dir" >&2; fi' EXIT
printf 'Run evidence: %s\n' "$run_dir"
mkdir -p "$run_dir/Temp" "$run_dir/UATLogs" "$project_root/Saved/DerivedDataCache/Zen"
export TMPDIR="$run_dir/Temp/" TMP="$run_dir/Temp" TEMP="$run_dir/Temp"
# FApplePlatformMisc maps the engine's hyphenated names to underscores.
# Explicit Zen override also works when the DDC is inside the project tree.
export UE_LocalDataCachePath="$project_root/Saved/DerivedDataCache"
export UE_ZenDataPath="$project_root/Saved/DerivedDataCache/Zen"
export UE_ZenSubprocessDataPath="$UE_ZenDataPath"
export uebp_LogFolder="$run_dir/UATLogs" uebp_FinalLogFolder="$run_dir/UATLogs"
export VIBECOASTER_BOOTSTRAP_RECEIPT="$run_dir/ContentBootstrap.json"
export DOTNET_CLI_HOME="$project_root/Saved/DotNet" NUGET_PACKAGES="$project_root/Saved/NuGetPackages"
mkdir -p "$DOTNET_CLI_HOME" "$NUGET_PACKAGES"

run_logged() {
    local name=$1
    shift
    printf 'Running %s; log: %s/%s.log\n' "$name" "$run_dir" "$name"
    if "$@" >"$run_dir/$name.log" 2>&1; then
        return 0
    else
        local result=$?
        tail -n 50 "$run_dir/$name.log" >&2
        fail "$name failed (exit $result). Full log retained."
    fi
}
version_at_least() {
    awk -v actual="$1" -v minimum="$2" 'BEGIN {
        split(actual,a,"."); split(minimum,b,".");
        for(i=1;i<=3;i++){if(a[i]+0>b[i]+0)exit 0;if(a[i]+0<b[i]+0)exit 1}exit 0
    }'
}

[[ -d "$unreal_root" ]] || fail "Engine directory missing: $unreal_root"
unreal_root=$(cd -- "$unreal_root" && pwd -P)
build="$unreal_root/Engine/Build/BatchFiles/Mac/Build.sh"
uat="$unreal_root/Engine/Build/BatchFiles/RunUAT.sh"
editor="$unreal_root/Engine/Binaries/Mac/UnrealEditor.app/Contents/MacOS/UnrealEditor"
version_file="$unreal_root/Engine/Build/Build.version"
for required in "$build" "$uat" "$editor" "$version_file"; do
    [[ -f "$required" ]] || fail "macOS Unreal installation missing: $required"
done
[[ -x "$editor" ]] || fail "Unreal editor is not executable: $editor"
major=$(/usr/bin/plutil -extract MajorVersion raw -o - "$version_file")
minor=$(/usr/bin/plutil -extract MinorVersion raw -o - "$version_file")
case "$major.$minor" in
    5.8) min_macos=15.6; min_xcode=26.0 ;; # Xcode 26 build-host minimum (Apple)
    *) fail "UE $major.$minor has no reviewed Mac prerequisites here; the current source requires UE 5.8" ;;
esac
macos_version=$(/usr/bin/sw_vers -productVersion)
version_at_least "$macos_version" "$min_macos" || fail "UE $major.$minor requires a build host with macOS $min_macos or newer for Xcode 26"
developer_dir=$(/usr/bin/xcode-select -p) || fail 'Select a full Xcode installation with xcode-select'
[[ -d "$developer_dir/Platforms/MacOSX.platform" ]] || fail "Full Xcode required; selected path is $developer_dir"
run_logged XcodeVersion /usr/bin/xcodebuild -version
xcode_version=$(awk '/^Xcode /{print $2; exit}' "$run_dir/XcodeVersion.log")
[[ -n "$xcode_version" ]] || fail 'Could not read the selected Xcode version'
version_at_least "$xcode_version" "$min_xcode" || fail "UE $major.$minor requires Xcode $min_xcode or newer"
if [[ "$minor" == 8 && "$xcode_version" == 26.4* ]]; then
    fail 'Epic lists Xcode 26.4 as incompatible with UE 5.8; select a compatible Xcode (26.1.1 recommended)'
fi
run_logged XcodeSetup /usr/bin/xcodebuild -checkFirstLaunchStatus
run_logged MacSDK /usr/bin/xcrun --sdk macosx --show-sdk-path
run_logged Clang /usr/bin/xcrun --sdk macosx clang --version
run_logged Metal /usr/bin/xcrun --sdk macosx metal --version
case "$(uname -m)" in
    arm64) editor_arch=arm64 ;;
    x86_64)
        if [[ $(/usr/sbin/sysctl -in sysctl.proc_translated 2>/dev/null || true) == 1 ]]; then
            fail 'Run this script in a native arm64 terminal, outside Rosetta translation'
        fi
        editor_arch=x64
        [[ "$architecture" == universal ]] || fail 'The default arm64 build requires Apple Silicon; use --architecture universal explicitly on an Intel Mac'
        ;;
    *) fail 'Unsupported Mac host architecture' ;;
esac
{
    printf 'UTC: %s\nEngine: %s\nUE: %s.%s\nmacOS: %s\nXcode: %s\nDeveloperDir: %s\nEditorArchitecture: %s\nGameArchitectures: %s\nConfiguration: %s\n' \
        "$(date -u +%FT%TZ)" "$unreal_root" "$major" "$minor" "$macos_version" "$xcode_version" "$developer_dir" "$editor_arch" "$game_arch" "$configuration"
    printf 'LocalDDC: %s\nZen: %s\n' "$UE_LocalDataCachePath" "$UE_ZenDataPath"
    cat "$version_file"
} >"$run_dir/Environment.txt"

run_logged EditorBuild /bin/bash "$build" VibeCoasterEditor Mac Development "$project" \
    "-Architecture=$editor_arch" -WaitMutex -NoHotReloadFromIDE "-Log=$run_dir/UnrealBuildTool.log"
run_logged ContentBootstrap "$editor" "$project" /Engine/Maps/Entry \
    "-ExecutePythonScript=$script_dir/create_content.py" -unattended -nop4 -NullRHI -nosplash -stdout -FullStdOutLogOutput \
    "-abslog=$run_dir/ContentBootstrap.engine.log"
[[ -s "$VIBECOASTER_BOOTSTRAP_RECEIPT" ]] || fail 'Editor did not produce the unique content bootstrap receipt'
for asset in Maps/Ride.umap Materials/M_Rail.uasset Materials/M_Ground.uasset Materials/M_Ground_Plain.uasset Materials/M_Ground_Relief.uasset Materials/M_Structure.uasset Materials/M_Train.uasset Materials/M_Footing.uasset Art/V072/Import1/SM_TrainCar.uasset Art/V072/TrackWeb1/SM_TrackTieWeb.uasset Art/V072/Import1/SM_StationPlatformPanel.uasset Art/V072/Import1/SM_StationPlatformEndPanel.uasset Art/V072/Import1/SM_StationRoofPanel.uasset Art/V072/Import1/SM_StationPost.uasset; do
    [[ -s "$project_root/Content/$asset" ]] || fail "Generated asset missing or empty: $asset"
done
run_logged Automation "$editor" "$project" -unattended -nop4 -NullRHI -nosplash -stdout -FullStdOutLogOutput \
    '-ExecCmds=Automation RunTests VibeCoaster' '-TestExit=Automation Test Queue Empty' \
    "-ReportExportPath=$run_dir/Automation" "-abslog=$run_dir/Automation.engine.log"
for contract in CoordinateContract MeshContract TerrainBackdropContract CanyonRenderBudgetContract StationArtContract ImportedArtContract SeedInputContract OperationHardware; do
    grep -Eq "Result=\\{Success\\}.*Path=\\{VibeCoaster\\.$contract\\}" "$run_dir/Automation.log" || fail "UE automation success unconfirmed: VibeCoaster.$contract"
done
if grep -Eq 'Result=\{Fail' "$run_dir/Automation.log"; then fail 'UE automation reported a failure'; fi
if [[ "$prepare_only" == true ]]; then
    printf 'Prepared project and passed named UE automation. No package or rendering acceptance claimed. Evidence: %s\n' "$run_dir"
    exit 0
fi

[[ -n "$output_directory" ]] || output_directory="$project_root/Packaged/Mac"
mkdir -p "$output_directory"
output_directory=$(cd -- "$output_directory" && pwd -P)
archive="$output_directory/$(basename -- "$run_dir")"
[[ ! -e "$archive" ]] || fail "Archive must be fresh: $archive"
mkdir "$archive"
printf '%s\n' "$archive" >"$run_dir/ArchivePath.txt"
run_logged Package /bin/bash "$uat" BuildCookRun "-project=$project" -noP4 -platform=Mac \
    "-clientconfig=$configuration" "-clientarchitecture=$game_arch" "-editorarchitecture=$editor_arch" \
    -build -cook -map=/Game/Maps/Ride -stage -pak -iostore -archive \
    "-stagingdirectory=$run_dir/StagedBuilds" "-archivedirectory=$archive" -NoCodeSign -utf8output
found_app=false
while IFS= read -r -d '' app; do
    executable=$(/usr/bin/plutil -extract CFBundleExecutable raw -o - "$app/Contents/Info.plist")
    [[ -n "$executable" && "$executable" != */* && "$executable" != . && "$executable" != .. ]] || fail "Invalid app executable identity: $app"
    binary="$app/Contents/MacOS/$executable"
    [[ -x "$binary" ]] || fail "Packaged app lacks its executable: $binary"
    /usr/bin/lipo -verify_arch arm64 "$binary" || fail "Packaged app lacks arm64: $app"
    if [[ "$architecture" == universal ]]; then
        /usr/bin/lipo -verify_arch x86_64 "$binary" || fail "Universal package lacks x86_64: $app"
    fi
    printf 'Packaged app: %s\n' "$app" | tee -a "$run_dir/PackagedApps.txt"
    /usr/bin/lipo -archs "$binary" >>"$run_dir/PackagedApps.txt"
    found_app=true
done < <(find "$archive" -type d -name VibeCoaster.app -print0)
[[ "$found_app" == true ]] || fail 'UAT returned success but no fresh VibeCoaster.app exists'
printf 'Packaging completed. Rendering, gameplay, save/load and 1440p/60fps remain unverified until measured on this Mac. Evidence: %s\n' "$run_dir"
