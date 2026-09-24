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
  --skip-editor-preparation            Package already committed assets without reimporting
  --help
Editor preparation imports current runtime art and runs all six named UE automation contracts.
Review and commit any generated content before packaging. Nothing is notarized or submitted.
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
skip_editor_preparation=false
while [[ $# -gt 0 ]]; do
    case "$1" in
        --unreal-root) require_value "$@"; unreal_root=$2; shift 2 ;;
        --configuration) require_value "$@"; configuration=$2; shift 2 ;;
        --architecture) require_value "$@"; architecture=$2; shift 2 ;;
        --output-directory) require_value "$@"; output_directory=$2; shift 2 ;;
        --prepare-only) prepare_only=true; shift ;;
        --skip-editor-preparation) skip_editor_preparation=true; shift ;;
        --help|-h) usage; exit 0 ;;
        *) fail "Unknown argument: $1" ;;
    esac
done
[[ -n "$unreal_root" ]] || { usage >&2; fail '--unreal-root is required'; }
[[ "$prepare_only" == false || "$skip_editor_preparation" == false ]] || fail '--prepare-only cannot be combined with --skip-editor-preparation'
case "$configuration" in Development|Shipping) ;; *) fail 'Configuration must be Development or Shipping' ;; esac
case "$architecture" in arm64) game_arch=arm64 ;; universal) game_arch=arm64+x64 ;; *) fail 'Architecture must be arm64 or universal' ;; esac
[[ $(uname -s) == Darwin ]] || fail 'A real macOS host with Xcode and the macOS Unreal Engine installation is required. No Mac build was attempted.'

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
project_root=$(cd -- "$script_dir/.." && pwd -P)
project="$project_root/VibeCoaster.uproject"
repository_root=$(cd -- "$project_root/../.." && pwd -P)
source_commit=$(git -c "safe.directory=$repository_root" -C "$repository_root" rev-parse HEAD) || fail 'Could not identify source commit'
[[ "$source_commit" =~ ^[0-9a-f]{40}$ ]] || fail 'Packaging needs an identified source commit'
version_header="$repository_root/native/core/include/coaster/version.hpp"
release_version=$(sed -n 's/^#define COASTER_GENERATOR_VERSION "\([^"]*\)"/\1/p' "$version_header")
[[ -n "$release_version" ]] || fail 'Canonical release version is missing'
game_config="$project_root/Config/DefaultGame.ini"
grep -Fxq "ProjectVersion=$release_version" "$game_config" || fail "ProjectVersion must equal canonical release $release_version before packaging"
source_status=$(git -c "safe.directory=$repository_root" -C "$repository_root" status --porcelain --untracked-files=all) || fail 'Could not inspect source status'
if [[ "$prepare_only" == false && -n "$source_status" ]]; then
    fail 'Commit the reviewed source before producing a versioned package'
fi
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

if [[ "$skip_editor_preparation" == false ]]; then
run_logged EditorBuild /bin/bash "$build" VibeCoasterEditor Mac Development "$project" \
    "-Architecture=$editor_arch" -WaitMutex -NoHotReloadFromIDE "-Log=$run_dir/UnrealBuildTool.log"
run_logged ContentBootstrap "$editor" "$project" /Engine/Maps/Entry \
    "-ExecutePythonScript=$script_dir/create_content.py" -unattended -nop4 -nosplash -stdout -FullStdOutLogOutput \
    "-abslog=$run_dir/ContentBootstrap.engine.log"
[[ -s "$VIBECOASTER_BOOTSTRAP_RECEIPT" ]] || fail 'Editor did not produce the unique content bootstrap receipt'
art_receipt="$project_root/Saved/V3ArtImport.json"
rm -f -- "$art_receipt"
run_logged ArtImport "$editor" "$project" /Engine/Maps/Entry \
    "-ExecutePythonScript=$script_dir/import_v2_art.py" -unattended -nop4 -nosplash -stdout -FullStdOutLogOutput \
    "-abslog=$run_dir/ArtImport.engine.log"
[[ -s "$art_receipt" ]] || fail 'Editor did not produce the art import receipt'
fi
art_manifest="$repository_root/native/art/exports/manifest.json"
[[ -s "$art_manifest" ]] || fail "Art manifest missing: $art_manifest"
runtime_art=$(/usr/bin/python3 - "$art_manifest" <<'PY'
import json, re, sys
with open(sys.argv[1], encoding='utf-8') as stream:
    assets = json.load(stream)['assets']
names = [asset['name'] for asset in assets if asset.get('runtime', True)]
if not names or any(not re.fullmatch(r'SM_[A-Za-z0-9_]+', name) for name in names):
    raise SystemExit('Invalid runtime art identities in manifest')
materials = sorted({material for asset in assets if asset.get('runtime', True)
                    for material in asset['materials']})
if any(not re.fullmatch(r'VC2_[A-Za-z0-9_]+', name) for name in materials):
    raise SystemExit('Invalid runtime material identities in manifest')
print('\n'.join('mesh:' + name for name in names))
print('\n'.join('material:' + name for name in materials))
PY
) || fail 'Could not read runtime art manifest'
for asset in Maps/Ride.umap Materials/M_Rail.uasset Materials/M_LSM.uasset Materials/M_Brake.uasset Materials/M_Ground_Highlands.uasset Materials/M_Structure.uasset Materials/M_Train.uasset Materials/M_Footing.uasset; do
    [[ -s "$project_root/Content/$asset" ]] || fail "Generated asset missing or empty: $asset"
done
while IFS= read -r entry; do
    case "$entry" in
        mesh:*)
            name=${entry#mesh:}
            [[ -s "$project_root/Content/Art/V3/$name.uasset" ]] || fail "Runtime art asset missing or empty: $name" ;;
        material:*)
            name=${entry#material:}
            [[ -s "$project_root/Content/Art/V3/Materials/M_$name.uasset" ]] || fail "Runtime art material missing or empty: $name" ;;
        *) fail "Unexpected runtime art manifest entry: $entry" ;;
    esac
done <<< "$runtime_art"
if [[ "$skip_editor_preparation" == false ]]; then
run_logged Automation "$editor" "$project" -unattended -nop4 -NullRHI -nosplash -stdout -FullStdOutLogOutput \
    '-ExecCmds=Automation RunTests VibeCoaster' '-TestExit=Automation Test Queue Empty' \
    "-ReportExportPath=$run_dir/Automation" "-abslog=$run_dir/Automation.engine.log"
for contract in CoordinateContract MeshContract GroundContract StationArtContract ImportedArtContract SeedInputContract; do
    grep -Eq "Result=\\{Success\\}.*Path=\\{VibeCoaster\\.$contract\\}" "$run_dir/Automation.log" || fail "UE automation success unconfirmed: VibeCoaster.$contract"
done
if grep -Eq 'Result=\{Fail' "$run_dir/Automation.log"; then fail 'UE automation reported a failure'; fi
fi
if [[ "$prepare_only" == true ]]; then
    printf 'Prepared project and passed named UE automation. No package or rendering acceptance claimed. Evidence: %s\n' "$run_dir"
    exit 0
fi

post_preparation_status=$(git -c "safe.directory=$repository_root" -C "$repository_root" status --porcelain --untracked-files=all) || fail 'Could not inspect source status after editor preparation'
[[ -z "$post_preparation_status" ]] || fail 'Editor preparation changed reviewed source or assets; review and commit, then package from the committed content'
[[ -n "$output_directory" ]] || output_directory="$project_root/Packaged/Mac"
mkdir -p "$output_directory"
output_directory=$(cd -- "$output_directory" && pwd -P)
archive="$output_directory/$(basename -- "$run_dir")"
[[ ! -e "$archive" ]] || fail "Archive must be fresh: $archive"
mkdir "$archive"
printf '%s\n' "$archive" >"$run_dir/ArchivePath.txt"
run_logged Package /bin/bash "$uat" BuildCookRun "-project=$project" -noP4 -platform=Mac \
    "-clientconfig=$configuration" "-clientarchitecture=$game_arch" "-editorarchitecture=$editor_arch" \
    -build -skipbuildeditor -cook -map=/Game/Maps/Ride -stage -pak -iostore -archive \
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
/usr/bin/python3 - "$archive" "$source_commit" "$release_version" "$configuration" "$architecture" "$art_manifest" "$run_dir" <<'PY' || fail 'Could not write package manifest'
import datetime, hashlib, json, pathlib, plistlib, sys
archive, commit, release, configuration, architecture, art_manifest, evidence = sys.argv[1:]
root = pathlib.Path(archive)
binaries = []
for app in sorted(root.glob('**/VibeCoaster.app')):
    with (app / 'Contents/Info.plist').open('rb') as stream:
        executable = plistlib.load(stream)['CFBundleExecutable']
    binary = app / 'Contents/MacOS' / executable
    if not binary.is_file():
        raise SystemExit(f'No executable in packaged app: {app}')
    binaries.append(binary)
if not binaries:
    raise SystemExit('No packaged game executable to record')
def sha256(path):
    digest = hashlib.sha256()
    with open(path, 'rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()
with open(art_manifest, encoding='utf-8') as stream:
    assets = json.load(stream)['assets']
manifest = {
    'schemaVersion': 1, 'release': release, 'commit': commit,
    'configuration': configuration, 'architecture': architecture,
    'createdUtc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'buildEvidence': evidence, 'artManifestSha256': sha256(art_manifest),
    'runtimeArt': [asset['name'] for asset in assets if asset.get('runtime', True)],
    'executables': [{'path': str(path), 'bytes': path.stat().st_size, 'sha256': sha256(path)} for path in binaries],
}
(root / 'package-manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
PY
printf 'Packaging completed. Rendering, gameplay, save/load and 1440p/60fps remain unverified until measured on this Mac. Evidence: %s\n' "$run_dir"
