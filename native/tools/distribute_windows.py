"""Make a portable player ZIP from a successfully packaged Unreal game.

The source package is read-only. Output directories must be fresh. Debug symbols
stay in the source package; runtime files and redistribution notices are retained.
"""
import argparse
import hashlib
import json
import re
import shutil
import zipfile
from pathlib import Path


def sha256(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def distribute(package, output, version):
    package, output = package.resolve(strict=True), output.resolve()
    if not re.fullmatch(r'[0-9A-Za-z][0-9A-Za-z.-]*', version):
        raise ValueError('Version must be a simple filename-safe identifier')
    if output == package or package in output.parents:
        raise ValueError('Distribution must be outside the original package')
    required = ['VibeCoaster.exe', 'VibeCoaster/Binaries/Win64/VibeCoaster.exe',
                'VibeCoaster/Content/Paks/global.utoc',
                'VibeCoaster/Content/Paks/global.ucas',
                'VibeCoaster/Content/Paks/VibeCoaster-Windows.utoc',
                'VibeCoaster/Content/Paks/VibeCoaster-Windows.ucas',
                'VibeCoaster/Content/Paks/VibeCoaster-Windows.pak',
                'NOTICES.txt', 'Engine/Extras/Redist/en-us/vc_redist.x64.exe']
    for name in required:
        if not (package / name).is_file():
            raise ValueError(f'Incomplete package: {name}')
    stem = f'VibeCoaster-{version}-Windows'
    folder, archive = output / stem, output / (stem + '.zip')
    if folder.exists() or archive.exists() or archive.with_suffix('.zip.sha256').exists():
        raise FileExistsError('Preserve existing delivery; choose a fresh output')
    folder.mkdir(parents=True)
    source_files = []
    for path in sorted(package.rglob('*')):
        if not path.is_file() or path.suffix.lower() == '.pdb' or path.name == 'Manifest_DebugFiles_Win64.txt':
            continue
        if path.is_symlink():
            raise ValueError(f'Unexpected package symlink: {path}')
        relative = path.relative_to(package)
        target = folder / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)
        expected = sha256(path)
        if sha256(target) != expected:
            raise IOError(f'Copy verification failed: {relative}')
        source_files.append(dict(path=relative.as_posix(), sha256=expected, bytes=target.stat().st_size))
    (folder / 'Play VibeCoaster.cmd').write_bytes(
        b'@echo off\r\nsetlocal\r\ncd /d "%~dp0"\r\n'
        b'start "" "%~dp0VibeCoaster.exe" -windowed -ResX=1600 -ResY=900 -ExecCmds="t.MaxFPS 60"\r\n')
    (folder / 'README-FIRST.txt').write_text(f'''VibeCoaster {version} - Windows development preview

Extract the entire ZIP before playing. Open Play VibeCoaster.cmd for a
1600x900 window capped at 60fps, or VibeCoaster.exe for normal saved settings.
Keep Engine and VibeCoaster folders beside the executable. Unreal Editor,
Blender and a compiler are not needed to play.

First ride:
1. Use Up/Down to select Mode, then Left/Right to choose PHYSICS-PROOF.
2. Select Canyon and seed 42 for the reviewed example.
3. Press Enter to generate, then Space to ride. Tab shows/hides setup.
4. During riding: 1/2/3 select front/middle/rear, M overview, R restart.
5. F5 saves; F9 loads. C comparison; T telemetry. Alt+F4 quits.

ALL RECORDS is explicitly unavailable without authentic eligible reference
recordings. PHYSICS-PROOF retains height, speed, launch, inversion and
numerical acceptance requirements; it is not an intensity record claim.

If Windows reports missing MSVC runtime DLLs, run the included
Engine/Extras/Redist/en-us/vc_redist.x64.exe, then start the game again.
This unsigned development preview may show a Windows publisher warning.
Mac binaries are not available yet. No new performance benchmark was run
for this delivery while Cities: Skylines 2 was open.

Source and status: https://github.com/Banrs/OpenVibeCoaster
''', encoding='utf-8')
    (folder / 'runtime-files.json').write_text(json.dumps(dict(version=version, files=source_files), indent=2) + '\n', encoding='utf-8')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED, compresslevel=1) as zipped:
        for path in sorted(folder.rglob('*')):
            if path.is_file():
                zipped.write(path, path.relative_to(output).as_posix())
    with zipfile.ZipFile(archive) as zipped:
        if zipped.testzip() is not None:
            raise IOError('ZIP CRC verification failed')
    digest = sha256(archive)
    archive.with_suffix('.zip.sha256').write_text(f'{digest}  {archive.name}\n', encoding='ascii')
    return dict(folder=str(folder), archive=str(archive), sha256=digest, runtimeFiles=len(source_files))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--version', required=True)
    args = parser.parse_args()
    print(json.dumps(distribute(args.package, args.output, args.version), indent=2))
