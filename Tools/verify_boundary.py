#!/usr/bin/env python3
"""Reproducible boundary gate: Core, default build, 3D logic/rendering and external composition.

Pass --render for real screenshots (opens a Godot window). Headless screenshots
are explicitly skipped and never count as visual verification.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / '.tmp' / 'boundary-verification'
OUT.mkdir(parents=True, exist_ok=True)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--godot', default=os.environ.get('GODOT_BIN', shutil.which('godot')))
parser.add_argument('--render', action='store_true')
args = parser.parse_args()
ENV = dict(os.environ, DOTNET_ROLL_FORWARD=os.environ.get('DOTNET_ROLL_FORWARD', 'Major'))
# Ignore a caller's telemetry path: verification may only write its own artifacts.
ENV.pop('STINTEGY_CSV_TELEMETRY', None)
results = []


def run(name, command, marker=None, timeout=180, environment=None):
    print(f'[{name}] {" ".join(map(str, command))}', flush=True)
    path = OUT / (name + '.log')
    with path.open('w') as log:
        completed = subprocess.run(command, cwd=ROOT, env=environment or ENV,
                                   stdout=log, stderr=subprocess.STDOUT, timeout=timeout)
    text = path.read_text()
    success = completed.returncode == 0 and (marker is None or marker in text)
    if marker and re.search(r'^(SCRIPT ERROR|ERROR):', text, re.M):
        success = False
    results.append(dict(check=name, passed=success, log=str(path), exit_code=completed.returncode))
    print(text[-2500:], flush=True)
    if not success:
        raise RuntimeError(f'{name} failed; see {path}')


def check_sources():
    # Check the candidate checkout, including not-yet-committed additions but not
    # unrelated ignored scratch files. Deleted tracked paths must not count.
    listed = subprocess.check_output(['git', 'ls-files', '-z', '--cached', '--others', '--exclude-standard'], cwd=ROOT)
    files = [ROOT / p.decode() for p in listed.split(b'\0') if p]
    errors = []
    legacy = re.compile(r'\b(?:IRaceDriver|ReferenceLineDriver|DirectDriveRaceDriver|ITrafficMotionPlanSource|TrafficMotionPlan|RacingRoomCoordinator|VehicleSpeedPlanner|StanleyPathPredictor|DirectDriveObservation|MlpDrivingPolicy|DriverCatalog|ReflexGovernedLongitudinal)\b')
    for path in files:
        if not path.is_file():
            continue
        rel = path.relative_to(ROOT).as_posix()
        if rel.startswith(('Training/', 'Core/Drivers/Learned/', 'Core/Track/RefLines/', 'Core/Track/Numerics/')):
            errors.append(rel)
        if rel.startswith('Assets/') and path.suffix in {'.nn', '.onnx', '.pt', '.pth', '.ckpt', '.safetensors'}:
            errors.append(rel)
        if path.suffix == '.cs' and rel.startswith(('Core/', 'GodotApp/')) and not rel.startswith('Core/Tests/'):
            if legacy.search(path.read_text()):
                errors.append(rel + ': legacy implementation reference')
    if errors:
        raise RuntimeError('Boundary violations:\n' + '\n'.join(errors))
    (OUT / 'source-boundary.json').write_text(json.dumps({'passed': True, 'checked_files': len(files)}, indent=2))
    results.append({'check': 'source-boundary', 'passed': True})


try:
    if not args.godot:
        raise RuntimeError('Set GODOT_BIN or --godot to the Godot .NET executable.')
    version = subprocess.check_output([args.godot, '--version'], env=ENV, text=True).strip()
    if 'mono' not in version:
        raise RuntimeError(f'Godot .NET/Mono is required, got {version}.')
    check_sources()
    run('diff-check', ['git', 'diff', '--check'])
    run('core-tests', ['dotnet', 'test', 'Core/Tests/StintegyEVO.Core.Tests.csproj', '-c', 'Release', '--logger', 'console;verbosity=minimal'], timeout=300)
    run('app-release', ['dotnet', 'build', 'StintegyEVO.csproj', '-c', 'Release'])
    # The opt-in fixture is compiled only during this step, then removed from the
    # ordinary Debug assembly in finally, even when an integration assertion fails.
    try:
        run('composition-build', ['dotnet', 'build', 'StintegyEVO.csproj', '-c', 'Debug', '-p:PresentationSmoke=true'])
        run('composition-smoke', [args.godot, '--headless', '--path', str(ROOT), 'res://Tools/Tests/composition_smoke.tscn'], 'COMPOSITION PASS', timeout=90)
    finally:
        run('app-debug', ['dotnet', 'build', 'StintegyEVO.csproj', '-c', 'Debug', '-p:PresentationSmoke=false'])
    # Pre-import textures in a fresh worktree before asking for rendered captures.
    run('godot-import', [args.godot, '--headless', '--editor', '--path', str(ROOT), '--import', '--quit'], timeout=90)
    mode = [] if args.render else ['--headless']
    # Never inherit a user's telemetry output path and overwrite unrelated data.
    smoke_env = dict(ENV, STINTEGY_CSV_TELEMETRY=str(OUT / 'telemetry.csv'))
    run('3d-smoke', [args.godot, *mode, '--path', str(ROOT), '--script', 'res://Tools/lowpoly_smoke.gd'], 'SMOKE PASS', timeout=100, environment=smoke_env)
    run('3d-motion', [args.godot, *mode, '--path', str(ROOT), '--script', 'res://Tools/lowpoly_motion_smoke.gd'], 'MOTION PASS', timeout=60, environment=dict(ENV, STINTEGY_CSV_TELEMETRY=str(OUT / 'motion-telemetry.csv')))
    import csv
    with (OUT / 'telemetry.csv').open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    if not rows or not all(None not in row and all(v is not None for v in row.values()) for row in rows):
        raise RuntimeError('3D telemetry is empty or has malformed rows.')
    results.append({'check': '3d-telemetry', 'passed': True, 'rows': len(rows)})
    print('BOUNDARY VERIFICATION PASS', flush=True)
except Exception as error:
    results.append({'check': 'run', 'passed': False, 'error': str(error)})
    print(f'BOUNDARY VERIFICATION FAIL: {error}', file=sys.stderr)
    sys.exit(1)
finally:
    (OUT / 'summary.json').write_text(json.dumps({'visual': args.render, 'results': results}, indent=2))
