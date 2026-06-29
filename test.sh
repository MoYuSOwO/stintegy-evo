#!/usr/bin/env bash
set -euo pipefail

export GODOT_BIN="${GODOT_BIN:-/Applications/Godot_mono.app/Contents/MacOS/Godot}"
export DOTNET_ROLL_FORWARD="${DOTNET_ROLL_FORWARD:-Major}"

dotnet test StintegyEVO.csproj --logger "console;verbosity=minimal" "$@"
