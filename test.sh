#!/usr/bin/env bash
set -euo pipefail

export DOTNET_ROLL_FORWARD="${DOTNET_ROLL_FORWARD:-Major}"

dotnet test Core/Tests/TheStint.Core.Tests.csproj --logger "console;verbosity=minimal" "$@"
