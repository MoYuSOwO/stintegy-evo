#!/usr/bin/env bash
set -euo pipefail

export DOTNET_ROLL_FORWARD="${DOTNET_ROLL_FORWARD:-Major}"

dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj --logger "console;verbosity=minimal" "$@"
dotnet test mods/official/Tests/official.Tests.csproj --logger "console;verbosity=minimal" "$@"
