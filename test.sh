#!/usr/bin/env bash
set -euo pipefail

export DOTNET_ROLL_FORWARD="${DOTNET_ROLL_FORWARD:-Major}"

dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj --logger "console;verbosity=minimal" "$@"
dotnet test Chassis/official/single_track/Tests/single_track.Tests.csproj --logger "console;verbosity=minimal" "$@"
