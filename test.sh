#!/usr/bin/env bash
set -euo pipefail

export DOTNET_ROLL_FORWARD="${DOTNET_ROLL_FORWARD:-Major}"

# The test host asks whether the process that started it is still there, and
# some Windows machines refuse that query (Win32 5), which ends the run before
# a single test starts. Nothing here needs it: the host lives as long as the
# run does. Passed before "$@" so that a caller can still ask for it.
HOST="-p:_MSTestEnableParentProcessQuery=false"

dotnet test Core/Tests/StintegyEVO.Core.Tests.csproj --logger "console;verbosity=minimal" "$HOST" "$@"
dotnet test mods/official/Tests/official.Tests.csproj --logger "console;verbosity=minimal" "$HOST" "$@"
