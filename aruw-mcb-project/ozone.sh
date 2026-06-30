#!/bin/bash
# Launch Windows SEGGER Ozone for a given robot target, from WSL.
# Usage: ./ozone.sh [robot]   e.g. ./ozone.sh engineer
set -euo pipefail

ROBOT="${1:-engineer}"
TARGET="TARGET_$(echo "$ROBOT" | tr '[:lower:]' '[:upper:]')"

PROJ_ROOT="$HOME/projects/ARUW/aruw-mcb/aruw-mcb-project"
JDEBUG="$PROJ_ROOT/build/hardware/scons-release/$TARGET/aruw-mcb.jdebug"
OZONE="/mnt/c/Program Files/SEGGER/Ozone/Ozone.exe"

if [[ ! -f "$JDEBUG" ]]; then
    echo "No .jdebug for $TARGET at:"
    echo "  $JDEBUG"
    echo "Build it first (e.g. 'scons build robot=$ROBOT' in the container)."
    exit 1
fi

WIN_JDEBUG="$(wslpath -w "$JDEBUG")"
echo "Launching Ozone with $WIN_JDEBUG"
"$OZONE" "$WIN_JDEBUG" &