#!/bin/bash

targets="\
    TARGET_SOLDIER \
    TARGET_OLD_SOLDIER \
    TARGET_ENGINEER \
    TARGET_HERO \
    TARGET_SENTINEL \
    TARGET_DRONE"

for target in $targets; do
    echo ===============================================================================
    echo building $target
    echo ===============================================================================
    /usr/bin/env python3 $(which scons) build robot=$target
    if [ $? -ne 0 ]; then
        exit -1
    fi
    /usr/bin/env python3 $(which scons) run-tests robot=$target
    if [ $? -ne 0 ]; then
        exit -1
    fi
done
