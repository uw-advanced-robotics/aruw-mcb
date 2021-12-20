#!/bin/bash
#
# Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
#
# This file is part of aruw-mcb.
#
# aruw-mcb is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# aruw-mcb is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
#
# This script validates that the git repository has the most up to date
# lbuild directory structure. Note at this script may change the state
# of your repository's generated code and thus you should not use it on
# a repository that has uncommitted changes.

if [[ "$#" -ne 2 ]]; then
    echo "usage: ./check_lbuild_diff.sh ./path/to/lbuild/dir ./path/to/taproot"
    exit 1
fi

LBUILD_DIR=$1
TAPROOT_DIR=$2
TEMP_DIR="tmp"

# First ensure the taproot commit is actually
# on develop
cd $TAPROOT_DIR
TAPROOT_COMMIT_SHA="$(git rev-parse HEAD)"
SHA_ON_DEVELOP="$(git branch -r --contains $TAPROOT_COMMIT_SHA | grep develop)"
if [[ -z $SHA_ON_DEVELOP ]]; then
    echo "sha not valid, must be on develop"
    exit 1
fi
cd -

# Next ensure generated taproot changes match the taproot repo's generated
# taproot
cd "$LBUILD_DIR"

cp -r "taproot" $TEMP_DIR

rm -rf "taproot"

lbuild build
if [[ "$?" != 0 ]]; then
    exit 1
fi

# Remove all project.xml.log files that were generated when running lbuild build
shopt -s globstar  # enable globstar, double astrisk (https://askubuntu.com/questions/1010707/how-to-enable-the-double-star-globstar-operator)
rm -f ./**/project.xml.log

if [[ ! -z "$(git diff --no-index "$TEMP_DIR" "taproot")" ]]; then
    echo "Generated lbuild is different, diff:"
    git diff --no-index "$TEMP_DIR" "taproot"
    rm -r $TEMP_DIR
    exit 1
else
    rm -r $TEMP_DIR
    exit 0
fi
