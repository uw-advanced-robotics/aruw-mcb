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

if [[ "$#" -ne 1 ]]; then
    echo "usage: ./check_lbuild_diff.sh ./path/to/lbuild/dir"
    exit 1
fi

LBUILD_DIR=$1
TEMP_DIR="tmp"

cd "$LBUILD_DIR"

cp -r "aruwlib" $TEMP_DIR

lbuild build
if [[ "$?" != 0 ]]; then
    exit 1
fi

if [[ ! -z "$(git diff "$TEMP_DIR" "aruwlib")" ]]; then
    echo "Generated lbuild is different, diff:"
    git diff "$TEMP_DIR" "aruwlib"
    rm -r $TEMP_DIR
    exit 1
else
    rm -r $TEMP_DIR
    exit 0
fi
