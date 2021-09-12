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

if [[ "$#" -ne 1 ]]; then
    echo "usage: update_taproot_submodule.sh ./path-to-aruw-mcb-project"
    exit 1
fi

ARUW_MCB_PROJECT_PATH=$1
UPDATE_SUBMODULE_BRANCH="update-submodule"

git fetch
git checkout develop
git pull
git submodule foreach git checkout develop
git push origin --delete $UPDATE_SUBMODULE_BRANCH &>/dev/null
git checkout -b $UPDATE_SUBMODULE_BRANCH

cd $ARUW_MCB_PROJECT_PATH
rm -rf taproot
lbuild build >/dev/null
cd -

if [[ "$(git status | grep -c "nothing to commit")" != 1 ]]; then
    echo "Files have changed..."
    git status
    git commit -a -m "Update taproot submodule"
    (git push &>/dev/null) || (git push --set-upstream origin $UPDATE_SUBMODULE_BRANCH)

    # The description of our new MR, we want to remove the branch after the MR has
    # been closed
    BODY="{
        \"id\": 20675733,
        \"source_branch\": \"$UPDATE_SUBMODULE_BRANCH\",
        \"target_branch\": \"develop\",
        \"remove_source_branch\": false,
        \"title\": \"CI Submodule Update\"
    }"

    API4_MR_URL="https://gitlab.com/api/v4/projects/20675733/merge_requests"
    LISTMR=`curl --silent "$API4_MR_URL?state=opened" --header "PRIVATE-TOKEN: $PRIVATE_TOKEN"`
    COUNTBRANCHES=`echo $LISTMR | grep -o "\"source_branch\":\"$UPDATE_SUBMODULE_BRANCH\"" | wc -l`

    if [[ $COUNTBRANCHES -eq "0" ]]; then
        curl -X POST "$API4_MR_URL" \
            --header "PRIVATE-TOKEN: $PRIVATE_TOKEN" \
            --header "Content-Type: application/json" \
            --data "$BODY" >/dev/null
        echo "Opened submodule update MR"
    else
        echo "MR already opened, changes pushed"
    fi
else
    echo "No updates, no MR opened"
fi
