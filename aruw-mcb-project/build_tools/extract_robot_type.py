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

from SCons.Script import *

from .parse_args import USAGE

ROBOT_TYPE_FILE = "robot-type/robot_type.hpp"
VALID_ROBOT_TYPES = ["TARGET_STANDARD_WOODY",
                     "TARGET_STANDARD_ELSA",
                     "TARGET_STANDARD_SPIDER",
                     "TARGET_DRONE",
                     "TARGET_ENGINEER",
                     "TARGET_SENTRY_BEEHIVE",
                     "TARGET_HERO_CYCLONE",
                     "TARGET_BALSTD",
                     "TARGET_DART"]


def get_robot_type():
    robot_type = ARGUMENTS.get("robot")

    if robot_type not in VALID_ROBOT_TYPES:
        prompt = "Please enter a valid robot type out of the following:\n"
        for type in VALID_ROBOT_TYPES:
            prompt += type + "\n"
        prompt += "--> "
        robot_type = input(prompt)
    
    # Check against valid robot type
    if robot_type not in VALID_ROBOT_TYPES:
        raise Exception(USAGE)

    return "TARGET_" + robot_type
