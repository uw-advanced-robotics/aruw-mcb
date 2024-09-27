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

# TODO: Make this sync up with check.py and c_cpp_properties.json if possible
VALID_ROBOT_TYPES   = [ "STANDARD_ELSA",
                        "STANDARD_SPIDER",
                        "STANDARD_ORION",
                        "STANDARD_CYGNUS",
                        "DRONE",
                        "ENGINEER",
                        "SENTRY_HYDRA",
                        "HERO_PERSEUS",
                        "DART",
                        "TESTBED",
                        "MOTOR_TESTER" ]

ROBOT_CLASS = {
    "STANDARD_ELSA": "standard",
    "STANDARD_SPIDER": "standard",
    "STANDARD_ORION": "standard",
    "STANDARD_CYGNUS": "standard",
    "DRONE": "drone",
    "ENGINEER": "engineer",
    "SENTRY_HYDRA": "sentry",
    "HERO_PERSEUS": "hero",
    "DART": "dart",
    "TESTBED": "testbed"
}

# Make sure that all robots have a class
assert all([robot in ROBOT_CLASS.keys() for robot in VALID_ROBOT_TYPES])

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
