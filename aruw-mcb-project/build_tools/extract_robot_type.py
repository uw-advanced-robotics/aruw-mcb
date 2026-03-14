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

from build_tools.parse_args import USAGE

# TODO: Make this sync up with check.py and c_cpp_properties.json if possible
VALID_ROBOT_TYPES = [
    "STANDARD_NULL",
    "STANDARD_VOID",
    "DRONE",
    "ENGINEER",
    "SENTRY_ECLIPSE",
    "HERO_ZERO",
    "DART",
    "TESTBED",
    "BLANK",
    "MOTOR_TESTER",
    "LAUNCHER_TARGET",
    "FLYWHEEL_TESTING",
    "CHARACTERIZER",
]

ROBOT_CLASS = {
    "STANDARD_NULL": "standard",
    "STANDARD_VOID": "standard",
    "DRONE": "drone",
    "ENGINEER": "engineer",
    "SENTRY_ECLIPSE": "sentry",
    "HERO_ZERO": "hero",
    "DART": "dart",
    "TESTBED": "testbed",
    "BLANK": "blank",
    "MOTOR_TESTER": "motor_tester",
    "FLYWHEEL_TESTING" : "flywheel_testing",
    "LAUNCHER_TARGET" : "launcher_target",
    "CHARACTERIZER": "characterizer",
}

# Make sure that all robots have a class
assert all([robot in ROBOT_CLASS.keys() for robot in VALID_ROBOT_TYPES])

def search_for_robot_type(query):
    return [robot for robot in VALID_ROBOT_TYPES if query.lower() in robot.lower()] if query else []

def get_robot_type():
    robot_query = ARGUMENTS.get("robot")
    robot_type_matches = search_for_robot_type(robot_query)

    if len(robot_type_matches) != 1:
        if not robot_query or not robot_type_matches:
            prompt = "Please enter a valid robot type out of the following:\n"
        else:
            prompt = "Robot type is ambiguous, please enter a valid robot type or unique substring out of the following:\n"

        for type in VALID_ROBOT_TYPES:
            prompt += type + "\n"
        prompt += "--> "
        robot_query = input(prompt)
        robot_type_matches = search_for_robot_type(robot_query)
    
    # Check against valid robot type
    if len(robot_type_matches) != 1:
        raise Exception(USAGE)

    return "TARGET_" + robot_type_matches[0]
