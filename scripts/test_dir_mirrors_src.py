#!/usr/bin/python3
#
# Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

# Checks if all source cpp files except for FILE_TO_IGNORE have a
# matching test file in the test directory.

import os, sys

SCRIPT_DIR = os.path.dirname(__file__)
SOURCE_FILE_EXTENSION = '.cpp'
SOURCE_DIR = os.path.join(SCRIPT_DIR, '../aruw-mcb-project/src/')
TEST_DIR = os.path.join(SCRIPT_DIR, '../aruw-mcb-project/test/')
# If the source file uses snake case, the test file suffix should be '_tests.cpp',
# vs. camel case, where the suffix should be 'Tests.cpp'
POSSIBLE_TEST_FILE_SUFFIXES = {'snake case': '_tests.cpp', 'camel case' : 'Tests.cpp' }

SOURCE_FILES_TO_IGNORE = [ os.path.join(SCRIPT_DIR, path) for path in [
        '../aruw-mcb-project/src/main.cpp',
        '../aruw-mcb-project/src/aruwsrc/control/drone_control.cpp',
        '../aruw-mcb-project/src/aruwsrc/control/engineer_control.cpp',
        '../aruw-mcb-project/src/aruwsrc/control/hero_control.cpp',
        '../aruw-mcb-project/src/aruwsrc/control/old_soldier_control.cpp',
        '../aruw-mcb-project/src/aruwsrc/control/sentinel_control.cpp',
        '../aruw-mcb-project/src/aruwsrc/control/soldier_control.cpp'
    ]]

def find_all_source_files(filepath, files_to_ignore):
    files = [ os.path.join(dp, f) for dp, dn, filenames in os.walk(filepath) for f in filenames ]
    files = [ f for f in files if f not in files_to_ignore ]
    files = [ f for f in files if os.path.splitext(f)[1] == SOURCE_FILE_EXTENSION ]
    return files

def get_possible_test_file_name(file):
    case_type = 'snake case'
    if file[0].isupper():
        case_type = 'camel case'
    file = os.path.splitext(file)[0] + POSSIBLE_TEST_FILE_SUFFIXES[case_type] 
    return file.replace(SOURCE_DIR, TEST_DIR)

source_files = find_all_source_files(SOURCE_DIR, SOURCE_FILES_TO_IGNORE)
test_files = find_all_source_files(TEST_DIR, [])

errors = 0

for source_file in source_files:
    possible_test_name = get_possible_test_file_name(source_file)
    if possible_test_name not in test_files:
        print('error: source file {0} not mirrored in test directory'.format(source_file))
        errors += 1

print("Scanning complete, {0} files that should be mirrored but are not found".format(errors))

sys.exit(not not errors)
