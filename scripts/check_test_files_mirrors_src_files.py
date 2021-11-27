#!/usr/bin/python3
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

# Checks if all source cpp files except for FILE_TO_IGNORE have a
# matching test file in the test directory.

import os, sys, glob

SCRIPT_DIR = os.path.dirname(__file__)
SOURCE_DIR = os.path.join(SCRIPT_DIR, '../aruw-mcb-project/src/')
TEST_DIR = os.path.join(SCRIPT_DIR, '../aruw-mcb-project/test/')
TEST_FILE_EXTENSION = '.cpp'
TEST_FILE_SUFFIX = "_tests.cpp"
TEST_GLOB_MATCHES_TO_IGNORE = [
    os.path.join(TEST_DIR, "aruwsrc/mock/*"),
    os.path.join(TEST_DIR, "aruwsrc/stub/*")
]

def find_all_source_files(test_directory, globed_files_to_ignore=[]):
    # Retreive all files
    files = [ os.path.join(dp, f) for dp, _, files in os.walk(test_directory) for f in files ]

    files_to_ignore = []
    for glob_match in globed_files_to_ignore:
        files_to_ignore += glob.glob(glob_match)

    # Filter out files to ignore and those without correct extension
    files = [ f for f in files if f not in files_to_ignore and os.path.splitext(f)[1] == TEST_FILE_EXTENSION ]

    return files

source_files =  find_all_source_files(SOURCE_DIR)
test_files = find_all_source_files(TEST_DIR, TEST_GLOB_MATCHES_TO_IGNORE)

errors = 0

for test_file in test_files:
    associated_src_file = test_file[:-len(TEST_FILE_SUFFIX)] + TEST_FILE_EXTENSION
    associated_src_file = associated_src_file.replace(TEST_DIR, SOURCE_DIR)

    if associated_src_file not in source_files:
        print('error: test file {0} doesn\'t have an associated source file'.format(test_file))
        errors += 1

retcode = not not errors

if retcode:
    print("Scanning complete, {0} test files that do not have associated source files are not found".format(errors))
else:
    print("Scanning complete, success!")

sys.exit(retcode)
