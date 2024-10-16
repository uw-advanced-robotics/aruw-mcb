# Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#from dotenv import load_dotenv
import os
import subprocess

from os.path import join, abspath

from SCons.Script import *
from extract_robot_type import VALID_ROBOT_TYPES

def run_ozone(env, source, ip="1.1.1.1"):
    # if robot_name not in VALID_ROBOT_TYPES:
    #     raise SystemExit('Invalid Robot Type')
    def call_run_ozone(target, source, env):
        jdebug = f"{env['BUILDPATH']}/{env['CONFIG_PROJECT_NAME']}.jdebug"
        import sys
        if sys.platform == "win32":
            os.startfile(jdebug)
        elif sys.platform == "darwin":
            subprocess.call(['open', '-n', '-a', 'Ozone.app', '--args', jdebug])
        else:
            subprocess.call(['xdg-open', jdebug])

    
    action = Action(call_run_ozone)
    return env.AlwaysBuild(env.Alias("ozone_run", [generate_ozone(env), source], action))

def generate_ozone(env, ip="1.1.1.1"):
    # if robot_name not in VALID_ROBOT_TYPES:
    #     raise SystemExit('Invalid Robot Type')
    def call_generate_ozone(target, source, env):
        project_content = ""
        with open("./build_tools/example_ozone_project/example.jdebug") as r:
            project_content = r.read()

        project_file_path = f"{env['BUILDPATH']}/{env['CONFIG_PROJECT_NAME']}.jdebug"

        project_content = project_content.replace("${ROBOT_IP}", ip)
        project_content = project_content.replace("${BUILD_DIR}", env['BUILDPATH'])
        project_content = project_content.replace("${BUILD_DIR_LOWER}", env['BUILDPATH'].lower())

        target.append(env.File(project_file_path))
        target.append(env.File(f"{project_file_path}.user"))

        with open(f"{project_file_path}", "w+") as w:
            w.write(project_content)

        with open("./build_tools/example_ozone_project/example.jdebug.user") as w:
            project_user_content = w.read()

        with open(f"{project_file_path}.user", "w+") as w:
            w.write(project_user_content)

    action = Action(call_generate_ozone)
    return env.AlwaysBuild(env.Alias("ozone_generate", '', action))
# def generate_project_with_presets(robot_name):
#     load_dotenv()

#     robot_ip = os.getenv(robot_name)
#     ozone_version = os.getenv("version")
#     generate_project(robot_name, robot_ip, ozone_version)

# def create_project_for_all_robots():
#     load_dotenv()

#     for robot in VALID_ROBOT_TYPES:
#         robot_ip = os.getenv(robot)
#         ozone_version = os.getenv("version")

#         generate_project(robot, robot_ip, ozone_version)



#generate_project("SENTRY_HYDRA", "1.1.1.1", "3.38")
# generate_project_with_presets("SENTRY_HYDRA")

def generate(env, **kw):
    env.AddMethod(run_ozone, "RunOzoneConfig")
    env.AddMethod(generate_ozone, "GenerateOzoneConfig")
     
def exists(env):
     return True
	# return env.Detect("openocd") Maybe change this to detect ozone