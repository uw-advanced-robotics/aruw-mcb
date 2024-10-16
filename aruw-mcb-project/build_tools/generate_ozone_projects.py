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
from dotenv import load_dotenv
import os

from os.path import join, abspath

from SCons.Script import *
from extract_robot_type import VALID_ROBOT_TYPES

def run_ozone(env, source, ip="1.1.1.1"):
    # if robot_name not in VALID_ROBOT_TYPES:
    #     raise SystemExit('Invalid Robot Type')
    def call_run_ozone(target, source, env):
        project_root_path = env["BUILDPATH"]
        
        project_content = ""
        with open("./build_tools/example_ozone_project/example.jdebug") as r:
            project_content = r.read()

        project_content = project_content.replace("${FILE_PATH}", project_root_path)
        project_content = project_content.replace("${ROBOT_IP}", ip)

        project_file_path = os.path.join(project_root_path, f"{env["CONFIG_PROJECT_NAME"]}.jdebug")

        with open(f"{project_root_path}/{env["CONFIG_PROJECT_NAME"]}.jdebug", "w+") as w:
            w.write(project_content)

        with open("./build_tools/example_ozone_project/example.jdebug.user") as w:
            project_user_content = w.read()

        with open(f"{project_root_path}/{env["CONFIG_PROJECT_NAME"]}.jdebug.user", "w+") as w:
            w.write(project_user_content)

        os.startfile(project_file_path)
    action = Action(call_run_ozone)
    return env.AlwaysBuild(env.Alias("ozone_generate", source, action))

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
	env.AddMethod(run_ozone, "GenerateOzoneConfig")
	# env.AddMethod(debug_openocd, "DebugOpenOcd")
	# env.AddMethod(reset_openocd, "ResetOpenOcd")
     
def exists(env):
     return True
	# return env.Detect("openocd") Maybe change this to detect ozone