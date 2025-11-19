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
import os
import subprocess
from requests import get
from datetime import datetime

from SCons.Script import *

FLEET_API_ENDPOINT = "https://fleet.aruw.org/fleet-api/"

def run_ozone(env, source, robot=""):
    def call_run_ozone(target, source, env):
        jdebug = f"{env['BUILDPATH']}/{env['CONFIG_PROJECT_NAME']}.jdebug"
        import sys
        if sys.platform == "win32":
            os.startfile(jdebug)
        elif sys.platform == "darwin":
            subprocess.call(['open', '-n', '-a', 'Ozone.app', '--args', jdebug])
        else:
            subprocess.call(['xdg-open', jdebug])

    action = Action(call_run_ozone, cmdstr="Launching Ozone...")
    return env.AlwaysBuild(env.Alias("ozone_run", [generate_ozone(env, robot), source], action))

def generate_ozone(env, robot=""):
    def call_generate_ozone(target, source, env):
        project_content = ""
        with open("./build_tools/example_ozone_project/example.jdebug") as r:
            project_content = r.read()

        project_file_path = f"{env['BUILDPATH']}/{env['CONFIG_PROJECT_NAME']}.jdebug"

        ip_arg = str(ARGUMENTS.get("ip", "")).lower()

        def use_ip(ip):
            nonlocal project_content
            print(f"Using IP({ip}) connection...")
            project_content = project_content.replace("${OZONE_CONNECTION}", f"Project.SetHostIF (\"IP\", \"{ip}\");")

        def use_usb():
            nonlocal project_content
            print(f"Using USB connection...")
            project_content = project_content.replace("${OZONE_CONNECTION}", f"Project.SetHostIF (\"USB\", \"\");")

        def fetch_robot_ip(robot):
            try:
                fleet_status = get(FLEET_API_ENDPOINT)
                fleet_status = fleet_status.json()
                fleet_status = fleet_status["robotPis"]
            except:
                print("Unable to query Fleet API")
                return None
                
            pi_candidates = [
                {
                    **pi, 
                    "mcbData": {
                        **data,  #
                        "lastUpdate": datetime.strptime(data["lastUpdate"], "%Y-%m-%dT%H:%M:%S.%fZ"),
                        "lastUpdate": datetime.strftime(mcbData["lastUpdate"], "%Y-%m-%dT%H:%M:%S.%fZ")
                    }
                }
                for pi in fleet_status
                if (data := pi.get("mcbData")) and data.get("buildTarget") == robot and data.get("lastUpdate")
            ]
            if not pi_candidates:
                print("No Pis with matching MCB build target")
                return None

            pi_match = max(pi_candidates, key=lambda p: p.get("lastUpdated"))
            return pi_match.get("ip")

        if not ip_arg and robot:
            ip = fetch_robot_ip(robot)
            use_ip(ip) if ip else use_usb()
        elif ip_arg == "usb":
            use_usb()
        else:
            use_ip(ip_arg)

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

    action = Action(call_generate_ozone, cmdstr="Generating Ozone config...")
    return env.AlwaysBuild(env.Alias("ozone_generate", '', action))

def generate(env, **kw):
    env.AddMethod(run_ozone, "RunOzoneConfig")
    env.AddMethod(generate_ozone, "GenerateOzoneConfig")

def exists(env):
    return True
