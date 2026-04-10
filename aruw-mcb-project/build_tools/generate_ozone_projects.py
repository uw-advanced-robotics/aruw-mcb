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

        def use_ip(ip):
            nonlocal project_content
            print(f"Using IP ({ip}) connection...")
            project_content = project_content.replace("${OZONE_CONNECTION}", f"Project.SetHostIF (\"IP\", \"{ip}\");")

        def use_usb():
            nonlocal project_content
            print(f"Using USB connection...")
            project_content = project_content.replace("${OZONE_CONNECTION}", f"Project.SetHostIF (\"USB\", \"\");")

        def fetch_robot_ip(robot_target):
            if not robot_target:
                return None
                
            try:
                fleet_status = get(FLEET_API_ENDPOINT, timeout=6).json()
                fleet_status = fleet_status.get("robotPis", [])
            except Exception as e:
                print(f"Unable to query Fleet API: {e}")
                return None
                
            def get_last_update(pi):
                try:
                    date_str = pi.get("mcbData", {}).get("lastUpdate")
                    return datetime.strptime(date_str, "%Y-%m-%dT%H:%M:%S.%fZ")
                except (ValueError, TypeError):
                    return datetime.min

            pi_candidates = [
                pi for pi in fleet_status
                if pi.get("mcbData", {}).get("buildTarget") == robot_target and pi.get("mcbData", {}).get("lastUpdate")
            ]
            
            if not pi_candidates:
                print(f"No Pis found with matching MCB build target for '{robot_target}'")
                return None

            pi_match = max(pi_candidates, key=get_last_update)
            return pi_match.get("ip")

        # Look for SCons variables (ip=x / usb=1) OR options (--ip=x / --usb)
        ip_arg = ARGUMENTS.get("ip") or GetOption("ip")
        usb_arg = "usb" in ARGUMENTS or GetOption("usb")

        if ip_arg:
            use_ip(ip_arg)
        elif usb_arg:
            use_usb()
        else:
            print("No connection argument provided, attempting to fetch IP...")
            fetched_ip = fetch_robot_ip(robot)
            if fetched_ip:
                use_ip(fetched_ip)
            else:
                print("Fetching IP failed. Falling back to USB.")
                use_usb()

        project_content = project_content.replace("${BUILD_DIR}", env['BUILDPATH'])
        project_content = project_content.replace("${BUILD_DIR_LOWER}", env['BUILDPATH'].lower())

        target.append(env.File(project_file_path))
        target.append(env.File(f"{project_file_path}.user"))

        with open(f"{project_file_path}", "w+") as w:
            w.write(project_content)

        with open("./build_tools/example_ozone_project/example.jdebug.user") as r:
            project_user_content = r.read()

        with open(f"{project_file_path}.user", "w+") as w:
            w.write(project_user_content)

    action = Action(call_generate_ozone, cmdstr="Generating Ozone config...")
    return env.AlwaysBuild(env.Alias("ozone_generate", '', action))

def generate(env, **kw):
    try:
        AddOption('--ip', dest='ip', type='string', nargs=1, action='store', help='Specify IP address for Ozone')
        AddOption('--usb', dest='usb', action='store_true', help='Use USB connection for Ozone')
    except Exception:
        pass

    env.AddMethod(run_ozone, "RunOzoneConfig")
    env.AddMethod(generate_ozone, "GenerateOzoneConfig")

def exists(env):
    return True