# Copyright (c) 2024-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
import json
import subprocess
from datetime import datetime

from SCons.Script import *

FLEET_API_ENDPOINT = "https://fleet.aruw.org/fleet-api/"

# should be present in .gitignore
CACHE_FILE_PATH = "./build_tools/build_target_ip_cache.json"


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
            project_content = project_content.replace(
                "${OZONE_CONNECTION}", f'Project.SetHostIF ("IP", "{ip}");'
            )

        def use_usb():
            nonlocal project_content
            print(f"Using USB connection...")
            project_content = project_content.replace("${OZONE_CONNECTION}", f"Project.SetHostIF (\"USB\", \"\");")

        def load_ip_cache():
            try:
                with open(CACHE_FILE_PATH, "r") as f:
                    return json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                return {}

        def save_ip_cache(cache_data):
            try:
                with open(CACHE_FILE_PATH, "w") as f:
                    json.dump(cache_data, f, indent=4)
            except Exception as e:
                print(f"Warning: Failed to save IP cache: {e}")

        def update_cache_from_api(fleet_status):
            cache = load_ip_cache()
            latest_ips = {}

            for pi in fleet_status:
                mcb_data = pi.get("mcbData", {})

                if not mcb_data:
                    continue

                target_name = mcb_data.get("buildTarget")
                date_str = mcb_data.get("lastUpdate")
                ip = pi.get("ip")

                if not target_name or not ip or not date_str:
                    continue

                try:
                    timestamp = datetime.strptime(date_str, "%Y-%m-%dT%H:%M:%S.%fZ")
                except (ValueError, TypeError):
                    continue

                if (
                    target_name not in latest_ips
                    or timestamp > latest_ips[target_name]["timestamp"]
                ):
                    latest_ips[target_name] = {"ip": ip, "timestamp": timestamp}

            updated = False
            for target_name, data in latest_ips.items():
                if cache.get(target_name) != data["ip"]:
                    cache[target_name] = data["ip"]
                    updated = True

            if updated:
                save_ip_cache(cache)

            return cache

        def fetch_robot_ip(robot_target):
            from requests import get

            if not robot_target:
                return None

            fleet_status = None
            try:
                print("Querying Fleet Status...")
                response = get(FLEET_API_ENDPOINT, timeout=6)
                response.raise_for_status()
                fleet_status = response.json().get("robotPis", [])
            except Exception as e:
                print(f"Unable to query Fleet Status: {e}")

            if fleet_status is not None:
                cache = update_cache_from_api(fleet_status)
                ip = cache.get(robot_target)
                if not ip:
                    print(
                        f"No Pis found in Fleet Status with matching MCB build target for '{robot_target}'"
                    )
                return ip
            else:
                print("Attempting to use locally cached IP...")
                cache = load_ip_cache()
                ip = cache.get(robot_target)
                if ip:
                    print(f"Found cached IP for '{robot_target}'.")
                    return ip
                else:
                    print(f"No cached IP found for '{robot_target}'.")
                    return None

        ip_arg = ARGUMENTS.get("ip") or GetOption("ip")
        usb_arg = "usb" in ARGUMENTS or GetOption("usb")

        if ip_arg:
            if usb_arg:
                print("Warning: Both IP and USB arguments provided, using IP.")
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

        project_content = project_content.replace("${BUILD_DIR}", env["BUILDPATH"])
        project_content = project_content.replace(
            "${BUILD_DIR_LOWER}", env["BUILDPATH"].lower()
        )

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
        AddOption(
            "--ip",
            dest="ip",
            type="string",
            nargs=1,
            action="store",
            help="Specify IP address for Ozone",
        )
        AddOption(
            "--usb",
            dest="usb",
            action="store_true",
            help="Use USB connection for Ozone",
        )
    except Exception:
        pass

    env.AddMethod(run_ozone, "RunOzoneConfig")
    env.AddMethod(generate_ozone, "GenerateOzoneConfig")

def exists(env):
    return True
