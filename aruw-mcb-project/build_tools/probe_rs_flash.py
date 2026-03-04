# Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
import shutil
import subprocess

from SCons.Script import *

def _as_bool(value, default):
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _resolve_probe_rs_binary():
    probe_rs_override = ARGUMENTS.get("probe_rs", "")
    if probe_rs_override:
        if os.path.sep in probe_rs_override:
            return probe_rs_override
        resolved = shutil.which(probe_rs_override)
        return resolved if resolved is not None else probe_rs_override

    return shutil.which("probe-rs")


def _probe_rs_common_args():
    common_args = []

    chip = ARGUMENTS.get("chip", "STM32F427II")
    protocol = ARGUMENTS.get("protocol", "swd")
    speed_khz = ARGUMENTS.get("speed", "4000")
    probe = ARGUMENTS.get("probe", "")
    non_interactive = _as_bool(ARGUMENTS.get("non_interactive", "true"), True)

    if chip:
        common_args += ["--chip", chip]
    if protocol:
        common_args += ["--protocol", protocol]
    if speed_khz:
        common_args += ["--speed", str(speed_khz)]
    if probe:
        common_args += ["--probe", probe]
    if non_interactive:
        common_args += ["--non-interactive"]

    return common_args


def probe_rs_flash(env, source):
    def call_probe_rs_flash(target, source, env):
        probe_rs = _resolve_probe_rs_binary()
        if probe_rs is None:
            raise Exception(
                "probe-rs is not installed or not on PATH. "
                "Install probe-rs before running `scons flash`."
            )

        elf_path = source[0].abspath
        common_args = _probe_rs_common_args()
        verify = _as_bool(ARGUMENTS.get("verify", "false"), False)

        download_cmd = [probe_rs, "download"] + common_args
        if verify:
            download_cmd.append("--verify")
        download_cmd.append(elf_path)

        reset_cmd = [probe_rs, "reset"] + common_args

        print(f"Flashing via probe-rs (local): {elf_path}")
        subprocess.run(download_cmd, check=True)
        subprocess.run(reset_cmd, check=True)

    action = Action(call_probe_rs_flash, cmdstr="Flashing with probe-rs...")
    return env.Command("probe-rs-flash", source, action)


def generate(env, **kw):
    env.AddMethod(probe_rs_flash, "ProgramProbeRsFlash")


def exists(env):
    return True
