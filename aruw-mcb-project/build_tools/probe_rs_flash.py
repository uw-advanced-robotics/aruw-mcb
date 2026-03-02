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

REMOTE_HELP_CHECKED = False
REMOTE_SUPPORTED = False


def _as_bool(value, default):
    if value is None:
        return default
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _probe_rs_common_args(remote):
    common_args = []

    chip = ARGUMENTS.get("chip", "STM32F427II")
    protocol = ARGUMENTS.get("protocol", "swd")
    speed_khz = ARGUMENTS.get("speed", "4000")
    probe = ARGUMENTS.get("probe", "")
    connect_under_reset = _as_bool(ARGUMENTS.get("connect_under_reset", "true"), True)
    non_interactive = _as_bool(ARGUMENTS.get("non_interactive", "true"), True)

    if chip:
        common_args += ["--chip", chip]
    if protocol:
        common_args += ["--protocol", protocol]
    if speed_khz:
        common_args += ["--speed", str(speed_khz)]
    if probe:
        common_args += ["--probe", probe]
    if connect_under_reset:
        common_args += ["--connect-under-reset"]
    if non_interactive:
        common_args += ["--non-interactive"]

    if remote:
        ip = ARGUMENTS.get("ip", "")
        host = ARGUMENTS.get("host", f"ws://{ip}:3000")
        token = ARGUMENTS.get("token", os.environ.get("PROBE_RS_TOKEN", "aruw"))

        common_args += ["--host", host, "--token", token]

    return common_args


def _check_remote_support(probe_rs):
    global REMOTE_HELP_CHECKED
    global REMOTE_SUPPORTED

    if REMOTE_HELP_CHECKED:
        return REMOTE_SUPPORTED

    REMOTE_HELP_CHECKED = True
    help_proc = subprocess.run(
        [probe_rs, "download", "--help"], capture_output=True, text=True, check=False
    )
    help_text = (help_proc.stdout or "") + (help_proc.stderr or "")
    REMOTE_SUPPORTED = ("--host" in help_text) and ("--token" in help_text)
    return REMOTE_SUPPORTED


def probe_rs_flash(env, source):
    def call_probe_rs_flash(target, source, env):
        probe_rs = shutil.which("probe-rs")
        if probe_rs is None:
            raise Exception(
                "probe-rs is not installed or not on PATH. "
                "Install probe-rs before running `scons flash`."
            )

        elf_path = source[0].abspath
        remote = ARGUMENTS.get("ip", "") != ""

        if remote and not _check_remote_support(probe_rs):
            raise Exception(
                "This probe-rs build does not support remote hosts (--host/--token). "
                "Install probe-rs with remote support on this machine."
            )

        common_args = _probe_rs_common_args(remote)
        verify = _as_bool(ARGUMENTS.get("verify", "false"), False)

        download_cmd = [probe_rs, "download"] + common_args
        if verify:
            download_cmd.append("--verify")
        download_cmd.append(elf_path)

        reset_cmd = [probe_rs, "reset"] + common_args

        mode = "remote" if remote else "local"
        print(f"Flashing via probe-rs ({mode}): {elf_path}")
        subprocess.run(download_cmd, check=True)
        subprocess.run(reset_cmd, check=True)

    action = Action(call_probe_rs_flash, cmdstr="Flashing with probe-rs...")
    return env.Command("probe-rs-flash", source, action)


def generate(env, **kw):
    env.AddMethod(probe_rs_flash, "ProgramProbeRsFlash")


def exists(env):
    return True
