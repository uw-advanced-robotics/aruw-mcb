# -*- coding: utf-8 -*-
#
# Copyright (c) 2018-2019, Niklas Hauser
# Copyright (c) 2019, Raphael Lehmann
#
# This file is part of the modm project.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
# -----------------------------------------------------------------------------

from os.path import abspath, relpath

def build_target(env, sources):
	# Building application
	program = env.Program(target=env["CONFIG_PROJECT_NAME"]+".elf", source=sources)

	env.Alias("qtcreator", env.QtCreatorProject(sources))
	env.Alias("symbols", env.Symbols(program))
	env.Alias("listing", env.Listing(program))
	env.Alias("bin", env.Bin(program))
	env.Alias("build", program)
	# The executable depends on the linkerscript
	env.Depends(target=program, dependency="$BASEPATH/link/linkerscript.ld")
	env.Alias("size", env.Size(program))
	env.Alias("log-itm", env.LogItmOpenOcd())

	env.Alias("artifact", env.CacheArtifact(program))
	env.Alias("program-openocd", [env.ProgramOpenOcd(program)])
	env.Alias("program-remote", [env.ProgramGdbRemote(program)])
	env.Alias("program-bmp", [env.ProgramBMP(program)])
	env.Alias('program-dfu', [env.ProgramDFU(env.Bin(program))])
	env.Alias("debug-openocd", env.DebugOpenOcd(program))
	env.Alias("debug-remote", env.DebugGdbRemote(program))
	env.Alias("debug-bmp", env.DebugBMP(program))
	env.Alias("debug-coredump", env.DebugCoredump(program))

	env.Alias("reset-openocd", env.ResetOpenOcd())
	env.Alias("reset-bmp", env.ResetBMP())
	env.Alias("reset-remote", env.ResetGdbRemote())

	# Default to OpenOCD
	env.Alias("program", "program-openocd")
	env.Alias("reset", "reset-openocd")
	env.Alias("debug", "debug-openocd")

	env.Alias("all", ["build", "size"])
	env.Default("all")
	return program


def generate(env, **kw):
	env.AddMethod(build_target, "BuildTarget")

def exists(env):
	return True