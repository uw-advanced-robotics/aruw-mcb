#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2018, Niklas Hauser
#
# This file is part of the modm project.
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
# -----------------------------------------------------------------------------

import os
from SCons.Script import *
import traceback
import jinja2
from io import open

def jinja2_template_action(target, source, env):
	filename = source[0].path
	loader = jinja2.Environment(loader = jinja2.FileSystemLoader(os.path.dirname(filename)))

	# enable '%%' as prefix for line statements
	loader.line_statement_prefix = '%%'
	loader.line_comment_prefix = '%#'

	try:
		template = loader.get_template(os.path.basename(filename))
	except Exception as e:
		traceback.print_exc()
		raise Exception('Failed to retrieve Template', e)

	output = template.render(env['substitutions'])
	open(target[0].path, 'w', encoding='utf-8').write(output)

def template_emitter(target, source, env):
	Depends(target, SCons.Node.Python.Value(env['substitutions']))
	return target, source

def generate(env, **kw):
	def template_string(target, source, env):
		return "{0}.----Jinja2---- {1}{4}\n" \
			   "{0}'---Template--> {2}{5}{3}" \
			   .format("\033[;0;32m", "\033[;0;33m", "\033[;1;33m", "\033[;0;0m",
			           str(source[0]), str(target[0]))

	env.Append(
		BUILDERS = {
		'Jinja2Template':  env.Builder(
			action = env.Action(jinja2_template_action, template_string),
			emitter = template_emitter,
			src_suffix = '.in',
			single_source = True
		),
	})

def exists(env):
	return True
