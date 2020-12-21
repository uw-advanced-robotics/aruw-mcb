// coding: utf-8
/*
 * Copyright (c) 2017, 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <modm/architecture/utils.hpp>

/// @cond
#define MODM_ASSERTION_INFO_HAS_DESCRIPTION 0

// C-type information struct
typedef struct modm_packed
{
	const char *name;
	uintptr_t context;
	uint8_t behavior;
} _modm_assertion_info;

// modm_assert* helper macros

#define _modm_assert_ifss(s) ((const char *)(s))
#define _modm_assert5(behavior, condition, name, description, context) \
	({ \
		const bool evaluated_condition = (bool) (condition); \
		if (!evaluated_condition) { \
			_modm_assertion_info info = { \
				_modm_assert_ifss(name), \
				 (uintptr_t)(context), (uint8_t)(behavior)}; \
			modm_assert_report(&info); \
			if (behavior & 4) __builtin_unreachable(); \
		} \
		evaluated_condition; \
	})
#define _modm_assert4(behavior, condition, name, description) \
	_modm_assert5(behavior, condition, name, description, -1)

#define _modm_assert_get_macro(_1,_2,_3,_4,_modm_assertN,...) _modm_assertN


// modm_assert* definitions
#define modm_assert(...) \
	_modm_assert_get_macro(__VA_ARGS__, _modm_assert5, _modm_assert4)(0x04, __VA_ARGS__)
#define modm_assert_continue_ignore(...) \
	_modm_assert_get_macro(__VA_ARGS__, _modm_assert5, _modm_assert4)(0x02, __VA_ARGS__)
#define modm_assert_continue_fail(...) \
	_modm_assert_get_macro(__VA_ARGS__, _modm_assert5, _modm_assert4)(0x01, __VA_ARGS__)

#ifdef MODM_DEBUG_BUILD

#define modm_assert_continue_ignore_debug(...) \
	_modm_assert_get_macro(__VA_ARGS__, _modm_assert5, _modm_assert4)(0x82, __VA_ARGS__)
#define modm_assert_continue_fail_debug(...) \
	_modm_assert_get_macro(__VA_ARGS__, _modm_assert5, _modm_assert4)(0x81, __VA_ARGS__)

#else

#define modm_assert_continue_ignore_debug(condition, ...) \
	({ bool modm_unused evaluated_condition = (bool)(condition); evaluated_condition; })
#define modm_assert_continue_fail_debug(condition, ...) \
	modm_assert_continue_ignore_debug(condition)

#endif

// required runtime implementation
modm_extern_c void
modm_assert_report(_modm_assertion_info *info);
/// @endcond