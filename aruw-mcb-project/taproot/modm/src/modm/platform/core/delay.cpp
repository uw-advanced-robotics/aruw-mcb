/*
 * Copyright (c) 2015-2016, 2021, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include "hardware_init.hpp"
#include "delay_impl.hpp"

void
modm::delay_us(uint32_t us)
{
	const uint32_t start = DWT->CYCCNT;
	asm inline ("" ::: "memory");
#ifdef MODM_DEBUG_BUILD
	unsigned int unshifted_cycles;
	modm_assert_continue_fail_debug(
		not __builtin_umul_overflow(platform::delay_fcpu_MHz, us, &unshifted_cycles),
		"delay.us", "modm::delay(us) can only delay ~1s! Use modm::delay(ms) for longer durations.");
#else
	const uint32_t unshifted_cycles = platform::delay_fcpu_MHz * us;
#endif
	const uint32_t cycles = unshifted_cycles >> platform::delay_fcpu_MHz_shift;
	while (true)
	{
		const uint32_t now = DWT->CYCCNT;
		if (now - start >= cycles) break;
	}
}

void
modm_dwt_enable(void)
{
	// Enable Tracing Debug Unit
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// Reset counter to 0
	DWT->CYCCNT = 0;
	// Enable CPU cycle counter
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
MODM_HARDWARE_INIT_ORDER(modm_dwt_enable, 100);