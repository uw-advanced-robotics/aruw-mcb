/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Georgi Grinshpun
 * Copyright (c) 2012, 2014, Sascha Schade
 * Copyright (c) 2012, 2014-2016, Niklas Hauser
 * Copyright (c) 2014, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#include <chrono>
#include <modm/architecture/interface/assert.hpp>
/// @cond
#define MODM_DELAY_NS_IS_ACCURATE 1

namespace modm
{
namespace platform
{
extern uint16_t delay_ns_per_loop;
extern uint16_t delay_fcpu_MHz;
}

inline void delay_ns(uint32_t ns)
{
	// ns_per_loop = nanoseconds per cycle times cycles per loop (3 cycles)
	asm volatile (
		".syntax unified"       "\n\t"
		"muls.n	%2, %2, %1"     "\n\t"  // multiply the overhead cycles with the ns per cycle:  1-2 cycles on cm3, up to 32 cycles on cm0
		"subs.n	%0, %0, %2"     "\n\t"  // subtract the overhead in ns from the input:          1 cycle
	"1:  subs.n	%0, %0, %1"     "\n\t"  // subtract the ns per loop from the input:             1 cycle
		"bpl.n	1b"             "\n\t"  // keep doing that while result is still positive:      2 cycles (when taken)
	:: "r" (ns), "r" (platform::delay_ns_per_loop), "r" (8));
	// => loop is 3 cycles long
}
void delay_us(uint32_t us);
inline void delay_ms(uint32_t ms) { delay_us(ms * 1000); }

// ----------------------------------------------------------------------------
template< class Rep >
void
delay(std::chrono::duration<Rep, std::nano> ns_)
{
	const auto ns{std::chrono::duration_cast<std::chrono::nanoseconds>(ns_)};
	modm_assert_continue_fail_debug(0 <= ns.count() and ns.count() <= 1'000'000,
		"delay.ns", "modm::delay(ns) can only delay a (positive) maximum of ~1 millisecond!");
	delay_ns(ns.count());
}

template< class Rep >
void
delay(std::chrono::duration<Rep, std::micro> us_)
{
	const auto us{std::chrono::duration_cast<std::chrono::microseconds>(us_)};
	delay_us(us.count());
}

template< class Rep >
void
delay(std::chrono::duration<Rep, std::milli> ms)
{
	const auto us{std::chrono::duration_cast<std::chrono::microseconds>(ms)};
	delay_us(us.count());
}

}
/// @endcond