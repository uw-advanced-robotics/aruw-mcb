/*
 * Copyright (c) 2015-2016, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include <modm/platform/clock/common.hpp>
#include "hardware_init.hpp"




extern "C"
{

void modm_fastcode
_delay_ns(uint16_t ns)
{
	// ns_per_loop = nanoseconds per cycle times cycles per loop (4 cycles)
	asm volatile (
		".syntax unified"       "\n\t"
		"muls.n	%2, %2, %1"     "\n\t"  // multiply the overhead cycles with the ns per cycle:  1-2 cycles on cm3, up to 32 cycles on cm0
		"subs.n	%0, %0, %2"     "\n\t"  // subtract the overhead in ns from the input:          1 cycle
	"1:  subs.n	%0, %0, %1"     "\n\t"  // subtract the ns per loop from the input:             1 cycle
		"bpl.n	1b"             "\n\t"  // keep doing that while result is still positive:      2 cycles (when taken)
	:: "r" (ns), "r" (modm::clock::ns_per_loop), "r" (7));
	// => loop is 4 cycles long
}

void modm_fastcode
_delay_us(uint16_t us)
{
	if (!us) return;    // 1 cycle, or 2 when taken

	asm volatile (
		".syntax unified"       "\n\t"
		"muls.n	%0, %0, %1"     "\n\t"  // get number of cycles by us * fcpu_MHz:               1-2 cycles on cm3, up to 32 cycles on cm0
	"1:  subs.n	%0, %0, #4"     "\n\t"  // subtract the loop cycles from the input:             1 cycle
		"bpl.n	1b"             "\n\t"  // keep doing that while result is still positive:      2 cycles (when taken)
	:: "r" (us), "r" (modm::clock::fcpu_MHz));
}

void modm_fastcode
_delay_ms(uint16_t ms)
{
	if (!ms) return;    // 1 cycle, or 2 when taken

	asm volatile (
		".syntax unified"       "\n\t"
		"muls.n	%0, %0, %1"     "\n\t"  // get number of cycles by ms * fcpu_kHz:               1-2 cycles on cm3, up to 32 cycles on cm0
	"1:  subs.n	%0, %0, #4"     "\n\t"  // subtract the loop cycles from the input:             1 cycle
		"bpl.n	1b"             "\n\t"  // keep doing that while result is still positive:      2 cycles (when taken)
	:: "r" (ms), "r" (modm::clock::fcpu_kHz));
}

}

