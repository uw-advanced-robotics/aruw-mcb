/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2011, 2017, Fabian Greif
 * Copyright (c) 2010, Georgi Grinshpun
 * Copyright (c) 2012, 2014, Sascha Schade
 * Copyright (c) 2012, 2014-2017, Niklas Hauser
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

/// @cond
#include <stdint.h>
#include <modm/architecture/utils.hpp>

#define MODM_DELAY_NS_IS_ACCURATE 0

extern "C" {
#include <unistd.h>
}

namespace modm
{

inline void delay_ns(uint32_t ns) { usleep(ns / 1000ul); }
inline void delay_us(uint32_t us) { usleep(us); }
inline void delay_ms(uint32_t ms) { usleep(ms * 1000ul); }

template< class Rep >
void
delay(std::chrono::duration<Rep, std::nano> ns_)
{
	const auto ns{std::chrono::duration_cast<std::chrono::nanoseconds>(ns_)};
	delay_ns(ns.count());
}

template< class Rep >
void
delay(std::chrono::duration<Rep, std::micro> us_)
{
	const auto us{std::chrono::duration_cast<std::chrono::microseconds>(us_)};
	delay_us(us.count());
}

}	// namespace modm
/// @endcond