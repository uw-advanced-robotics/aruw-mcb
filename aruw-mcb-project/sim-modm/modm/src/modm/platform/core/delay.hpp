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

#ifndef	MODM_PLATFORM_DELAY_HPP
#define	MODM_PLATFORM_DELAY_HPP

/// @cond
#include <stdint.h>
#include <modm/architecture/utils.hpp>

extern "C" {
#include <unistd.h>
}

namespace modm
{

/// @cond
inline void
delayNanoseconds(uint16_t /*ns*/)
{
	usleep(1);
}

inline void
delayMicroseconds(uint16_t us)
{
	usleep(us);
}

inline void
delayMilliseconds(uint16_t ms)
{
	usleep(uint32_t(ms) * 1000);
}
/// @endcond

}	// namespace modm

/// @endcond
#endif	// MODM_PLATFORM_DELAY_HPP