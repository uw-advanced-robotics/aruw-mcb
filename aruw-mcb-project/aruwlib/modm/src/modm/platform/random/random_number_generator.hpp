/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_RANDOM_NUMBER_GENERATOR_HPP
#define MODM_STM32_RANDOM_NUMBER_GENERATOR_HPP

#include <stdint.h>
#include "../device.hpp"
#include <modm/platform/clock/rcc.hpp>


namespace modm
{

namespace platform
{

/**
 * Random Number Generator
 *
 * @todo check error status
 *
 * @author	Fabian Greif
 * @ingroup	modm_platform_random
 */
class RandomNumberGenerator
{
public:
	static inline void
	enable()
	{
		Rcc::enable<Peripheral::Rng>();
		RNG->CR = RNG_CR_RNGEN;
	}

	/**
	 * Check if a new random number is ready
	 *
	 * @return	 `true` if a new random number has been generated
	 * 			and can be read via getValue()
	 */
	static inline bool
	isReady()
	{
		return (RNG->SR & RNG_SR_DRDY);
	}

	/**
	 * Read the generated 32-bit random number
	 *
	 * Once read isReady() will return `false` until a new number
	 * has be generated.
	 */
	static inline uint32_t
	getValue()
	{
		return RNG->DR;
	}
};

}

}

#endif	// MODM_STM32_RANDOM_NUMBER_GENERATOR_HPP