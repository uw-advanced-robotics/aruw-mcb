/*
 * Copyright (c) 2014-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UART_BAUDRATE_HPP
#define MODM_STM32_UART_BAUDRATE_HPP

#include "uart_base.hpp"
#include <modm/architecture/interface/peripheral.hpp>
#include <modm/math/algorithm/prescaler.hpp>
#include <cmath>

namespace modm
{

namespace platform
{

/**
 * This class provides several helper functions to calculate the values for
 * the fractional baudrate generator.
 *
 * All of these functions are `constexpr` functions which allow compile time
 * evaluation.
 * If you really need to change the baudrate at runtime, consider calculating
 * the register values at compile time, storing them in your program, then
 * switching them at runtime.
 * This will save you a *lot* of flash and execution time.
 *
 * @author	Kevin Laeufer
 * @author	Niklas Hauser
 * @ingroup	modm_platform_uart
 */
class UartBaudrate : /** @cond */protected modm::PeripheralDriver /** @endcond */
{
public:
	/**
	 * Returns the highest Oversampling Mode that is possible for this configuration.
	 */
	static constexpr UartBase::OversamplingMode
	getOversamplingMode(frequency_t clockrate, baudrate_t baudrate)
	{
		return (baudrate <= clockrate / 16) ? UartBase::OversamplingMode::By16 : UartBase::OversamplingMode::By8;
	}
	/**
	 * Get the best BRR for chosen baudrate and tolerance.
	 *
	 * @tparam	clockrate
	 * 		the modules clock frequency in Hz
	 * @tparam	baudrate
	 * 		the desired baudrate in Hz
	 * @tparam	tolerance
	 * 		the allowed absolute tolerance for the resulting baudrate
	 */
	template<frequency_t clockrate, baudrate_t baudrate, percent_t tolerance=pct(10)>
	static uint16_t
	getBrr()
	{
		constexpr uint32_t scalar = (baudrate * 16l > clockrate) ? 8 : 16;
		constexpr uint32_t max = ((scalar == 16) ? (1ul << 16) : (1ul << 15)) - 1ul;
		constexpr auto result = Prescaler::from_range(clockrate, baudrate, 1, max);
		assertBaudrateInTolerance< result.frequency, baudrate, tolerance >();
		// When OVER8 = 0:, BRR[3:0] = USARTDIV[3:0].
		if (scalar == 16) return result.prescaler;
		// When OVER8 = 1:
		// BRR[15:4] = USARTDIV[15:4]
		// BRR[2:0] = USARTDIV[3:0] shifted 1 bit to the right. BRR[3] must be kept cleared.
		return (result.prescaler & ~0b1111) | ((result.prescaler & 0b1111) >> 1);
	}

};

}	// namespace platform

}	// namespace modm

#endif // MODM_STM32_UART_BAUDRATE_HPP