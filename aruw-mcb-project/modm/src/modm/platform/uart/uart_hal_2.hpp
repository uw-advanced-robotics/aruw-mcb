/*
 * Copyright (c) 2013, Sascha Schade
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UARTHAL_2_HPP
#define MODM_STM32_UARTHAL_2_HPP

#include <stdint.h>
#include "../device.hpp"
#include "uart_base.hpp"
#include "uart_baudrate.hpp"

namespace modm
{

namespace platform
{

/**
 * Universal asynchronous receiver transmitter (UsartHal2)
 *
 * Not available on the low- and medium density devices.
 *
 * Very basic implementation that exposes more hardware features than
 * the regular Usart classes.
 *
 * @author		Kevin Laeufer
 * @ingroup		modm_platform_uart
 */
class UsartHal2 : public UartBase
{
private:
	/**
	 * Disable Parity or Enable Odd/Even Parity
	 *
	 * This method assumes 8 databits + 1 parity bit
	 * Remember to enable the clock but not the UART peripheral
	 * before setting the party.
	 */
	static inline void
	setParity(const Parity parity);

public:
	/// Enables the clock, resets the hardware and sets the UE bit
	static inline void
	enable();

	/// Disables the hw module (by disabling its clock line)
	static inline void
	disable();

	/**
	* Initialize Uart HAL Peripheral
	*
	* Enables clocks, the UART peripheral (but neither TX nor RX)
	* Sets baudrate and parity.
	*/
	template<	class SystemClock, baudrate_t baudrate >
	static void
	initialize(	Parity parity = Parity::Disabled);

	/**
	 * Initialize Uart HAL Peripheral
	 *
	 * Enables clocks, the UART peripheral (but neither TX nor RX)
	 * Sets baudrate and parity.
	 */
	template<	class SystemClock, baudrate_t baudrate,
				OversamplingMode oversample = OversamplingMode::By16 >
	static void
	initialize(	Parity parity = Parity::Disabled);
	/**
	* Initialize Uart HAL Peripheral
	*
	* Enables clocks, the UART peripheral (but neither TX nor RX)
	* Sets raw brr, parity and oversampling mode.
	*/
	static void
	initializeWithBrr(uint16_t brr,
			Parity parity,
			OversamplingMode oversample);

	/// Choose if you want to oversample by 16 (_default_) or by 8
	static inline void
	setOversamplingMode(OversamplingMode mode);
	// Methods needed to use this Usart Peripheral for SPI
	static inline void
	setSpiClock(SpiClock clk);

	static inline void
	setSpiDataMode(SpiDataMode mode);

	static inline void
	setLastBitClockPulse(LastBitClockPulse pulse);
	/**
	 * \brief	Write a single byte to the transmit register
	 *
	 * @warning 	This method does NOT do any sanity checks!!
	 *				It is your responsibility to check if the register
	 *				is empty!
	 */
	static inline void
	write(uint8_t data);

	/**
	 * Saves the value of the receive register to data
	 *
	 * @warning 	This method does NOT do any sanity checks!!
	 *				It is your responsibility to check if the register
	 *				contains something useful!
	 */
	static inline void
	read(uint8_t &data);

	/// Enable/Disable Transmitter
	static inline void
	setTransmitterEnable(const bool enable);

	/// Enable/Disable Receiver
	static inline void
	setReceiverEnable(bool enable);

	/// Set the UE (USART enable) bit
	static inline void
	enableOperation();

	/// Clear the UE (USART enable) bit
	static inline void
	disableOperation();

	/// Returns true if data has been received
	static inline bool
	isReceiveRegisterNotEmpty();

	/// Returns true if data can be written
	static inline bool
	isTransmitRegisterEmpty();

	static inline void
	enableInterruptVector(bool enable, uint32_t priority);

	static inline void
	enableInterrupt(Interrupt_t interrupt);

	static inline void
	disableInterrupt(Interrupt_t interrupt);

	static inline InterruptFlag_t
	getInterruptFlags();

	/**
	 * Returns the value of the receive register
	 *
	 * @warning 	Not all InterruptFlags can be cleared this way.
	 */
	static inline void
	acknowledgeInterruptFlags(InterruptFlag_t flags);
};

}	// namespace platform

}	// namespace modm

#include "uart_hal_2_impl.hpp"

#endif // MODM_STM32_UARTHAL_2_HPP