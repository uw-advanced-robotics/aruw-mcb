/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_PERIPHERALS_HPP
#define MODM_STM32_PERIPHERALS_HPP

namespace modm::platform
{

enum class
Peripheral
{
	BitBang,
	Adc,
	Can,
	Comp1,
	Comp2,
	Crc,
	Dac,
	Dma1,
	Flash,
	HdmiCec,
	I2c1,
	I2c2,
	I2s1,
	I2s2,
	Irtim,
	Iwdg,
	None,
	Rcc,
	Rtc,
	Spi1,
	Spi2,
	Sys,
	Tim1,
	Tim14,
	Tim15,
	Tim16,
	Tim17,
	Tim2,
	Tim3,
	Tim6,
	Tim7,
	Tsc,
	Usart1,
	Usart2,
	Usart3,
	Usart4,
	Usb,
	Wwdg,
	Syscfg = Sys,
};

}

#endif // MODM_STM32_PERIPHERALS_HPP