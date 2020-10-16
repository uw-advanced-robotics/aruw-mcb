/*
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2013-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include <modm/platform/core/hardware_init.hpp>

void
modm_gpio_enable(void)
{
	// Enable GPIO clock
	RCC->AHBENR  |=
		RCC_AHBENR_GPIOAEN |
		RCC_AHBENR_GPIOBEN |
		RCC_AHBENR_GPIOCEN |
		RCC_AHBENR_GPIODEN |
		RCC_AHBENR_GPIOFEN;
	// Reset GPIO peripheral
	RCC->AHBRSTR |=
		RCC_AHBRSTR_GPIOARST |
		RCC_AHBRSTR_GPIOBRST |
		RCC_AHBRSTR_GPIOCRST |
		RCC_AHBRSTR_GPIODRST |
		RCC_AHBRSTR_GPIOFRST;
	RCC->AHBRSTR &= ~(
		RCC_AHBRSTR_GPIOARST |
		RCC_AHBRSTR_GPIOBRST |
		RCC_AHBRSTR_GPIOCRST |
		RCC_AHBRSTR_GPIODRST |
		RCC_AHBRSTR_GPIOFRST);
}

MODM_HARDWARE_INIT_ORDER(modm_gpio_enable, 80);