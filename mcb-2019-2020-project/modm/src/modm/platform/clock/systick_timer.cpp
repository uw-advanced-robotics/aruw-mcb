/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2014-2019, Niklas Hauser
 * Copyright (c) 2017, Fabian Greif
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/architecture/interface/atomic_lock.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <modm/platform/device.hpp>

#include "systick_timer.hpp"

static modm::platform::InterruptHandler sysTickHandler(nullptr);

extern "C" void
SysTick_Handler(void)
{
	modm::Clock::increment(1);
	if (sysTickHandler) sysTickHandler();
}

// ----------------------------------------------------------------------------
void
modm::platform::SysTickTimer::enable(uint32_t reload)
{
	SysTick->LOAD = reload;
	// Lower systick interrupt priority to lowest level
	NVIC_SetPriority(SysTick_IRQn, 0xf);

	SysTick->VAL = 0;
	SysTick->CTRL =
			SysTick_CTRL_CLKSOURCE_Msk |
			SysTick_CTRL_ENABLE_Msk |
			SysTick_CTRL_TICKINT_Msk;
}

void
modm::platform::SysTickTimer::disable()
{
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
}

// ----------------------------------------------------------------------------
void
modm::platform::SysTickTimer::attachInterruptHandler(InterruptHandler handler)
{
	atomic::Lock lock;
	sysTickHandler = handler;
}

void
modm::platform::SysTickTimer::detachInterruptHandler()
{
	atomic::Lock lock;
	sysTickHandler = nullptr;
}