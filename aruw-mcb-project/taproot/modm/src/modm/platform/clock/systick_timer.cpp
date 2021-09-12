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
#include <modm/platform/device.hpp>

#include "systick_timer.hpp"

static constexpr auto systick_step(250);
static volatile uint32_t milli_time{0};
static volatile uint32_t micro_time{0};
static volatile uint8_t interrupt{false};

extern "C" void
SysTick_Handler(void)
{
	milli_time += systick_step;
	micro_time += systick_step * 1'000ul;
	interrupt = true;
}

// pick the lowest possible priority for the SysTick interrupt
static constexpr uint8_t systick_priority{(1ul << __NVIC_PRIO_BITS) - 1ul};

// ----------------------------------------------------------------------------
void
modm::platform::SysTickTimer::enable(uint32_t modm_unused reload, bool modm_unused prescaler8)
{
	// Lower systick interrupt priority to lowest level
	NVIC_SetPriority(SysTick_IRQn, systick_priority);

	SysTick->LOAD = reload;
	SysTick->VAL  = reload;
	if (prescaler8) {
		SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
	} else {
		SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_CLKSOURCE_Msk;
	}
}

void
modm::platform::SysTickTimer::disable()
{
	SysTick->CTRL = 0;
}

// ----------------------------------------------------------------------------
modm::chrono::milli_clock::time_point modm_weak
modm::chrono::milli_clock::now() noexcept
{
	uint32_t val;
	uint32_t ms;
	do	// We cannot use an atomic lock here, the counter still overflows even
	{	// if the interrupt hasn't happened yet.
		interrupt = false;
		val = SysTick->VAL;
		ms = milli_time;
	}
	while(interrupt);
	const auto diff = SysTick->LOAD - val;
	const auto ms_per_Ncycles = platform::SysTickTimer::ms_per_Ncycles;
	constexpr auto Ncycles = platform::SysTickTimer::Ncycles;

	ms += (uint64_t(diff) * uint64_t(ms_per_Ncycles)) >> Ncycles;
	return time_point{duration{ms}};
}

modm::chrono::micro_clock::time_point modm_weak
modm::chrono::micro_clock::now() noexcept
{
	uint32_t val;
	uint32_t us;
	do	// We cannot use an atomic lock here, the counter still overflows even
	{	// if the interrupt hasn't happened yet.
		interrupt = false;
		val = SysTick->VAL;
		us = micro_time;
	}
	while(interrupt);
	const auto diff = SysTick->LOAD - val;
	const auto us_per_Ncycles = platform::SysTickTimer::us_per_Ncycles;
	constexpr auto Ncycles = platform::SysTickTimer::Ncycles;

	// use a 32x32=64bit multiplication
	us += (uint64_t(diff) * uint64_t(us_per_Ncycles)) >> Ncycles;
	return time_point{duration{us}};
}