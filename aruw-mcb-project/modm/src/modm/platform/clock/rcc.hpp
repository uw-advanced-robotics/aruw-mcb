/*
 * Copyright (c) 2012, 2016, Sascha Schade
 * Copyright (c) 2012, 2017, Fabian Greif
 * Copyright (c) 2012, 2014-2017, Niklas Hauser
 * Copyright (c) 2013-2014, Kevin Läufer
 * Copyright (c) 2018, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_RCC_HPP
#define MODM_STM32_RCC_HPP

#include <stdint.h>
#include "../device.hpp"
#include <modm/platform/core/peripherals.hpp>

namespace modm::platform
{

/**
 * Reset and Clock Control for STM32 devices.
 *
 * This class abstracts access to clock settings on the STM32.
 * You need to use this class to enable internal and external clock
 * sources & outputs, set PLL parameters and AHB & APB prescalers.
 * Don't forget to set the flash latencies.
 *
 * @author		Niklas Hauser
 * @ingroup		modm_platform_rcc
 */
class Rcc
{
public:
	enum class
	PllSource : uint32_t
	{
		/// High speed internal clock (8 MHz)
		Hsi = RCC_CFGR_PLLSRC_HSI_DIV2,
		/// High speed external clock (see HseConfig)
		Hse = RCC_CFGR_PLLSRC_HSE_PREDIV,
		/// High speed internal clock (48 MHz)
		Hsi48 = RCC_CFGR_PLLSRC_HSI48_PREDIV,
		InternalClockMHz48 = Hsi48,
		InternalClock = Hsi,
		ExternalClock = Hse,
		ExternalCrystal = Hse,
	};

	enum class
	SystemClockSource : uint32_t
	{
		Hsi = RCC_CFGR_SW_HSI,
		Hse = RCC_CFGR_SW_HSE,
		Hsi48 = RCC_CFGR_SW_HSI48,
		InternalClockMHz48 = Hsi48,
		InternalClock = Hsi,
		ExternalClock = Hse,
		ExternalCrystal = Hse,
		Pll = RCC_CFGR_SW_PLL,
	};

	enum class
	RealTimeClockSource : uint32_t
	{
		Lsi = RCC_BDCR_RTCSEL_1,
		Lse = RCC_BDCR_RTCSEL_0,
		Hse = RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCSEL_1,

		ExternalClock = Hse,
		ExternalCrystal = Hse,
		LowSpeedInternalClock = Lsi,
		LowSpeedExternalClock = Lse,
		LowSpeedExternalCrystal = Lse
	};

	enum class
	WatchdogClockSource : uint32_t
	{
		LowSpeedInternalClock = 0
	};

	enum class
	AhbPrescaler : uint32_t
	{
		Div1   = RCC_CFGR_HPRE_DIV1,
		Div2   = RCC_CFGR_HPRE_DIV2,
		Div4   = RCC_CFGR_HPRE_DIV4,
		Div8   = RCC_CFGR_HPRE_DIV8,
		Div16  = RCC_CFGR_HPRE_DIV16,
		Div64  = RCC_CFGR_HPRE_DIV64,
		Div128 = RCC_CFGR_HPRE_DIV128,
		Div256 = RCC_CFGR_HPRE_DIV256,
		Div512 = RCC_CFGR_HPRE_DIV512
	};

	enum class
	ApbPrescaler : uint32_t
	{
		Div1   = RCC_CFGR_PPRE_DIV1,
		Div2   = RCC_CFGR_PPRE_DIV2,
		Div4   = RCC_CFGR_PPRE_DIV4,
		Div8   = RCC_CFGR_PPRE_DIV8,
		Div16  = RCC_CFGR_PPRE_DIV16
	};
	enum class
	ClockOutputSource : uint32_t
	{
		SystemClock   = RCC_CFGR_MCO_SYSCLK,
		InternalClock = RCC_CFGR_MCO_HSI,
		ExternalClock = RCC_CFGR_MCO_HSE,
		ExternalCrystal = RCC_CFGR_MCO_HSE,
		InternalClockMHz14 = RCC_CFGR_MCO_HSI14,
		InternalClockMHz48 = RCC_CFGR_MCO_HSI48,
		Pll = RCC_CFGR_MCO_PLL,	///< divided by 2
	};
public:
	// sources
	static bool
	enableInternalClock(uint32_t waitCycles = 2048);

	static bool
	enableInternalClockMHz14(uint32_t waitCycles = 2048);
	static bool
	enableInternalClockMHz48(uint32_t waitCycles = 2048);
	static bool
	enableExternalClock(uint32_t waitCycles = 2048);

	static bool
	enableExternalCrystal(uint32_t waitCycles = 2048);

	static bool
	enableLowSpeedInternalClock(uint32_t waitCycles = 2048);

	static bool
	enableLowSpeedExternalClock(uint32_t waitCycles = 2048);

	static bool
	enableLowSpeedExternalCrystal(uint32_t waitCycles = 2048);

	// plls
	static bool
	enablePll(PllSource source,
	          uint8_t pllMul,
				uint8_t pllPrediv,
				uint32_t waitCycles = 2048);
	// sinks
	static bool
	enableSystemClock(SystemClockSource src, uint32_t waitCycles = 2048);

	static inline bool
	enableRealTimeClock(RealTimeClockSource src)
	{
		RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_RTCSEL) | RCC_BDCR_RTCEN | uint32_t(src);
		return true;
	}

	static inline bool
	enableWatchdogClock(WatchdogClockSource /*src*/)
	{ return true; }

	static inline bool
	enableClockOutput(ClockOutputSource src)
	{
		RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_MCO)) | uint32_t(src);
		return true;
	}
public:
	static inline bool
	setAhbPrescaler(AhbPrescaler prescaler)
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | uint32_t(prescaler);
		return true;
	}

	static inline bool
	setApbPrescaler(ApbPrescaler prescaler)
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE) | uint32_t(prescaler);
		return true;
	}
public:
	/** Set flash latency for CPU frequency and voltage.
	 * Does nothing if CPU frequency is too high for the available
	 * voltage.
	 *
	 * @returns maximum CPU frequency for voltage.
	 * @retval	<=CPU_Frequency flash latency has been set correctly.
	 * @retval	>CPU_Frequency requested frequency too high for voltage.
	 */
	template< uint32_t Core_Hz, uint16_t Core_mV = 3300>
	static uint32_t
	setFlashLatency();

	template< uint32_t Core_Hz >
	static void
	updateCoreFrequency();

public:
	template< Peripheral peripheral >
	static void
	enable();

	template< Peripheral peripheral >
	static bool
	isEnabled();

	template< Peripheral peripheral >
	static void
	disable();

private:
	struct flash_latency
	{
		uint32_t latency;
		uint32_t max_frequency;
	};
	static constexpr flash_latency
	computeFlashLatency(uint32_t Core_Hz, uint16_t Core_mV);
};

using ClockControl [[deprecated("Please use `modm::platform:Rcc` instead")]] = Rcc;

}   // namespace modm::platform


#include "rcc_impl.hpp"

#endif	//  MODM_STM32_RCC_HPP