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

#include "common.hpp"

namespace modm::platform
{

constexpr Rcc::flash_latency
Rcc::computeFlashLatency(uint32_t Core_Hz, uint16_t Core_mV)
{
	constexpr uint32_t flash_latency_1800[] =
	{
		24000000,
		48000000,
	};
	const uint32_t *lut(flash_latency_1800);
	uint8_t lut_size(sizeof(flash_latency_1800) / sizeof(uint32_t));
	(void) Core_mV;
	// find the next highest frequency in the table
	uint8_t latency(0);
	uint32_t max_freq(0);
	while (latency < lut_size)
	{
		if (Core_Hz <= (max_freq = lut[latency]))
			break;
		latency++;
	}
	return {latency, max_freq};
}

template< uint32_t Core_Hz, uint16_t Core_mV = 3300 >
uint32_t
Rcc::setFlashLatency()
{
	constexpr flash_latency fl = computeFlashLatency(Core_Hz, Core_mV);
	static_assert(Core_Hz <= fl.max_frequency, "CPU Frequency is too high for this core voltage!");

	uint32_t acr = FLASH->ACR & ~FLASH_ACR_LATENCY;
	// set flash latency
	acr |= fl.latency;
	// enable flash prefetch
	acr |= FLASH_ACR_PRFTBE;
	FLASH->ACR = acr;
	return fl.max_frequency;
}

template< uint32_t Core_Hz >
void
Rcc::updateCoreFrequency()
{
	modm::clock::fcpu     = Core_Hz;
	modm::clock::fcpu_kHz = Core_Hz / 1'000;
	modm::clock::fcpu_MHz = Core_Hz / 1'000'000;
	modm::clock::ns_per_loop = ::round(4000.f / (Core_Hz / 1'000'000));
}

constexpr bool
rcc_check_enable(Peripheral peripheral)
{
	switch(peripheral) {
		case Peripheral::Adc:
		case Peripheral::Can:
		case Peripheral::Crc:
		case Peripheral::Dac:
		case Peripheral::Dma1:
		case Peripheral::I2c1:
		case Peripheral::I2c2:
		case Peripheral::Rtc:
		case Peripheral::Spi1:
		case Peripheral::Spi2:
		case Peripheral::Tim1:
		case Peripheral::Tim14:
		case Peripheral::Tim15:
		case Peripheral::Tim16:
		case Peripheral::Tim17:
		case Peripheral::Tim2:
		case Peripheral::Tim3:
		case Peripheral::Tim6:
		case Peripheral::Tim7:
		case Peripheral::Tsc:
		case Peripheral::Usart1:
		case Peripheral::Usart2:
		case Peripheral::Usart3:
		case Peripheral::Usart4:
		case Peripheral::Usb:
		case Peripheral::Wwdg:
			return true;
		default:
			return false;
	}
}

template< Peripheral peripheral >
void
Rcc::enable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::enable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Adc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_ADCEN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;
		}
	if constexpr (peripheral == Peripheral::Can)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_CANEN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_CANRST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_CANRST;
		}
	if constexpr (peripheral == Peripheral::Crc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHBENR |= RCC_AHBENR_CRCEN;
		}
	if constexpr (peripheral == Peripheral::Dac)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_DACEN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_DACRST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;
		}
	if constexpr (peripheral == Peripheral::Dma1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHBENR |= RCC_AHBENR_DMA1EN;
		}
	if constexpr (peripheral == Peripheral::I2c1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
		}
	if constexpr (peripheral == Peripheral::I2c2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_I2C2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
		}
	if constexpr (peripheral == Peripheral::Rtc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->BDCR |= RCC_BDCR_RTCEN;
		}
	if constexpr (peripheral == Peripheral::Spi1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
		}
	if constexpr (peripheral == Peripheral::Spi2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_SPI2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
		}
	if constexpr (peripheral == Peripheral::Tim1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM1RST;
		}
	if constexpr (peripheral == Peripheral::Tim14)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM14EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM14RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM14RST;
		}
	if constexpr (peripheral == Peripheral::Tim15)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM15RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM15RST;
		}
	if constexpr (peripheral == Peripheral::Tim16)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM16EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM16RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM16RST;
		}
	if constexpr (peripheral == Peripheral::Tim17)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_TIM17EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_TIM17RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM17RST;
		}
	if constexpr (peripheral == Peripheral::Tim2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
		}
	if constexpr (peripheral == Peripheral::Tim3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM3RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM3RST;
		}
	if constexpr (peripheral == Peripheral::Tim6)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM6RST;
		}
	if constexpr (peripheral == Peripheral::Tim7)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_TIM7RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM7RST;
		}
	if constexpr (peripheral == Peripheral::Tsc)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->AHBENR |= RCC_AHBENR_TSCEN; __DSB();
			RCC->AHBRSTR |= RCC_AHBRSTR_TSCRST; __DSB();
			RCC->AHBRSTR &= ~RCC_AHBRSTR_TSCRST;
		}
	if constexpr (peripheral == Peripheral::Usart1)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB2ENR |= RCC_APB2ENR_USART1EN; __DSB();
			RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; __DSB();
			RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		}
	if constexpr (peripheral == Peripheral::Usart2)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_USART2EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_USART2RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
		}
	if constexpr (peripheral == Peripheral::Usart3)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_USART3EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_USART3RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
		}
	if constexpr (peripheral == Peripheral::Usart4)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_USART4EN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_USART4RST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USART4RST;
		}
	if constexpr (peripheral == Peripheral::Usb)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_USBEN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_USBRST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
		}
	if constexpr (peripheral == Peripheral::Wwdg)
		if (not Rcc::isEnabled<peripheral>()) {
			RCC->APB1ENR |= RCC_APB1ENR_WWDGEN; __DSB();
			RCC->APB1RSTR |= RCC_APB1RSTR_WWDGRST; __DSB();
			RCC->APB1RSTR &= ~RCC_APB1RSTR_WWDGRST;
		}
	__DSB();
}

template< Peripheral peripheral >
void
Rcc::disable()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::disable() doesn't know this peripheral!");

	__DSB();
	if constexpr (peripheral == Peripheral::Adc)
		RCC->APB2ENR &= ~RCC_APB2ENR_ADCEN;
	if constexpr (peripheral == Peripheral::Can)
		RCC->APB1ENR &= ~RCC_APB1ENR_CANEN;
	if constexpr (peripheral == Peripheral::Crc)
		RCC->AHBENR &= ~RCC_AHBENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac)
		RCC->APB1ENR &= ~RCC_APB1ENR_DACEN;
	if constexpr (peripheral == Peripheral::Dma1)
		RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;
	if constexpr (peripheral == Peripheral::I2c1)
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c2)
		RCC->APB1ENR &= ~RCC_APB1ENR_I2C2EN;
	if constexpr (peripheral == Peripheral::Rtc)
		RCC->BDCR &= ~RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Spi1)
		RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi2)
		RCC->APB1ENR &= ~RCC_APB1ENR_SPI2EN;
	if constexpr (peripheral == Peripheral::Tim1)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim14)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN;
	if constexpr (peripheral == Peripheral::Tim15)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM15EN;
	if constexpr (peripheral == Peripheral::Tim16)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM16EN;
	if constexpr (peripheral == Peripheral::Tim17)
		RCC->APB2ENR &= ~RCC_APB2ENR_TIM17EN;
	if constexpr (peripheral == Peripheral::Tim2)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim3)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM3EN;
	if constexpr (peripheral == Peripheral::Tim6)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		RCC->APB1ENR &= ~RCC_APB1ENR_TIM7EN;
	if constexpr (peripheral == Peripheral::Tsc)
		RCC->AHBENR &= ~RCC_AHBENR_TSCEN;
	if constexpr (peripheral == Peripheral::Usart1)
		RCC->APB2ENR &= ~RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	if constexpr (peripheral == Peripheral::Usart3)
		RCC->APB1ENR &= ~RCC_APB1ENR_USART3EN;
	if constexpr (peripheral == Peripheral::Usart4)
		RCC->APB1ENR &= ~RCC_APB1ENR_USART4EN;
	if constexpr (peripheral == Peripheral::Usb)
		RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
	if constexpr (peripheral == Peripheral::Wwdg)
		RCC->APB1ENR &= ~RCC_APB1ENR_WWDGEN;
	__DSB();
}

template< Peripheral peripheral >
bool
Rcc::isEnabled()
{
	static_assert(rcc_check_enable(peripheral),
		"Rcc::isEnabled() doesn't know this peripheral!");

	if constexpr (peripheral == Peripheral::Adc)
		return RCC->APB2ENR & RCC_APB2ENR_ADCEN;
	if constexpr (peripheral == Peripheral::Can)
		return RCC->APB1ENR & RCC_APB1ENR_CANEN;
	if constexpr (peripheral == Peripheral::Crc)
		return RCC->AHBENR & RCC_AHBENR_CRCEN;
	if constexpr (peripheral == Peripheral::Dac)
		return RCC->APB1ENR & RCC_APB1ENR_DACEN;
	if constexpr (peripheral == Peripheral::Dma1)
		return RCC->AHBENR & RCC_AHBENR_DMA1EN;
	if constexpr (peripheral == Peripheral::I2c1)
		return RCC->APB1ENR & RCC_APB1ENR_I2C1EN;
	if constexpr (peripheral == Peripheral::I2c2)
		return RCC->APB1ENR & RCC_APB1ENR_I2C2EN;
	if constexpr (peripheral == Peripheral::Rtc)
		return RCC->BDCR & RCC_BDCR_RTCEN;
	if constexpr (peripheral == Peripheral::Spi1)
		return RCC->APB2ENR & RCC_APB2ENR_SPI1EN;
	if constexpr (peripheral == Peripheral::Spi2)
		return RCC->APB1ENR & RCC_APB1ENR_SPI2EN;
	if constexpr (peripheral == Peripheral::Tim1)
		return RCC->APB2ENR & RCC_APB2ENR_TIM1EN;
	if constexpr (peripheral == Peripheral::Tim14)
		return RCC->APB1ENR & RCC_APB1ENR_TIM14EN;
	if constexpr (peripheral == Peripheral::Tim15)
		return RCC->APB2ENR & RCC_APB2ENR_TIM15EN;
	if constexpr (peripheral == Peripheral::Tim16)
		return RCC->APB2ENR & RCC_APB2ENR_TIM16EN;
	if constexpr (peripheral == Peripheral::Tim17)
		return RCC->APB2ENR & RCC_APB2ENR_TIM17EN;
	if constexpr (peripheral == Peripheral::Tim2)
		return RCC->APB1ENR & RCC_APB1ENR_TIM2EN;
	if constexpr (peripheral == Peripheral::Tim3)
		return RCC->APB1ENR & RCC_APB1ENR_TIM3EN;
	if constexpr (peripheral == Peripheral::Tim6)
		return RCC->APB1ENR & RCC_APB1ENR_TIM6EN;
	if constexpr (peripheral == Peripheral::Tim7)
		return RCC->APB1ENR & RCC_APB1ENR_TIM7EN;
	if constexpr (peripheral == Peripheral::Tsc)
		return RCC->AHBENR & RCC_AHBENR_TSCEN;
	if constexpr (peripheral == Peripheral::Usart1)
		return RCC->APB2ENR & RCC_APB2ENR_USART1EN;
	if constexpr (peripheral == Peripheral::Usart2)
		return RCC->APB1ENR & RCC_APB1ENR_USART2EN;
	if constexpr (peripheral == Peripheral::Usart3)
		return RCC->APB1ENR & RCC_APB1ENR_USART3EN;
	if constexpr (peripheral == Peripheral::Usart4)
		return RCC->APB1ENR & RCC_APB1ENR_USART4EN;
	if constexpr (peripheral == Peripheral::Usb)
		return RCC->APB1ENR & RCC_APB1ENR_USBEN;
	if constexpr (peripheral == Peripheral::Wwdg)
		return RCC->APB1ENR & RCC_APB1ENR_WWDGEN;
}

}   // namespace modm::platform
