/*
 * Copyright (c) 2017-2018, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_GPIO_PIN_C10_HPP
#define MODM_STM32_GPIO_PIN_C10_HPP

#include "../device.hpp"
#include "base.hpp"
#include "set.hpp"

namespace modm::platform
{

/// @cond
class GpioC10;
using GpioOutputC10 = GpioC10;
using GpioInputC10  = GpioC10;
/// @endcond

/// IO class for Pin C10
/// @ingroup	modm_platform_gpio
class GpioC10 : public Gpio, public ::modm::GpioIO
{
	template<class... Gpios>
	friend class GpioSet;
	using PinSet = GpioSet<GpioC10>;
	friend class Adc;
	friend class Adc1; friend class Adc2;
	friend class Adc3; friend class Adc4;
public:
	using Output = GpioC10;
	using Input = GpioC10;
	using IO = GpioC10;
	using Type = GpioC10;
	static constexpr bool isInverted = false;
	static constexpr Port port = Port::C; ///< Port name
	static constexpr uint8_t pin = 10; ///< Pin number
	static constexpr IRQn_Type ExternalInterruptIRQ = EXTI15_10_IRQn;

protected:
	/// Bitmask for registers that contain a 1bit value for every pin.
	static constexpr uint16_t mask  = 0x1 << pin;
	/// Bitmask for registers that contain a 2bit value for every pin.
	static constexpr uint32_t mask2 = 0x3 << (pin * 2);
	/// Port Number.
	static constexpr uint8_t port_nr = uint8_t(port);
	/// Alternate Function register id. 0 for pin 0-7. 1 for pin 8-15.
	static constexpr uint8_t af_id  = pin / 8;
	/// Alternate Function offset.
	static constexpr uint8_t af_offset = (pin * 4) % 32;
	/// Alternate Function register mask.
	static constexpr uint32_t af_mask  = 0xf << af_offset;

public:
	/// @cond
	inline static void setAlternateFunction(uint8_t af) {
		GPIOC->AFR[af_id] = (GPIOC->AFR[af_id] & ~af_mask) | ((af & 0xf) << af_offset);
		GPIOC->MODER = (GPIOC->MODER & ~mask2) | (i(Mode::AlternateFunction) << (pin * 2));
	}

	/// Enable Analog Mode which is needed to use this pin as an ADC input.
	inline static void setAnalogInput() { PinSet::setAnalogInput(); }
	/// @endcond

public:
	// GpioOutput
	// start documentation inherited
	inline static void setOutput() { PinSet::setOutput(); }
	inline static void setOutput(bool status) { PinSet::setOutput(status); }
	inline static void set() { PinSet::set(); }
	inline static void set(bool status) { PinSet::set(status); }
	inline static void reset() { PinSet::reset(); }
	inline static void toggle() {
		if (isSet()) { reset(); }
		else         { set();   }
	}
	inline static bool isSet() { return (GPIOC->ODR & mask); }
	// stop documentation inherited
	inline static void configure(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::configure(type, speed); }
	inline static void setOutput(OutputType type, OutputSpeed speed = OutputSpeed::MHz50) { PinSet::setOutput(type, speed); }
	// GpioInput
	// start documentation inherited
	inline static void setInput() { PinSet::setInput(); }
	inline static bool read() { return (GPIOC->IDR & mask); }
	// end documentation inherited
	inline static void configure(InputType type) { PinSet::configure(type); }
	inline static void setInput(InputType type) { PinSet::setInput(type); }
	// External Interrupts
	// Warning: This will disable any previously enabled interrupt which is
	// routed to the same interupt line, e.g. PA3 will disable PB3.
	// This is a hardware limitation by the STM32 EXTI.
	inline static void enableExternalInterrupt()
	{
		// PA[x], x =  0 ..  3 maps to EXTICR[0]
		// PA[x], x =  4 ..  7 maps to EXTICR[1]
		// PA[x], x =  8 .. 11 maps to EXTICR[2]
		// PA[x], x = 12 .. 15 maps to EXTICR[3]
		// => bit3 and bit2 (mask 0x0c) specify the register
		// => bit1 and bit0 (mask 0x03) specify the bit position
		constexpr uint8_t index   = (pin & 0b1100) >> 2;
		constexpr uint8_t bit_pos = (pin & 0b0011) << 2;
		constexpr uint16_t syscfg_mask = (0b1111) << bit_pos;
		constexpr uint16_t syscfg_value = (port_nr & (0b1111)) << bit_pos;
		SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~syscfg_mask) | syscfg_value;
		EXTI->IMR |= mask;
	}
	inline static void disableExternalInterrupt() { EXTI->IMR &= ~mask; }
	inline static void enableExternalInterruptVector(const uint32_t priority)
	{
		NVIC_SetPriority(ExternalInterruptIRQ, priority);
		NVIC_EnableIRQ(ExternalInterruptIRQ);
	}
	inline static void disableExternalInterruptVector() { NVIC_DisableIRQ(ExternalInterruptIRQ); }
	inline static void setInputTrigger(const InputTrigger trigger)
	{
		switch (trigger)
		{
		case InputTrigger::RisingEdge:
			EXTI->RTSR |=  mask;
			EXTI->FTSR &= ~mask;
			break;
		case InputTrigger::FallingEdge:
			EXTI->RTSR &= ~mask;
			EXTI->FTSR |=  mask;
			break;
		case InputTrigger::BothEdges:
			EXTI->RTSR |=  mask;
			EXTI->FTSR |=  mask;
			break;
		}
	}
	inline static bool getExternalInterruptFlag() { return (EXTI->PR & mask); }
	inline static void acknowledgeExternalInterruptFlag() { EXTI->PR = mask; }
	// GpioIO
	// start documentation inherited
	inline static Direction getDirection() {
		uint32_t mode = (GPIOC->MODER & mask2);
		if (mode == (i(Mode::Input) << pin * 2)) {
			return Direction::In;
		}
		if (mode == (i(Mode::Output) << pin * 2)) {
			return Direction::Out;
		}
		return Direction::Special;
	}
	// end documentation inherited
	inline static void lock() { PinSet::lock(); }
	inline static void disconnect() {
		setInput(InputType::Floating);
		GPIOC->AFR[af_id] &= ~af_mask;
	}

public:
#ifdef __DOXYGEN__
	/// @{
	/// Connect to any software peripheral
	using BitBang = GpioSignal;
	/// Connect to I2s3
	using Ck = GpioSignal;
	/// Connect to Sdio
	using D2 = GpioSignal;
	/// Connect to Dcmi
	using D8 = GpioSignal;
	/// Connect to Spi3
	using Sck = GpioSignal;
	/// Connect to Usart3 or Uart4
	using Tx = GpioSignal;
	/// @}
#endif
	/// @cond
	template< Peripheral peripheral >
	struct BitBang { static void connect();
		static_assert(
			(peripheral == Peripheral::BitBang),
			"GpioC10::BitBang only connects to software drivers!");
	};
	template< Peripheral peripheral >
	struct Ck { static void connect();
		static_assert(
			(peripheral == Peripheral::I2s3),
			"GpioC10::Ck only connects to I2s3!");
	};
	template< Peripheral peripheral >
	struct D2 { static void connect();
		static_assert(
			(peripheral == Peripheral::Sdio),
			"GpioC10::D2 only connects to Sdio!");
	};
	template< Peripheral peripheral >
	struct D8 { static void connect();
		static_assert(
			(peripheral == Peripheral::Dcmi),
			"GpioC10::D8 only connects to Dcmi!");
	};
	template< Peripheral peripheral >
	struct Sck { static void connect();
		static_assert(
			(peripheral == Peripheral::Spi3),
			"GpioC10::Sck only connects to Spi3!");
	};
	template< Peripheral peripheral >
	struct Tx { static void connect();
		static_assert(
			(peripheral == Peripheral::Usart3) ||
			(peripheral == Peripheral::Uart4),
			"GpioC10::Tx only connects to Usart3 or Uart4!");
	};
	/// @endcond
private:
	template< Peripheral peripheral >
	static constexpr int8_t AdcChannel = -1;
	template< Peripheral peripheral >
	static constexpr int8_t DacChannel = -1;
};

/// @cond
template<>
struct GpioC10::BitBang<Peripheral::BitBang>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::BitBang;
	static constexpr int af = -1;
	inline static void connect() {}
};
template<>
struct GpioC10::Ck<Peripheral::I2s3>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Ck;
	static constexpr int af = 6;
	inline static void
	connect()
	{
		setAlternateFunction(6);
	}
};
template<>
struct GpioC10::D2<Peripheral::Sdio>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::D2;
	static constexpr int af = 12;
	inline static void
	connect()
	{
		setAlternateFunction(12);
	}
};
template<>
struct GpioC10::D8<Peripheral::Dcmi>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::D8;
	static constexpr int af = 13;
	inline static void
	connect()
	{
		setAlternateFunction(13);
	}
};
template<>
struct GpioC10::Sck<Peripheral::Spi3>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Sck;
	static constexpr int af = 6;
	inline static void
	connect()
	{
		setAlternateFunction(6);
	}
};
template<>
struct GpioC10::Tx<Peripheral::Usart3>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Tx;
	static constexpr int af = 7;
	inline static void
	connect()
	{
		setAlternateFunction(7);
	}
};
template<>
struct GpioC10::Tx<Peripheral::Uart4>
{
	using Gpio = GpioC10;
	static constexpr Gpio::Signal Signal = Gpio::Signal::Tx;
	static constexpr int af = 8;
	inline static void
	connect()
	{
		setAlternateFunction(8);
	}
};
/// @endcond

} // namespace modm::platform

#endif // MODM_STM32_GPIO_PIN_C10_HPP