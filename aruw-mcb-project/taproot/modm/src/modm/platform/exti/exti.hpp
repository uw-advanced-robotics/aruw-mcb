/*
 * Copyright (c) 2021, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <modm/platform/device.hpp>
#include <modm/architecture/interface/interrupt.hpp>
#include <modm/platform/gpio/base.hpp>
#include <modm/utils/inplace_function.hpp>

#if defined __DOXYGEN__ || !defined MODM_EXTI_HANDLER_STORAGE
/// @ingroup modm_platform_exti
#define MODM_EXTI_HANDLER_STORAGE sizeof(void*)
#endif
namespace modm::platform
{

/// @cond
MODM_ISR_DECL(EXTI0);
MODM_ISR_DECL(EXTI1);
MODM_ISR_DECL(EXTI15_10);
MODM_ISR_DECL(EXTI2);
MODM_ISR_DECL(EXTI3);
MODM_ISR_DECL(EXTI4);
MODM_ISR_DECL(EXTI9_5);
/// @endcond

/**
 * External Interrupt/Event Controller.
 *
 * @author		Niklas Hauser
 * @ingroup		modm_platform_exti
 */
class Exti
{
public:
	enum class
	Trigger
	{
		NoEdges,
		RisingEdge,
		FallingEdge,
		BothEdges,
	};

	enum class
	Vector
	{
		LineUnknown = 0,
		Line0 = EXTI0_IRQn,
		Line1 = EXTI1_IRQn,
		Line10_15 = EXTI15_10_IRQn,
		Line2 = EXTI2_IRQn,
		Line3 = EXTI3_IRQn,
		Line4 = EXTI4_IRQn,
		Line5_9 = EXTI9_5_IRQn,
	};

	using MaskType = uint32_t;
	using Handler = modm::inplace_function<void(uint8_t), MODM_EXTI_HANDLER_STORAGE, alignof(void*)>;
public:
	template< class Pin >
	static void
	connect(Trigger trigger, Handler&& handler, uint8_t priority=15)
	{
		disableInterrupts<Pin>();
		acknowledgeFlags<Pin>();
		// Now it's safe to copy the handler
		handlers[Pin::pin] = handler;

		// Select the source of the exti line
		setTriggerSource<Pin>();
		setTriggers<Pin>(trigger);

		enableVector<Pin>(priority);
		enableInterrupts<Pin>();
	}

	template< class Pin >
	static void
	disconnect()
	{
		disableInterrupts<Pin>();
		// Disable the IRQ vector if all pins are disabled
		if (not (EXTI->IMR & getVectorMaskForLine(Pin::pin)))
			disableVector<Pin>();
		handlers[Pin::pin] = nullptr;
	}
public:
	// - - - TRIGGERS - - -
	/// Selects the GPIO port trigger source for EXTI lines 0-16
	inline static void
	setTriggerSource(uint8_t line, Gpio::Port port)
	{
		const uint8_t index   =  line >> 2;
		const uint8_t bit_pos = (line & 0b0011) << 2;
		const uint16_t syscfg_mask = 0xf << bit_pos;
		const uint16_t syscfg_value = (uint8_t(port) & 0xf) << bit_pos;
		SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~syscfg_mask) | syscfg_value;
	}
	template< class Pin >
	static void setTriggerSource()
	{
		constexpr uint8_t index   =  Pin::pin >> 2;
		constexpr uint8_t bit_pos = (Pin::pin & 0b0011) << 2;
		constexpr uint16_t syscfg_mask = 0xf << bit_pos;
		constexpr uint16_t syscfg_value = (uint8_t(Pin::port) & 0xf) << bit_pos;
		SYSCFG->EXTICR[index] = (SYSCFG->EXTICR[index] & ~syscfg_mask) | syscfg_value;
	}
	template< class... Pins >
	static void setTriggerSources()
	{
		static_assert(((Pins::pin == 0 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 0!");
		static_assert(((Pins::pin == 1 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 1!");
		static_assert(((Pins::pin == 2 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 2!");
		static_assert(((Pins::pin == 3 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 3!");
		static_assert(((Pins::pin == 4 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 4!");
		static_assert(((Pins::pin == 5 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 5!");
		static_assert(((Pins::pin == 6 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 6!");
		static_assert(((Pins::pin == 7 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 7!");
		static_assert(((Pins::pin == 8 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 8!");
		static_assert(((Pins::pin == 9 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 9!");
		static_assert(((Pins::pin == 10 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 10!");
		static_assert(((Pins::pin == 11 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 11!");
		static_assert(((Pins::pin == 12 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 12!");
		static_assert(((Pins::pin == 13 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 13!");
		static_assert(((Pins::pin == 14 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 14!");
		static_assert(((Pins::pin == 15 ? 1 : 0) + ...) <= 1,
		              "Only one GPIO port can trigger line 15!");
		(setTriggerSource<Pins>(), ...);
	}
	inline static void
	setTrigger(MaskType mask, Trigger trigger)
	{
		switch (trigger)
		{
			case Trigger::NoEdges:
				EXTI->RTSR &= ~mask;
				EXTI->FTSR &= ~mask;
				break;
			case Trigger::RisingEdge:
				EXTI->RTSR |=  mask;
				EXTI->FTSR &= ~mask;
				break;
			case Trigger::FallingEdge:
				EXTI->RTSR &= ~mask;
				EXTI->FTSR |=  mask;
				break;
			case Trigger::BothEdges:
				EXTI->RTSR |= mask;
				EXTI->FTSR |= mask;
				break;
		}
	}
	template< class... Pins >
	static void setTriggers(Trigger trigger)
	{ setTrigger(((1ul << Pins::pin) | ...), trigger); }

	// - - - INTERRUPTS - - -
	inline static void
	enableInterrupts(MaskType mask)
	{
		EXTI->IMR |= mask;
	}
	template< class... Pins > static void
	enableInterrupts()
	{ EXTI->IMR |= ((1ul << Pins::pin) | ...); }

	inline static MaskType
	getInterruptEnabled()
	{
		return EXTI->IMR;
	}
	template< class... Pins > static bool
	areInterruptsEnabled()
	{ return EXTI->IMR & ((1ul << Pins::pin) | ...); }

	inline static void
	disableInterrupt(MaskType mask)
	{
		EXTI->IMR &= ~mask;
	}
	template< class... Pins > static void
	disableInterrupts()
	{ EXTI->IMR &= ~((1ul << Pins::pin) | ...); }


	// - - - EVENTS - - -
	inline static void
	enableEvent(MaskType mask)
	{
		EXTI->EMR |= mask;
	}
	template< class... Pins > static void
	enableEvents() { EXTI->EMR |= ((1ul << Pins::pin) | ...); }

	inline static MaskType
	getEventEnabled()
	{
		return EXTI->EMR;
	}
	template< class... Pins > static void
	areEventsEnabled() { return EXTI->EMR & ((1ul << Pins::pin) | ...); }

	inline static void
	disableEvent(MaskType mask)
	{
		EXTI->EMR &= ~mask;
	}
	template< class... Pins > static void
	disableEvents() { EXTI->EMR &= ~((1ul << Pins::pin) | ...); }

	// - - - VECTORS - - -
	inline static void
	enableVector(Vector vector, uint8_t priority)
	{
		NVIC_SetPriority(IRQn_Type(vector), priority);
		NVIC_EnableIRQ(IRQn_Type(vector));
	}
	template< class Pin > static void
	enableVector(uint8_t priority) { enableVector(getVectorForLine(Pin::pin), priority); }

	inline static void
	enableVectors(MaskType mask, uint8_t priority)
	{
		if (getVectorMaskForLine(0) & mask) enableVector(Vector::Line0, priority);
		if (getVectorMaskForLine(1) & mask) enableVector(Vector::Line1, priority);
		if (getVectorMaskForLine(10) & mask) enableVector(Vector::Line10_15, priority);
		if (getVectorMaskForLine(2) & mask) enableVector(Vector::Line2, priority);
		if (getVectorMaskForLine(3) & mask) enableVector(Vector::Line3, priority);
		if (getVectorMaskForLine(4) & mask) enableVector(Vector::Line4, priority);
		if (getVectorMaskForLine(5) & mask) enableVector(Vector::Line5_9, priority);
	}
	template< class... Pins > static void
	enableVectors(uint8_t priority)
	{
		constexpr uint32_t mask = ((1ul << Pins::pin) | ...);
		if constexpr (getVectorMaskForLine(0) & mask) enableVector(Vector::Line0, priority);
		if constexpr (getVectorMaskForLine(1) & mask) enableVector(Vector::Line1, priority);
		if constexpr (getVectorMaskForLine(10) & mask) enableVector(Vector::Line10_15, priority);
		if constexpr (getVectorMaskForLine(2) & mask) enableVector(Vector::Line2, priority);
		if constexpr (getVectorMaskForLine(3) & mask) enableVector(Vector::Line3, priority);
		if constexpr (getVectorMaskForLine(4) & mask) enableVector(Vector::Line4, priority);
		if constexpr (getVectorMaskForLine(5) & mask) enableVector(Vector::Line5_9, priority);
	}

	inline static void
	disableVector(Vector vector)
	{
		NVIC_DisableIRQ(IRQn_Type(vector));
	}
	template< class Pin > static void
	disableVector() { disableVector(getVectorForLine(Pin::pin)); }

	inline static void
	disableVectors(MaskType mask)
	{
		if (getVectorMaskForLine(0) & mask) disableVector(Vector::Line0);
		if (getVectorMaskForLine(1) & mask) disableVector(Vector::Line1);
		if (getVectorMaskForLine(10) & mask) disableVector(Vector::Line10_15);
		if (getVectorMaskForLine(2) & mask) disableVector(Vector::Line2);
		if (getVectorMaskForLine(3) & mask) disableVector(Vector::Line3);
		if (getVectorMaskForLine(4) & mask) disableVector(Vector::Line4);
		if (getVectorMaskForLine(5) & mask) disableVector(Vector::Line5_9);
	}
	template< class... Pins > static void
	disableVectors()
	{
		constexpr uint32_t mask = ((1ul << Pins::pin) | ...);
		if constexpr (getVectorMaskForLine(0) & mask) disableVector(Vector::Line0);
		if constexpr (getVectorMaskForLine(1) & mask) disableVector(Vector::Line1);
		if constexpr (getVectorMaskForLine(10) & mask) disableVector(Vector::Line10_15);
		if constexpr (getVectorMaskForLine(2) & mask) disableVector(Vector::Line2);
		if constexpr (getVectorMaskForLine(3) & mask) disableVector(Vector::Line3);
		if constexpr (getVectorMaskForLine(4) & mask) disableVector(Vector::Line4);
		if constexpr (getVectorMaskForLine(5) & mask) disableVector(Vector::Line5_9);
	}

	// - - - FLAGS - - -
	inline static MaskType
	getFlags()
	{
		return EXTI->PR;
	}
	template< class... Pins >
	static bool areFlagsSet()
	{ return EXTI->PR & ((1ul << Pins::pin) | ...); }

	inline static void
	acknowledgeFlags(MaskType mask)
	{
		EXTI->PR = mask;
	}
	template< class... Pins >
	static void acknowledgeFlags()
	{ EXTI->PR = ((1ul << Pins::pin) | ...); }
	inline static void
	setFlags(MaskType mask)
	{
		EXTI->SWIER = mask;
	}
	template< class... Pins >
	static void setFlags()
	{ EXTI->SWIER = ((1ul << Pins::pin) | ...); }

public:
	static constexpr Vector
	getVectorForLine(uint8_t line)
	{
		switch(line)
		{
			case 0:
				return Vector::Line0;
			case 1:
				return Vector::Line1;
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
			case 15:
				return Vector::Line10_15;
			case 2:
				return Vector::Line2;
			case 3:
				return Vector::Line3;
			case 4:
				return Vector::Line4;
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				return Vector::Line5_9;
		}
		return Vector::LineUnknown;
	}

	static constexpr MaskType
	getVectorMaskForLine(uint8_t line)
	{
		switch(line)
		{
			case 0:
				return 1ul << 0;
			case 1:
				return 1ul << 1;
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
			case 15:
				return ((1ul << 16) - 1ul) & ~((1ul << 10) - 1ul);
			case 2:
				return 1ul << 2;
			case 3:
				return 1ul << 3;
			case 4:
				return 1ul << 4;
			case 5:
			case 6:
			case 7:
			case 8:
			case 9:
				return ((1ul << 10) - 1ul) & ~((1ul << 5) - 1ul);
		}
		return 0;
	}

protected:
	/// @cond
	static void irq(uint32_t mask);
	static constexpr uint8_t Lines = 16;
	static Handler handlers[Lines];
	friend void MODM_ISR_NAME(EXTI0)();
	friend void MODM_ISR_NAME(EXTI1)();
	friend void MODM_ISR_NAME(EXTI15_10)();
	friend void MODM_ISR_NAME(EXTI2)();
	friend void MODM_ISR_NAME(EXTI3)();
	friend void MODM_ISR_NAME(EXTI4)();
	friend void MODM_ISR_NAME(EXTI9_5)();
	/// @endcond
};

}  // namespace modm::platform