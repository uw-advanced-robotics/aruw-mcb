/*
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2017, Fabian Greif
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2018, Christopher Durand
 * Copyright (c) 2018, Lucas Mösch
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_UARTHAL_7_HPP
#	error 	"Don't include this file directly, use uart_hal_7.hpp instead!"
#endif
#include <modm/platform/clock/rcc.hpp>

// ----------------------------------------------------------------------------
void
modm::platform::UartHal7::setParity(const Parity parity)
{
	uint32_t flags = UART7->CR1;
	flags &= ~(USART_CR1_PCE | USART_CR1_PS | USART_CR1_M);
	flags |= static_cast<uint32_t>(parity);
	if (parity != Parity::Disabled) {
		// Parity Bit counts as 9th bit -> enable 9 data bits
		flags |= USART_CR1_M;
	}
	UART7->CR1 = flags;

}

void
modm::platform::UartHal7::enable()
{
	Rcc::enable<Peripheral::Uart7>();
	UART7->CR1 |= USART_CR1_UE;		// Uart Enable
}

void
modm::platform::UartHal7::disable()
{
	// TX, RX, Uart, etc. Disable
	UART7->CR1 = 0;
	Rcc::disable<Peripheral::Uart7>();
}
template<class SystemClock, modm::baudrate_t baudrate,
		modm::platform::UartHal7::OversamplingMode oversample>
void modm_always_inline
modm::platform::UartHal7::initialize(Parity parity)
{
	initializeWithBrr(UartBaudrate::getBrr<SystemClock::Uart7, baudrate>(),
			parity,
			oversample);
}
template<class SystemClock, modm::baudrate_t baudrate>
void modm_always_inline
modm::platform::UartHal7::initialize(Parity parity)
{
	initializeWithBrr(UartBaudrate::getBrr<SystemClock::Uart7, baudrate>(), parity,
					  UartBaudrate::getOversamplingMode(SystemClock::Uart7, baudrate));
}


void inline
modm::platform::UartHal7::initializeWithBrr(uint16_t brr, Parity parity, OversamplingMode oversample)
{
	enable();
	// DIRTY HACK: disable and reenable uart to be able to set
	//             baud rate as well as parity
	UART7->CR1 &= ~USART_CR1_UE;	// Uart Disable
	// set baudrate
	UART7->BRR = brr;
	setParity(parity);
	setOversamplingMode(oversample);
	UART7->CR1 |=  USART_CR1_UE;	// Uart Reenable
}

void
modm::platform::UartHal7::setOversamplingMode(OversamplingMode mode)
{
	if(mode == OversamplingMode::By16) {
		UART7->CR1 &= ~static_cast<uint32_t>(OversamplingMode::By8);
	} else {
		UART7->CR1 |=  static_cast<uint32_t>(OversamplingMode::By8);
	}

}
void
modm::platform::UartHal7::write(uint8_t data)
{
	UART7->DR = data;
}

void
modm::platform::UartHal7::read(uint8_t &data)
{
	data = UART7->DR;
}

void
modm::platform::UartHal7::setTransmitterEnable(const bool enable)
{
	if (enable) {
		UART7->CR1 |=  USART_CR1_TE;
	} else {
		UART7->CR1 &= ~USART_CR1_TE;
	}
}

void
modm::platform::UartHal7::setReceiverEnable(bool enable)
{
	if (enable) {
		UART7->CR1 |=  USART_CR1_RE;
	} else {
		UART7->CR1 &= ~USART_CR1_RE;
	}
}

void
modm::platform::UartHal7::enableOperation()
{
	UART7->CR1 |= USART_CR1_UE;
}

void
modm::platform::UartHal7::disableOperation()
{
	UART7->CR1 &= ~USART_CR1_UE;
}

bool
modm::platform::UartHal7::isReceiveRegisterNotEmpty()
{
	return UART7->SR & USART_SR_RXNE;
}

bool
modm::platform::UartHal7::isTransmitRegisterEmpty()
{
	return UART7->SR & USART_SR_TXE;
}

void
modm::platform::UartHal7::enableInterruptVector(bool enable, uint32_t priority)
{
	if (enable) {
		// Set priority for the interrupt vector
		NVIC_SetPriority(UART7_IRQn, priority);

		// register IRQ at the NVIC
		NVIC_EnableIRQ(UART7_IRQn);
	}
	else {
		NVIC_DisableIRQ(UART7_IRQn);
	}
}

void
modm::platform::UartHal7::enableInterrupt(Interrupt_t interrupt)
{
	UART7->CR1 |= interrupt.value;
}

void
modm::platform::UartHal7::disableInterrupt(Interrupt_t interrupt)
{
	UART7->CR1 &= ~interrupt.value;
}

modm::platform::UartHal7::InterruptFlag_t
modm::platform::UartHal7::getInterruptFlags()
{
	return InterruptFlag_t( UART7->SR );
}

void
modm::platform::UartHal7::acknowledgeInterruptFlags(InterruptFlag_t flags)
{
	/* Interrupts must be cleared manually by accessing SR and DR.
	 * Overrun Interrupt, Noise flag detected, Framing Error, Parity Error
	 * p779: "It is cleared by a software sequence (an read to the
	 * USART_SR register followed by a read to the USART_DR register"
	 */
	if (flags & InterruptFlag::OverrunError) {
		uint32_t tmp;
		tmp = UART7->SR;
		tmp = UART7->DR;
		(void) tmp;
	}
	(void) flags;	// avoid compiler warning
}