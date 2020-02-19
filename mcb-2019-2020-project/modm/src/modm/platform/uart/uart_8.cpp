/*
 * Copyright (c) 2009, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010-2011, 2013, Georgi Grinshpun
 * Copyright (c) 2013-2014, Sascha Schade
 * Copyright (c) 2013, 2016, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2018, Lucas Mösch
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include "uart_hal_8.hpp"
#include "uart_8.hpp"

#include <modm/architecture/interface/atomic_lock.hpp>
#include <modm/architecture/driver/atomic/queue.hpp>

namespace
{
	static modm::atomic::Queue<uint8_t, 1024> rxBuffer;
	static modm::atomic::Queue<uint8_t, 1024> txBuffer;
}
void
modm::platform::Uart8::initializeBuffered(uint32_t interruptPriority)
{
	UartHal8::enableInterruptVector(true, interruptPriority);
	UartHal8::enableInterrupt(Interrupt::RxNotEmpty);
}
void
modm::platform::Uart8::writeBlocking(uint8_t data)
{
	while(!UartHal8::isTransmitRegisterEmpty());
	UartHal8::write(data);
}

void
modm::platform::Uart8::writeBlocking(const uint8_t *data, std::size_t length)
{
	while (length-- != 0) {
		writeBlocking(*data++);
	}
}

void
modm::platform::Uart8::flushWriteBuffer()
{
	while(!isWriteFinished());
}

bool
modm::platform::Uart8::write(uint8_t data)
{
	if(txBuffer.isEmpty() && UartHal8::isTransmitRegisterEmpty()) {
		UartHal8::write(data);
	} else {
		if (!txBuffer.push(data))
			return false;
		// Disable interrupts while enabling the transmit interrupt
		atomic::Lock lock;
		// Transmit Data Register Empty Interrupt Enable
		UartHal8::enableInterrupt(Interrupt::TxEmpty);
	}
	return true;
}

std::size_t
modm::platform::Uart8::write(const uint8_t *data, std::size_t length)
{
	uint32_t i = 0;
	for (; i < length; ++i)
	{
		if (!write(*data++)) {
			return i;
		}
	}
	return i;
}

bool
modm::platform::Uart8::isWriteFinished()
{
	return txBuffer.isEmpty() && UartHal8::isTransmitRegisterEmpty();
}

std::size_t
modm::platform::Uart8::discardTransmitBuffer()
{
	std::size_t count = 0;
	// disable interrupt since buffer will be cleared
	UartHal8::disableInterrupt(UartHal8::Interrupt::TxEmpty);
	while(!txBuffer.isEmpty()) {
		++count;
		txBuffer.pop();
	}
	return count;
}

bool
modm::platform::Uart8::read(uint8_t &data)
{
	if (rxBuffer.isEmpty()) {
		return false;
	} else {
		data = rxBuffer.get();
		rxBuffer.pop();
		return true;
	}
}

std::size_t
modm::platform::Uart8::read(uint8_t *data, std::size_t length)
{
	uint32_t i = 0;
	for (; i < length; ++i)
	{
		if (rxBuffer.isEmpty()) {
			return i;
		} else {
			*data++ = rxBuffer.get();
			rxBuffer.pop();
		}
	}
	return i;
}

std::size_t
modm::platform::Uart8::discardReceiveBuffer()
{
	std::size_t count = 0;
	while(!rxBuffer.isEmpty()) {
		++count;
		rxBuffer.pop();
	}
	return count;
}


MODM_ISR(UART8)
{
	if (modm::platform::UartHal8::isReceiveRegisterNotEmpty()) {
		// TODO: save the errors
		uint8_t data;
		modm::platform::UartHal8::read(data);
		rxBuffer.push(data);
	}
	if (modm::platform::UartHal8::isTransmitRegisterEmpty()) {
		if (txBuffer.isEmpty()) {
			// transmission finished, disable TxEmpty interrupt
			modm::platform::UartHal8::disableInterrupt(modm::platform::UartHal8::Interrupt::TxEmpty);
		}
		else {
			modm::platform::UartHal8::write(txBuffer.get());
			txBuffer.pop();
		}
	}
}
