/*
 * Copyright (c) 2014, 2016, Niklas Hauser
 * Copyright (c) 2015-2016, Sascha Schade
 * Copyright (c) 2022, Rasmus Kleist Hørlyck Sørensen
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_CAN_MESSAGE_HPP
#define MODM_CAN_MESSAGE_HPP

#include <stdint.h>
#include <string.h>  // strlen
#include <algorithm>
#include <array>
#include <modm/architecture/utils.hpp>

namespace modm::can
{

/// @ingroup modm_architecture_can
struct Message
{
	constexpr Message(uint32_t inIdentifier = 0, uint8_t inLength = 0) :
		identifier(inIdentifier), flags()
	{
        setLength(inLength);
	}

	constexpr Message(uint32_t inIdentifier, uint8_t inLength, const uint64_t &inData, bool extended = false) :
        Message(inIdentifier, std::min(inLength, uint8_t(8)))
	{
		flags.extended = extended;
		for (uint8_t ii = 0; ii < length; ++ii)
			data[ii] = inData >> ((7 - ii - (8 - length)) * 8);
	}

	constexpr Message(uint32_t inIdentifier, uint8_t inLength, const uint8_t *inData, bool extended = false) :
		Message(inIdentifier, inLength)
	{
		flags.extended = extended;
		for (uint8_t ii = 0; ii < length; ++ii)
 			data[ii] = inData[ii];
	}

	constexpr Message(const modm::can::Message& rhs)
		: identifier{rhs.identifier}, flags{rhs.flags}, dlc{rhs.dlc}, length{rhs.length}
	{
		std::copy(std::begin(rhs.data), std::end(rhs.data), std::begin(data));
	}

	constexpr uint32_t
	getIdentifier() const
	{
		return identifier;
	}

	constexpr void
	setIdentifier(uint32_t id)
	{
		identifier = id;
	}

	constexpr uint8_t
	getCapacity() const
	{
		return capacity;
	}

	constexpr void
	setFlexibleData(bool fd = true)
	{
		flags.fd = fd;
	}

	constexpr bool
	isFlexibleData() const
	{
		return (flags.fd != 0);
	}

	constexpr bool
	isBitRateSwitching() const
	{
		return (flags.brs != 0);
	}

	constexpr void
	setExtended(bool extended = true)
	{
		flags.extended = (extended) ? 1 : 0;
	}

	constexpr bool
	isExtended() const
	{
		return (flags.extended != 0);
	}

	constexpr void
	setRemoteTransmitRequest(bool rtr = true)
	{
		flags.rtr = (rtr) ? 1 : 0;
	}

	constexpr bool
	isRemoteTransmitRequest() const
	{
		return (flags.rtr != 0);
	}

	constexpr void
	setDataLengthCode(uint8_t inDlc)
	{
		while (dlcConversionTable[inDlc] > capacity) inDlc--;

		dlc = inDlc;
		length = dlcConversionTable[inDlc];
		if (dlc > 8) setFlexibleData();
	}

	constexpr void
	setLength(uint8_t inLength)
	{
		if constexpr (capacity <= 8)
		{
			length = dlc = std::min<uint8_t>(inLength, capacity);
		}
		else
		{
			uint8_t inDlc = 0;
			while (dlcConversionTable[inDlc] < std::min<uint8_t>(inLength, capacity)) inDlc++;
			setDataLengthCode(inDlc);
		}
	}

	constexpr uint8_t
	getLength() const
	{
		return length;
	}

	constexpr uint8_t
	getDataLengthCode() const
	{
		return dlc;
	}

public:
	static constexpr uint8_t capacity = 8;
	static constexpr uint8_t dlcConversionTable[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
public:
	uint32_t identifier;
	struct Flags
	{
		constexpr Flags() :
			fd(0), brs(0), rtr(0), extended(1)
		{
		}

		bool fd : 1;
		bool brs : 1;
		bool rtr : 1;
		bool extended : 1;
	} flags;
	uint8_t dlc;
	uint8_t length;
	uint8_t modm_aligned(4) data[capacity]{};

public:
	constexpr bool
	operator == (const modm::can::Message& rhs) const
	{
		return ((this->identifier     == rhs.identifier) and
				(this->dlc            == rhs.dlc)        and
				(this->length         == rhs.length)     and
				(this->flags.fd       == rhs.flags.fd)   and
				(this->flags.brs      == rhs.flags.brs)  and
				(this->flags.rtr      == rhs.flags.rtr)  and
				(this->flags.extended == rhs.flags.extended) and
				std::equal(data, data + getLength(), rhs.data));
	}

	constexpr modm::can::Message&
	operator = (const modm::can::Message& rhs)
	{
		this->identifier     = rhs.identifier;
        this->dlc            = rhs.dlc;
		this->length         = rhs.length;
		this->flags.fd       = rhs.flags.fd;
		this->flags.brs      = rhs.flags.brs;
		this->flags.rtr      = rhs.flags.rtr;
		this->flags.extended = rhs.flags.extended;
		std::copy(std::begin(rhs.data), std::end(rhs.data), std::begin(this->data));
		return *this;
	}

	constexpr bool
	operator < (const modm::can::Message& rhs) const
	{
		return (this->identifier << (this->flags.extended ? 0 : 18))
			< (rhs.identifier << (rhs.flags.extended ? 0 : 18));
	}

};

}   // namespace modm::can

#if MODM_HAS_IOSTREAM
#include <inttypes.h>
#include <modm/io/iostream.hpp>

namespace modm
{

/// @ingroup modm_architecture_can
inline modm::IOStream&
operator << (modm::IOStream& s, const modm::can::Message& m)
{
	s.printf("id = %04" PRIx32 ", len = %u, dlc = %u", m.identifier, m.length, m.dlc);
	s.printf(", flags = %c%c%c%c",
			 m.flags.fd  ? 'F' : 'f',
			 m.flags.brs ? 'B' : 'b',
			 m.flags.rtr ? 'R' : 'r',
			 m.flags.extended ? 'E' : 'e');
	if (not m.isRemoteTransmitRequest()) {
		s.printf(", data = ");
		for (uint_fast8_t ii = 0; ii < m.getLength(); ++ii) {
			s.printf("%02x ", m.data[ii]);
		}
	}
	return s;
}

} // modm namespace

#endif

#endif // MODM_CAN_MESSAGE_HPP