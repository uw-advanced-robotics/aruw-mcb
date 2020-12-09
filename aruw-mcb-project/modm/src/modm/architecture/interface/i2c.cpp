/*
 * Copyright (c) 2017, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "i2c.hpp"

modm::IOStream&
operator << (modm::IOStream& s, const modm::I2c::Operation op)
{
	switch (op)
	{
		case modm::I2c::Operation::Stop:    s << "Stop";    break;
		case modm::I2c::Operation::Restart: s << "Restart"; break;
		case modm::I2c::Operation::Write:   s << "Write";   break;
		case modm::I2c::Operation::Read:    s << "Read";    break;
	}
	return s;
}

modm::IOStream&
operator << (modm::IOStream& s, const modm::I2c::OperationAfterStart op)
{
	s << static_cast<modm::I2c::Operation>(op);
	return s;
}

modm::IOStream&
operator << (modm::IOStream& s, const modm::I2c::OperationAfterRead op)
{
	s << static_cast<modm::I2c::Operation>(op);
	return s;
}

modm::IOStream&
operator << (modm::IOStream& s, const modm::I2c::OperationAfterWrite op)
{
	s << static_cast<modm::I2c::Operation>(op);
	return s;
}

