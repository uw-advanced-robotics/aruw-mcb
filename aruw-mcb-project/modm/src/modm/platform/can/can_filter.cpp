/*
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2014-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "can_filter.hpp"

// ----------------------------------------------------------------------------
void
modm::platform::CanFilter::setFilterBase(uint8_t bank, uint32_t flags, uint32_t fr1, uint32_t fr2)
{
	uint32_t mask = (1UL << bank);

	// Initialization mode for the filter
	CAN->FMR |= CAN_FMR_FINIT;

	// Disable filter
	CAN->FA1R &= ~mask;

	if (flags & LIST_MODE) {
		CAN->FM1R |= mask;
	}
	else {
		CAN->FM1R &= ~mask;
	}

	if (flags & SINGLE_MODE) {
		CAN->FS1R |= mask;
	}
	else {
		CAN->FS1R &= ~mask;
	}

	if (flags & FIFO1) {
		CAN->FFA1R |= mask;
	}
	else {
		CAN->FFA1R &= ~mask;
	}

	CAN->sFilterRegister[bank].FR1 = fr1;
	CAN->sFilterRegister[bank].FR2 = fr2;

	// re-enable filter
	CAN->FA1R |= mask;

	// Leave the initialization mode for the filter
	CAN->FMR &= ~CAN_FMR_FINIT;
}

void
modm::platform::CanFilter::disableFilter(uint8_t id)
{
	uint32_t mask = (1UL << id);

	// Initialization mode for the filter
	CAN->FMR |= CAN_FMR_FINIT;

	// Disable filter
	CAN->FA1R &= ~mask;

	// Leave the initialization mode for the filter
	CAN->FMR &= ~CAN_FMR_FINIT;
}

// ----------------------------------------------------------------------------
