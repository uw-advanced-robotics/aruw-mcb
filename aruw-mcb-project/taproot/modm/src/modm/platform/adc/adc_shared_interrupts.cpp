/*
 * Copyright (c) 2009-2010, Fabian Greif
 * Copyright (c) 2009-2010, Martin Rosekeit
 * Copyright (c) 2012, 2014-2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "../device.hpp"
#include <modm/architecture/interface/interrupt.hpp>

// this should be able to be generated instead of using Macros for this.
#include "adc_interrupt_1.hpp"
MODM_ISR(ADC)
{
	if (modm::platform::AdcInterrupt1::getInterruptFlags()) {
		modm::platform::AdcInterrupt1::handler();
	}
}
