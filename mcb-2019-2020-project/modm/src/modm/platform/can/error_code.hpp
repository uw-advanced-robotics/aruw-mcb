/*
 * Copyright (c) 2009, Georgi Grinshpun
 * Copyright (c) 2009-2011, 2017, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2011-2012, 2014-2017, Niklas Hauser
 * Copyright (c) 2013, Kevin Läufer
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_ERROR_HPP
#define MODM_STM32_ERROR_HPP

namespace modm
{
	namespace platform
	{
		/// @ingroup modm_platform_can
		enum ErrorCode
		{
			CAN_ERROR_PASSIVE_MODE = 0x0010,
			CAN_BUS_OFF_MODE = 0x0011,
			CAN_FIFO0_OVERFLOW = 0x0012,
			CAN_FIFO1_OVERFLOW = 0x0013,
			CAN_TX_OVERFLOW = 0x0014,

			CAN1_ERROR_PASSIVE_MODE = 0x0010,
			CAN1_BUS_OFF_MODE = 0x0011,
			CAN1_FIFO0_OVERFLOW = 0x0012,
			CAN1_FIFO1_OVERFLOW = 0x0013,
			CAN1_TX_OVERFLOW = 0x0014,

			CAN2_ERROR_PASSIVE_MODE = 0x0010,
			CAN2_BUS_OFF_MODE = 0x0011,
			CAN2_FIFO0_OVERFLOW = 0x0012,
			CAN2_FIFO1_OVERFLOW = 0x0013,
			CAN2_TX_OVERFLOW = 0x0014,

			CAN3_ERROR_PASSIVE_MODE = 0x0010,
			CAN3_BUS_OFF_MODE = 0x0011,
			CAN3_FIFO0_OVERFLOW = 0x0012,
			CAN3_FIFO1_OVERFLOW = 0x0013,
			CAN3_TX_OVERFLOW = 0x0014,
		};
	}
}

#endif
