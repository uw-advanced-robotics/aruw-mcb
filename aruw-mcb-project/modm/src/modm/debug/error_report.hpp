/*
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Martin Rosekeit
 * Copyright (c) 2012, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_ERROR_REPORT_HPP
#define MODM_ERROR_REPORT_HPP

#include <stdint.h>

namespace modm
{
	/**
	 * Global error reporter
	 *
	 * Used to report severe errors at one place.
	 *
	 * @ingroup	modm_debug
	 * @author	Fabian Greif
	 */
	class ErrorReport
	{
	public:
		typedef void (*Handler)(uint16_t errorCode);

		/**
		 * Attach a Global Error Handler Function.
		 *
		 * This handler is used to report severe errors (Buffer Overflows in
		 * Peripheral Drivers etc.). The error code is architecture dependent.
		 */
		static void
		attach(Handler handler);

		/**
		 * Remove Error Handler.
		 */
		static void
		detach();

		static inline void
		report(uint16_t errorCode)
		{
			globalErrorHandler(errorCode);
		}

	private:
		ErrorReport();

		static Handler globalErrorHandler;
	};
}

#endif	// MODM_ERROR_REPORT_HPP
