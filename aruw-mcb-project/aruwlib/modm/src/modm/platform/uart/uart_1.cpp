/*
 * Copyright (c) 2020 Matthew Arnold
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "uart_1.hpp"

MODM_ISR(USART1)
{
    modm::platform::uart1IrqHandler();
}
