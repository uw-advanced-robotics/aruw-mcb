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

#include "exti.hpp"

namespace modm::platform
{

Exti::Handler
Exti::handlers[Exti::Lines] modm_fastdata;

void Exti::irq(uint32_t mask)
{
	uint32_t flags = EXTI->IMR & mask;
	flags &= EXTI->PR;
	while(flags)
	{
		const uint8_t lmb = 31ul - __builtin_clz(flags);
		auto& fn = handlers[lmb];
		if (fn) fn(lmb);
		EXTI->PR = 1ul << lmb;
		flags &= ~(1ul << lmb);
	}
}

MODM_ISR(EXTI0)
{
	Exti::irq(Exti::getVectorMaskForLine(0));
}
MODM_ISR(EXTI1)
{
	Exti::irq(Exti::getVectorMaskForLine(1));
}
MODM_ISR(EXTI15_10)
{
	Exti::irq(Exti::getVectorMaskForLine(10));
}
MODM_ISR(EXTI2)
{
	Exti::irq(Exti::getVectorMaskForLine(2));
}
MODM_ISR(EXTI3)
{
	Exti::irq(Exti::getVectorMaskForLine(3));
}
MODM_ISR(EXTI4)
{
	Exti::irq(Exti::getVectorMaskForLine(4));
}
MODM_ISR(EXTI9_5)
{
	Exti::irq(Exti::getVectorMaskForLine(5));
}
}