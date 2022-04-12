/*
 * Copyright (c) 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include "flash.hpp"

static constexpr uint32_t FLASH_SR_ERR = 0xfffe;

namespace modm::platform
{

bool
Flash::unlock()
{
	Flash::enable();
	if (isLocked())
	{
		FLASH->KEYR = 0x45670123;
		FLASH->KEYR = 0xCDEF89AB;
	}
	return not isLocked();
}

uint8_t
Flash::getPage(uintptr_t offset)
{
	const uint8_t index = (offset >> 17);
	uint8_t small_index{0};
	// 128kB Block 0 and 8 are subdivided into 4x16kB + 64kB
	if (index == 0 or index == 8)
	{
		if (index == 8) small_index += 4;
		// Check upper 64kB first
		if (offset & 0x1'0000ul) small_index += 4;
		// Otherwise check lower 16kB
		else small_index += ((offset & 0xC000) >> 14);
	}
	else small_index += 4;
	// 128kB Blocks
	return index + small_index;
}

uint32_t
Flash::getOffset(uint8_t index)
{
	switch(index) {
		case 0: return (1ul << 14);
		case 1: return (2ul << 14);
		case 2: return (3ul << 14);
		case 3: return (4ul << 14);
		case 4: return (1ul << 16);
		default: index -= 4; break;
	}
	return (1ul << 17) * index;
}

size_t
Flash::getSize(uint8_t modm_unused index)
{
	if (index < 4) return (1ul << 14);
	if (index == 5) return (1ul << 16);
	return (1ul	<< 17);
}

modm_ramcode uint32_t
Flash::erase(uint8_t index, WordSize size)
{
	FLASH->SR = FLASH_SR_ERR;
	FLASH->CR = FLASH_CR_STRT | FLASH_CR_SER | uint32_t(size) |
			((index << FLASH_CR_SNB_Pos) & FLASH_CR_SNB_Msk);
	while(isBusy()) ;
	FLASH->CR = 0;

	return FLASH->SR & FLASH_SR_ERR;
}

modm_ramcode uint32_t
Flash::program(uintptr_t addr, MaxWordType data, WordSize size)
{
	FLASH->SR = FLASH_SR_ERR;
	FLASH->CR = FLASH_CR_PG | uint32_t(size);
	switch(size)
	{
		case WordSize::B8:
			*(uint8_t *) addr = data;
			break;
		case WordSize::B16:
			*(uint16_t *) addr = data;
			break;
		default:
			*(uint32_t *) addr = data;
			break;
	}
	while(isBusy()) ;
	FLASH->CR = 0;

	return FLASH->SR & FLASH_SR_ERR;
}

} // namespace modm::platform