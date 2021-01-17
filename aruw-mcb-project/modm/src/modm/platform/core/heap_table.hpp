/*
 * Copyright (c) 2019, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include <stdint.h>
#include <iterator>
#include <tuple>
#include <modm/architecture/interface/memory.hpp>

namespace modm::platform
{

/**
 * Provides information about the unused memory sections in RAM listed by
 * size and memory traits.
 *
 * The iterator returns a tuple of values for each memory section:
 * 1. MemoryTraits traits,
 * 2. const uint32_t *start_addr,
 * 3. const uint32_t *end_addr,
 * 4. size_t size in Bytes.
 *
 * @ingroup modm_platform_cortex_m
 */
class HeapTable
{
public:
	class Iterator;
	Iterator begin();
	Iterator end();

	/// Find the largest *continuous* memory section which satisfies *at least*
	/// the selected memory traits.
	static bool
	find_largest(const uint8_t **start,
				 const uint8_t **end,
				 MemoryTraits trait_mask = MemoryDefault);

	/// @cond
public:
	class Iterator
	{
		using Type = std::tuple<MemoryTraits, const uint8_t*, const uint8_t*, size_t>;
		const void* table;
	public:
		using iterator_category = std::input_iterator_tag;
		using value_type = Type;
		using difference_type = std::ptrdiff_t;
		using pointer = Type*;
		using reference = Type&;

		explicit Iterator(const void* table);
		Type operator*() const;
		Iterator& operator++();
		Iterator operator++(int);

		bool operator==(const Iterator& other) const;
		bool operator!=(const Iterator& other) const;
	};
	/// @endcond
};

}