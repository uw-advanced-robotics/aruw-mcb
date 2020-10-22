/*
 * Copyright (c) 2009-2010, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2012, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_ALLOCATOR_STATIC_HPP
#define MODM_ALLOCATOR_STATIC_HPP

#include "allocator_base.hpp"

namespace modm
{
	namespace allocator
	{
		/**
		 * \brief	Static memory allocator
		 *
		 * Allocates a big static block and distributes pieces of it during
		 * run-time. No reallocation is done when no more pieces are available.
		 *
		 * \ingroup	modm_utils_allocator
		 * \author	Fabian Greif
		 */
		template <typename T,
				  std::size_t N>
		class Static : public AllocatorBase<T>
		{
		public:
			template <typename U>
			struct rebind
			{
				typedef Static<U, N> other;
			};

		public:
			Static() :
				AllocatorBase<T>(),
				occupied()
			{
			}

			Static(const Static& other) :
				AllocatorBase<T>(other),
				occupied()
			{
			}

			template <typename U>
			Static(const Static<U, N>&) :
				AllocatorBase<T>(),
				occupied()
			{
			}

			// TODO
			// Currently only supports allocating one T space at a time.
			T*
			allocate(std::size_t)
			{
				for (std::size_t i = 0; i < N; i++)
				{
					if (!occupied[i])
					{
						occupied[i] = true;
						return &memory[i];
					}
				}
				return nullptr;
			}

			void
			deallocate(T* t)
			{
				if (memory < t) { return; }
				std::size_t off = memory - t;
				if (off % sizeof(T) != 0) { return; }
				std::size_t i = off / sizeof(T);
				if (i < 0 || i >= N) { return; }
				occupied[i] = false;
			}

		private:
			bool occupied[N];
			T memory[N];
		};
	}
}

#endif // MODM_ALLOCATOR_STATIC_HPP
