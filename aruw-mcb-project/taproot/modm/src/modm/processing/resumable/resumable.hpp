/*
 * Copyright (c) 2023, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "macros.hpp"
#include <modm/processing/fiber.hpp>
#include <modm/processing/fiber/mutex.hpp>
#include <stdint.h>

#define MODM_RESUMABLE_IS_FIBER

namespace modm
{

/// @ingroup	modm_processing_resumable
/// @{

/// Resumable functions implemented via fibers return like a normal function
template < typename T >
using ResumableResult = T;

/// Resumable functions implemented via fibers are normal functions
template< uint8_t Functions = 1 >
class Resumable
{
public:
    bool isResumableRunning(uint8_t id) const
    {
        return id < Functions and rfStateArray[id].locked;
    }
    bool areAnyResumablesRunning() const
    {
        for (const auto &state : rfStateArray) if (state.locked) return true;
        return false;
    }
    bool areAnyResumablesRunning(std::initializer_list<uint8_t> ids) const
    {
        for (uint8_t id : ids) if (isResumableRunning(id)) return true;
        return false;
    }
    bool areAllResumablesRunning(std::initializer_list<uint8_t> ids) const
    {
        for (uint8_t id : ids) if (not isResumableRunning(id)) return false;
        return true;
    }
    bool joinResumables(std::initializer_list<uint8_t> ids) const
    {
        modm::this_fiber::poll([&]{ return not areAnyResumablesRunning(ids); });
        return true;
    }
    // unimplementable with fibers, but may be stubbed by user application
    void stopAllResumables();
    bool stopResumable(uint8_t id);

protected:
    /// @cond
    template<uint8_t index>
    static void
    checkRfFunctions()
    {
        static_assert(index < Functions,
                "Index out of bounds! Increase the `Functions` template argument of your Resumable class.");
    }
    template<bool isNested>
    static void
    checkRfType()
    {
        static_assert(isNested == false, "You must declare an index for this resumable function!");
    }
    modm::fiber::mutex rfStateArray[Functions];
    /// @endcond
};

/// Resumable functions implemented via fibers are normal functions
template< uint8_t Levels = 1 >
class NestedResumable
{
public:
    bool isResumableRunning() const
    {
        return rfState.owner != rfState.NoOwner;
    }
    int8_t getResumableDepth() const
    {
        return isResumableRunning() ? rfState.count - 1 : -1;
    }
    // unimplementable with fibers, but may be stubbed by user application
    void stopResumable();

protected:
    /// @cond
    template<bool isNested>
    static void
    checkRfType()
    {
        static_assert(isNested == true, "You cannot declare an index for a _nested_ resumable function!");
    }

    modm::fiber::recursive_mutex rfState;
    /// @endcond
};

/// @}

} // namespace modm
