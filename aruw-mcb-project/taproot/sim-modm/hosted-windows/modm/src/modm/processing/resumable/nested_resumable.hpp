/*
 * Copyright (c) 2014, Kevin Läufer
 * Copyright (c) 2014-2015, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_NESTED_RESUMABLE_HPP
#define MODM_NESTED_RESUMABLE_HPP

#include "macros.hpp"
#include <modm/architecture/utils.hpp>
#include <stdint.h>

namespace modm
{

/**
 * @ingroup	modm_processing_resumable
 * @author	Niklas Hauser
 * @tparam	Levels	maximum number of nesting levels (should be <128):
 * 					max(number of resumable functions that are called within resumable functions) + 1
 */
template< uint8_t Levels = 1>
class NestedResumable
{
	static_assert(Levels > 0, "The number of nesting levels must be at least 1!");

protected:
	/// Construct a new class with nested resumable functions
	NestedResumable()
	:	rfLevel(0)
	{
		for (rf::State &level : rfStateArray)
		{
			level = rf::Stopped;
		}
	}

public:
	/// Force all resumable functions to stop running at the current nesting level
	inline void
	stopResumable()
	{
		uint_fast8_t level = rfLevel;
		while (level < Levels)
		{
			rfStateArray[level++] = rf::Stopped;
		}
	}

	/// @return	`true` if a resumable function is running at the current nesting level, else `false`
	bool inline
	isResumableRunning() const
	{
		return !isStoppedRf();
	}

	/// @return the nesting depth in the current resumable function, or -1 if called outside any resumable function
	int8_t inline
	getResumableDepth() const
	{
		return static_cast<int8_t>(rfLevel) - 1;
	}

#ifdef __DOXYGEN__
	/**
	 * Run the resumable function.
	 *
	 * You need to implement this method in you subclass yourself.
	 *
	 * @return	>`NestingError` if still running, <=`NestingError` if it has finished.
	 */
	modm::ResumableResult< ReturnType >
	resumable function(...);
#endif

protected:
	/// @cond

	/// increases nesting level, call this in the switch statement!
	/// @return current state before increasing nesting level
	rf::State inline
	pushRf(uint8_t /*index*/)
	{
		return rfStateArray[rfLevel++];
	}

	/// always call this before returning from the run function!
	/// decreases nesting level
	void inline
	popRf()
	{
		rfLevel--;
	}

	// invalidates the parent nesting level
	// @warning	be aware in which nesting level you call this! (before popRf()!)
	void inline
	stopRf(uint8_t /*index*/)
	{
		rfStateArray[rfLevel-1] = rf::Stopped;
	}

	/// sets the state of the parent nesting level
	/// @warning	be aware in which nesting level you call this! (before popRf()!)
	void inline
	setRf(rf::State state, uint8_t /*index*/)
	{
		rfStateArray[rfLevel-1] = state;
	}

	/// @return `true` if the nesting depth allows for another level.
	/// @warning	be aware in which nesting level you call this! (before pushRf()!)
	bool inline
	nestingOkRf() const
	{
		return modm_assert_continue_fail_debug(rfLevel < Levels, "rf.nest",
		    	"Called too many nested ResumableFunctions of the same class!", this);
	}

	/// @return	`true` if `stopRf()` has been called before
	bool inline
	isStoppedRf() const
	{
		return (rfStateArray[rfLevel] == rf::Stopped);
	}

	/// compatibility with Resumable class
	template<uint8_t index>
	static void
	checkRfFunctions()
	{}

	/// asserts that this method is called in this parent class
	template<bool isNested>
	static void
	checkRfType()
	{
		static_assert(isNested == true, "You cannot declare an index for a _nested_ resumable function!");
	}
	/// @endcond
private:
	uint_fast8_t rfLevel;
	rf::State rfStateArray[Levels];
};

// ----------------------------------------------------------------------------
// we won't document the specialisation again
/// @cond
template <>
class NestedResumable<1>
{
protected:
	NestedResumable() :
		rfLevel(-1), rfState(rf::Stopped)
	{
	}

public:
	void inline
	stopResumable()
	{
		rfState = rf::Stopped;
	}

	bool inline
	isResumableRunning() const
	{
		return !isStoppedRf();
	}

	int8_t inline
	getResumableDepth() const
	{
		return rfLevel;
	}

protected:
	rf::State inline
	pushRf(uint8_t /*index*/)
	{
		rfLevel = 0;
		return rfState;
	}

	void inline
	popRf()
	{
		rfLevel = -1;
	}

	void inline
	stopRf(uint8_t /*index*/)
	{
		rfState = rf::Stopped;
	}

	bool inline
	nestingOkRf() const
	{
		return (rfLevel != 0);
	}

	void inline
	setRf(rf::State state, uint8_t /*index*/)
	{
		rfState = state;
	}

	bool inline
	isStoppedRf() const
	{
		return (rfState == rf::Stopped);
	}

	template<uint8_t index>
	static void
	checkRfFunctions()
	{}

	template<bool isNested>
	static void
	checkRfType()
	{
		static_assert(isNested == true, "You cannot declare an index for a _nested_ resumable function!");
	}

private:
	int_fast8_t rfLevel;
	rf::State rfState;
};
/// @endcond

} // namespace modm

#endif // MODM_NESTED_RESUMABLE_HPP