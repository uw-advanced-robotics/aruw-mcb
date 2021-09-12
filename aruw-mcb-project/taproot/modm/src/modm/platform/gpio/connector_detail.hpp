/*
 * Copyright (c) 2017, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_PLATFORM_GPIO_CONNECTOR_DETAIL_HPP
#define MODM_PLATFORM_GPIO_CONNECTOR_DETAIL_HPP

#include "unused.hpp"
#include <type_traits>

namespace modm
{

namespace platform
{

/// @cond
namespace detail
{

template< Peripheral peripheral, class GpioQ, template<Peripheral _> class... Signals >
struct GpioContains;
template< Peripheral peripheral, class GpioQ, template<Peripheral _> class SignalT, template<Peripheral _> class... Signals >
struct GpioContains<peripheral, GpioQ, SignalT, Signals...>
{
	using SGpio = typename SignalT<peripheral>::Gpio;
	static constexpr bool value = (
				std::is_same_v<typename SGpio::Type, typename GpioQ::Type> ?
				true :
				GpioContains<peripheral, GpioQ, Signals...>::value
			);
};
template< Peripheral peripheral, class GpioQ >
struct GpioContains<peripheral, GpioQ>
{
	static constexpr bool value = false;
};

template< Peripheral peripheral, Gpio::Signal signal, template<Peripheral _> class... Signals >
struct GpioGetSignal;
template< Peripheral peripheral, Gpio::Signal signal, template<Peripheral _> class SignalT, template<Peripheral _> class... Signals >
struct GpioGetSignal<peripheral, signal, SignalT, Signals...>
{
	using Gpio = std::conditional_t<
				(SignalT<peripheral>::Signal == signal),
				typename SignalT<peripheral>::Gpio,
				typename GpioGetSignal<peripheral, signal, Signals...>::Gpio
			>;
};
template< Peripheral peripheral, Gpio::Signal signal >
struct GpioGetSignal<peripheral, signal>
{
	using Gpio = GpioUnused;
};

template< Peripheral peripheral, template<Peripheral _> class... Signals >
struct GpioSignalConnect;
template< Peripheral peripheral, template<Peripheral _> class SignalT, template<Peripheral _> class... Signals >
struct GpioSignalConnect<peripheral, SignalT, Signals...>
{
	static inline void connect()
	{
		SignalT<peripheral>::connect();
		GpioSignalConnect<peripheral, Signals...>::connect();
	}
	static inline void disconnect()
	{
		SignalT<peripheral>::Gpio::disconnect();
		GpioSignalConnect<peripheral, Signals...>::disconnect();
	}
};
template< Peripheral peripheral >
struct GpioSignalConnect<peripheral>
{
	static inline void connect() {}
	static inline void disconnect() {}
};

} // namespace detail
/// @endcond

} // namespace platform

} // namespace modm

#endif // MODM_PLATFORM_GPIO_CONNECTOR_DETAIL_HPP