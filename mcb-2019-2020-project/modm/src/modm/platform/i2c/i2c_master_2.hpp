/*
 * Copyright (c) 2013, Kevin Läufer
 * Copyright (c) 2013-2017, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_STM32_I2C_2_HPP
#define MODM_STM32_I2C_2_HPP

#include "../device.hpp"
#include <modm/platform/gpio/connector.hpp>
#include <modm/architecture/interface/i2c_master.hpp>

namespace modm
{

namespace platform
{

/**
 * I2cMaster implementation of I2C2 module.
 *
 * Interrupts must be enabled.
 *
 * @author		Georgi Grinshpun
 * @author		Niklas Hauser
 * @ingroup		modm_platform_i2c modm_platform_i2c_2
 */
class I2cMaster2 : public ::modm::I2cMaster
{
public:
	static constexpr size_t TransactionBufferSize = 8;

public:
	template< template<Peripheral _> class... Signals, ResetDevices reset = ResetDevices::Standard>
	static void
	connect(PullUps pullups = PullUps::External)
	{
		using Connector = GpioConnector<Peripheral::I2c2, Signals...>;
		using Scl = typename Connector::template GetSignal<Gpio::Signal::Scl>;
		using Sda = typename Connector::template GetSignal<Gpio::Signal::Sda>;
		static_assert(sizeof...(Signals) == 2 and
					  Connector::template IsValid<Scl> and Connector::template IsValid<Sda>,
					  "I2cMaster2::connect() requires one Scl and one Sda signal!");
		const Gpio::InputType input =
			(pullups == PullUps::Internal) ? Gpio::InputType::PullUp : Gpio::InputType::Floating;

		Connector::disconnect();
		Scl::configure(input);
		Sda::configure(input);
		Scl::setOutput(Gpio::OutputType::OpenDrain);
		Sda::setOutput(Gpio::OutputType::OpenDrain);
		if (reset != ResetDevices::NoReset) resetDevices<Scl, uint32_t(reset)>();
		Connector::connect();
	}

	/**
	 * Set up the I2C module for master operation.
	 *
	 * @param	rate
	 *		`Standard` or `Fast`, `High` datarate is not supported
	 */
	template<class SystemClock, baudrate_t baudrate=kBd(100), percent_t tolerance=pct(5)>
	static modm_always_inline void
	initialize()
	{
		// calculate the expected clock ratio
		constexpr uint8_t scalar = (baudrate <= 100'000) ? 2 : ((baudrate <= 300'000) ? 3 : 25);

		// calculate the fractional prescaler value
		constexpr float pre_raw  = float(SystemClock::I2c2) / (scalar * baudrate);
		// respect the prescaler range of 1 or 4 to 4095
		constexpr uint32_t pre_ceil = std::ceil(pre_raw) > 4095 ? 4095 : std::ceil(pre_raw);
		constexpr uint32_t pre_floor = std::floor(pre_raw) < ((scalar < 3) ? 4 : 1) ?
				((scalar < 3) ? 4 : 1) : std::floor(pre_raw);

		// calculate the possible baudrates above and below the requested baudrate
		constexpr uint32_t baud_lower = SystemClock::I2c2 / ( scalar * pre_ceil  );
		constexpr uint32_t baud_upper = SystemClock::I2c2 / ( scalar * pre_floor );

		// calculate the fractional prescaler value corresponding to the baudrate exactly
		// between the upper and lower baudrate
		constexpr uint32_t baud_middle = (baud_upper + baud_lower) / 2;
		// decide which prescaler value is closer to a possible baudrate
		constexpr uint32_t pre = (baudrate < baud_middle) ? pre_ceil : pre_floor;

		// check if within baudrate tolerance
		constexpr uint32_t generated_baudrate = SystemClock::I2c2 / ( scalar * pre );
		assertBaudrateInTolerance<
				/* clostest available baudrate */ generated_baudrate,
				/* desired baudrate */ baudrate,
				tolerance >();

		// the final prescaler value is augmented with the F/S and DUTY bit.
		constexpr uint32_t prescaler = pre |
				((scalar >= 3) ? (1 << 15) : 0) |
				((scalar == 25) ? (1 << 14) : 0);

		// peripheral frequency clock
		constexpr uint8_t freq = SystemClock::I2c2 / 1'000'000;

		// maximum rise time: assuming its linear:
		// 1000ns @ 100kHz and 300ns @ 400kHz
		//   => y = x * m + b, with m = -2.3333ns/kHz, b = 1'233.3333ns
		constexpr float max_rise_time = -2.333333f * (float(baudrate) / 1'000.f) + 1'233.333333f;
		// calculate trise
		constexpr float trise_raw = max_rise_time < 0 ? 0 : std::floor(max_rise_time / (1'000.f / freq));
		constexpr uint8_t trise = trise_raw > 62 ? 63 : (trise_raw + 1);

		initializeWithPrescaler(freq, trise, prescaler);
	}

	// start documentation inherited
	static bool
	start(I2cTransaction *transaction, ConfigurationHandler handler = nullptr);

	static Error
	getErrorState();

	static void
	reset();
	// end documentation inherited

private:
	static void
	initializeWithPrescaler(uint8_t peripheralFrequency, uint8_t riseTime, uint16_t prescaler);
};

} // namespace platform

} // namespace modm

#endif // MODM_STM32_I2C_2_HPP