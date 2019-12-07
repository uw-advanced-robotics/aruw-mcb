/*
 * Copyright (c) 2015-2018, Niklas Hauser
 * Copyright (c) 2017, Sascha Schade
 * Copyright (c) 2018, Antal Szabó
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_ROBOMASTER_DEV_BOARD_A_HPP
#define MODM_ROBOMASTER_DEV_BOARD_A_HPP

#include <modm/platform.hpp>
#include <modm/architecture/interface/clock.hpp>
#include <src/aruwlib/communication/gpio/analog.hpp>
#include <src/aruwlib/communication/gpio/pwm.hpp>
#include "robot-type/robot.hpp"

using namespace modm::platform;

/// @ingroup TODO
namespace Board
{
    using namespace modm::literals;

/// STM32F427 running at 180MHz from the external 12MHz crystal
struct SystemClock
{
    static constexpr uint32_t Frequency = 180_MHz;
    static constexpr uint32_t Apb1 = Frequency / 2;
    static constexpr uint32_t Apb2 = Frequency;

    static constexpr uint32_t Adc = Apb2;

    static constexpr uint32_t Spi1 = Apb2;
    static constexpr uint32_t Spi2 = Apb1;
    static constexpr uint32_t Spi3 = Apb1;
    static constexpr uint32_t Spi4 = Apb2;
    static constexpr uint32_t Spi5 = Apb2;
    static constexpr uint32_t Spi6 = Apb2;

    static constexpr uint32_t Usart1 = Apb2;
    static constexpr uint32_t Usart2 = Apb1;
    static constexpr uint32_t Usart3 = Apb1;
    static constexpr uint32_t Uart4  = Apb1;
    static constexpr uint32_t Uart5  = Apb1;
    static constexpr uint32_t Usart6 = Apb2;
    static constexpr uint32_t Uart7  = Apb1;
    static constexpr uint32_t Uart8  = Apb1;

    static constexpr uint32_t Can1 = Apb1;
    static constexpr uint32_t Can2 = Apb1;

    static constexpr uint32_t I2c1 = Apb1;
    static constexpr uint32_t I2c2 = Apb1;
    static constexpr uint32_t I2c3 = Apb1;

    static constexpr uint32_t Apb1Timer = 2 * Apb1;
    static constexpr uint32_t Apb2Timer = 2 * Apb2;
    static constexpr uint32_t Timer1  = Apb2Timer;
    static constexpr uint32_t Timer2  = Apb1Timer;
    static constexpr uint32_t Timer3  = Apb1Timer;
    static constexpr uint32_t Timer4  = Apb1Timer;
    static constexpr uint32_t Timer5  = Apb1Timer;
    static constexpr uint32_t Timer6  = Apb1Timer;
    static constexpr uint32_t Timer7  = Apb1Timer;
    static constexpr uint32_t Timer8  = Apb2Timer;
    static constexpr uint32_t Timer9  = Apb2Timer;
    static constexpr uint32_t Timer10 = Apb2Timer;
    static constexpr uint32_t Timer11 = Apb2Timer;
    static constexpr uint32_t Timer12 = Apb1Timer;
    static constexpr uint32_t Timer13 = Apb1Timer;
    static constexpr uint32_t Timer14 = Apb1Timer;

    static bool inline
    enable()
    {
        Rcc::enableExternalCrystal();  // 8 MHz
        Rcc::enablePll(
            Rcc::PllSource::ExternalCrystal,
            6,    // 12MHz / N=6 -> 2MHz
            180,  // 2MHz * M=180 -> 360MHz
            2     // 360MHz / P=2 -> 180MHz = F_cpu
        );

        Rcc::setFlashLatency<Frequency>();
        Rcc::enableSystemClock(Rcc::SystemClockSource::Pll);
        Rcc::setApb1Prescaler(Rcc::Apb1Prescaler::Div2);
        Rcc::setApb2Prescaler(Rcc::Apb2Prescaler::Div1);
        Rcc::updateCoreFrequency<Frequency>();

        return true;
    }
};

// initialize a button built into mcb
using Button = GpioInputB2;

// initialize 9 green Leds and 1 red LED
// leds 1-8 used for error handling codes
// led9 used for error handling error (unrepresentable error)
using LedA = GpioOutputG1;
using LedB = GpioOutputG2;
using LedC = GpioOutputG3;
using LedD = GpioOutputG4;
using LedE = GpioOutputG5;
using LedF = GpioOutputG6;
using LedG = GpioOutputG7;
using LedH = GpioOutputG8;
using LedGreen = GpioOutputF14;
using LedRed = GpioOutputE11;

using Leds = SoftwareGpioPort
<
    LedA, LedB, LedC, LedD, LedE,
    LedF, LedG, LedH, LedGreen, LedRed
>;

// initialize 4 24V outputs
using PowerOut1 = GpioOutputH2;
using PowerOut2 = GpioOutputH3;
using PowerOut3 = GpioOutputH4;
using PowerOut4 = GpioOutputH5;

using PowerOuts = SoftwareGpioPort
<
    PowerOut1, PowerOut2,
    PowerOut3, PowerOut4
>;

// initialize 4 analog input pins
using AnalogInPinS = GpioOutputA0;
using AnalogInPinT = GpioOutputA1;
using AnalogInPinU = GpioOutputA2;
using AnalogInPinV = GpioOutputA3;

using AnalogInPins = SoftwareGpioPort
<
    AnalogInPinS, AnalogInPinT,
    AnalogInPinU, AnalogInPinV
>;

// initialize 4 pwm output pins
using PWMOutPinW = GpioInputI5;
using PWMOutPinX = GpioInputI6;
using PWMOutPinY = GpioInputI7;
using PWMOutPinZ = GpioInputI2;

using PWMOutPins = SoftwareGpioPort
<
    PWMOutPinW, PWMOutPinX,
    PWMOutPinY, PWMOutPinZ
>;

// initialize 4 digital input pins
using DigitalInPinA = GpioOutputI0;
using DigitalInPinB = GpioOutputH12;
using DigitalInPinC = GpioOutputH11;
using DigitalInPinD = GpioOutputH10;

using DigitalInPins = SoftwareGpioPort
<
    DigitalInPinA, DigitalInPinB,
    DigitalInPinC, DigitalInPinD
>;

// initialize 4 digital output pins
using DigitalOutPinE = GpioInputD15;
using DigitalOutPinF = GpioInputD14;
using DigitalOutPinG = GpioInputD13;
using DigitalOutPinH = GpioInputD12;

using DigitalOutPins = SoftwareGpioPort
<
    DigitalOutPinE, DigitalOutPinF,
    DigitalOutPinG, DigitalOutPinH
>;

// gpio pins used for SPI communication to the onboard MPU6500 IMU
using ImuSck = GpioF7;
using ImuMiso = GpioF8;
using ImuMosi = GpioF9;
using ImuNcc = GpioF6;
using ImuSpiMaster = SpiMaster5;

inline void
killAllGpioOutput()
{
    Leds::setOutput(modm::Gpio::High);
    PowerOuts::setOutput(modm::Gpio::Low);
    DigitalOutPins::setOutput(modm::Gpio::Low);
    aruwlib::gpio::Pwm::WriteAll(0.0);
}

inline void
initialize()
{
    // init clock
    SystemClock::enable();
    SysTickTimer::initialize<SystemClock>();
    // init Leds
    Leds::setOutput(modm::Gpio::Low);
    // init 24V output
    PowerOuts::setOutput(modm::Gpio::High);
    // init digital out pins
    DigitalOutPins::setOutput(modm::Gpio::Low);
    // init digital in pins
    // interrupts disabled
    DigitalInPins::setInput();
    // set analog in pins
    AnalogInPins::setAnalogInput();
    // init button on board
    Button::setInput();

    // init PWM and analog pins
    aruwlib::gpio::Analog::init();
    aruwlib::gpio::Pwm::init();

    CanFilter::setStartFilterBankForCan2(14);
    // initialize CAN 1
    Can1::connect<GpioD0::Rx, GpioD1::Tx>(Gpio::InputType::PullUp);
    Can1::initialize<Board::SystemClock, 1000_kbps>(9);
    // receive every message for CAN 1
    CanFilter::setFilter(0, CanFilter::FIFO0,
        CanFilter::StandardIdentifier(0),
        CanFilter::StandardFilterMask(0));
    Can2::connect<GpioB12::Rx, GpioB13::Tx>(Gpio::InputType::PullUp);
    Can2::initialize<Board::SystemClock, 1000_kbps>(12);
    // receive every message for CAN 2
    CanFilter::setFilter(14, CanFilter::FIFO0,
        CanFilter::StandardIdentifier(0),
        CanFilter::StandardFilterMask(0));
}

}  // namespace Board

#endif  // MODM_ROBOMASTER_DEV_BOARD_A_HPP
