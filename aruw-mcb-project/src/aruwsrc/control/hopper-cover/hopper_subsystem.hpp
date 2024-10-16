/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

/*
   Hopper subsystem is made of a servo that will spin
   to two certain angles determined by the user as
   the open position and the close position
*/

#ifndef HOPPER_SUBSYSTEM_HPP_
#define HOPPER_SUBSYSTEM_HPP_

#include "tap/control/command.hpp"
#include "tap/control/command_scheduler.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/motor/servo.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"

namespace aruwsrc
{
namespace control
{
class HopperSubsystem;

class HopperTestCommand : public tap::control::Command
{
public:
    HopperTestCommand(HopperSubsystem* subsystem);

    bool isReady() override { return true; };

    void initialize() override;

    void execute() override{};

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "hopper test command"; }

private:
    HopperSubsystem* subsystem;
    uint64_t startTime;

};  // class HopperTestCommand

class HopperSubsystem : public tap::control::Subsystem
{
public:
#if defined(ALL_STANDARDS)
    static constexpr float STANDARD_HOPPER_OPEN_PWM = 0.21f;
    static constexpr float STANDARD_HOPPER_CLOSE_PWM = 0.11f;
    static constexpr float STANDARD_PWM_RAMP_SPEED = 0.001f;
#endif

    /*
     * constructor
     * @param pwmPin the pin that the servo is connected to
     * @param open     the angle defined as open; a PWM value
     *                 (between 0 and 1)
     * @param close    the angle defined as close; a PWM value
     *                 (between 0 and 1)
     * @param pwmRampSpeed   determines the speed of servo operation;
     *                 a PWM value (between 0 and 1)
     */
    HopperSubsystem(
        tap::Drivers* drivers,
        tap::gpio::Pwm::Pin pwmPin,
        float open,
        float close,
        float pwmRampSpeed);

    /*
     * set servo to the open angle
     */
    mockable void setOpen();

    /*
     * set servo to the close angle
     */
    mockable void setClose();

    void refresh() override;

    const char* getName() const override { return "Hopper"; }

private:
    tap::motor::Servo hopper;

    HopperTestCommand hopperTestCommand;

    /*
     * return the angle defined as open as a PWM value
     */
    float getOpenPWM();

    /*
     * return the angle defined as close as a PWM value
     */
    float getClosePWM();
};

}  // namespace control

}  // namespace aruwsrc

#endif  // HOPPER_SUBSYSTEM_HPP_
