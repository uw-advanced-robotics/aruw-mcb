/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BARREL_SWITCHER_SUBSYSTEM_HPP_
#define BARREL_SWITCHER_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include <gmock/gmock.h>

#include "tap/mock/dji_motor_mock.hpp"
#endif

namespace aruwsrc::control
{
static constexpr int32_t MOTOR_OUTPUT = SHRT_MAX / 16;

struct StallThresholdConfig
{
    /**Maximum rpm value at which to detect a stall*/
    int16_t maxRPM;
    /**Minimum torque value at which to not detect a stall*/
    int16_t minTorque;
};

enum class BarrelState
{
    IDLE,
    USING_LEFT_BARREL,
    USING_RIGHT_BARREL
};

class BarrelSwitcherSubsystem : public tap::control::Subsystem
{
public:
    BarrelSwitcherSubsystem(
        tap::Drivers* drivers,
        aruwsrc::control::StallThresholdConfig config,
        tap::motor::DjiMotor& motor);

    bool isStalled() const;

    bool isInPosition() const;

    void useRight();

    void useLeft();

    void stop();

    BarrelState getBarrelState() const;

    /* Inherited Methods */

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override { motor.setDesiredOutput(0); }

private:
    void setMotorOutput(int32_t velocity);

    /**
     * true if there is a barrel in position to shoot, false otherwise
     */
    bool inPosition;

    /**
     * Stores the state of this barrel switcher's state, mainly which barrel (left or right)
     * is currently being used
     */
    BarrelState barrelState;

    /**
     * stores the thresholds for shaftRPM and torque; used to indicate motor stall
     */
    aruwsrc::control::StallThresholdConfig config;

    /**
     * The motor that switches the turret's barrels
     */
    tap::motor::DjiMotor& motor;
};  // class BarrelSwitcherSubsystem
}  // namespace aruwsrc::control

#endif  // BARREL_SWITCHER_SUBSYSTEM_HPP_
