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

#ifndef BARREL_SWITCHER_SUBSYSTEM_HPP
#define BARREL_SWITCHER_SUBSYSTEM_HPP

#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

#include "tap/control/subsystem.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include <gmock/gmock.h>

#include "tap/mock/dji_motor_mock.hpp"
#endif

namespace aruwsrc::control
{

static constexpr int32_t MOTOR_OUTPUT = SHRT_MAX / 16;
static constexpr int16_t MOTOR_POSITION_TOLERANCE = 3000;

struct StallThresholdConfig {
    /**Maximum rpm value at which to detect a stall*/
    int16_t maxRPM;
    /**Minimum torque value at which to not detect a stall*/ 
    int16_t minTorque;
};

enum class BarrelState
{
    IDLE, //at a position that is neither left nor right barrel
    USING_LEFT_BARREL,
    USING_RIGHT_BARREL
};

class BarrelSwitcherSubsystem : public tap::control::Subsystem
{
public:
    BarrelSwitcherSubsystem(
        tap::Drivers* drivers,
        aruwsrc::control::StallThresholdConfig config,
        tap::motor::MotorId motorid);

    void initialize() override;
    void refresh() override;
    bool isStalled() const;
    bool isInPosition() const {return inPosition;}
    void useRight();
    void useLeft();
    void stop();
    BarrelState getBarrelState();

private:
    void setMotorOutput(int32_t velocity);

    //FOR DEBUGGING, to be removed!
    int16_t outputDesiredDebug;
    int16_t torqueDebug;
    int16_t shaftRPMDebug;
    bool stalled;

    /**
     * True if the barrels are in position to shoot, false otherwise (e.g. 
     * the barrel is moving to switch)
    */
    bool inPosition;

    /**
     * The position of where the barrel-switching motor last stalled, to use
     * for keeping the barrel in firing position
    */
    int16_t desiredPosition;

    /**
     * Stores the state of this barrel switcher's state,
     * including homing states and which barrel (left or right) is in use
     */
    BarrelState barrelState;

    /**
     * stores the thresholds for shaftRPM and torque; used to indicate motor stall
     */
    aruwsrc::control::StallThresholdConfig config;

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    testing::NiceMock<tap::mock::DjiMotorMock> motor;

private:
#else
    /**
     * The motor that switches the turret's barrels
     */
    tap::motor::DjiMotor motor;
#endif
};
}  // namespace aruwsrc::control
#endif