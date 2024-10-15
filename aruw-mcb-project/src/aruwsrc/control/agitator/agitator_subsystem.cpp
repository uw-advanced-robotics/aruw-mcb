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

#include "agitator_subsystem.hpp"

#include <cassert>

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/errors/create_errors.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
#include "tap/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

using namespace tap::motor;

namespace aruwsrc
{
namespace agitator
{
AgitatorSubsystem::AgitatorSubsystem(
    tap::Drivers* drivers,
    const tap::algorithms::SmoothPidConfig& pidParams,
    float agitatorGearRatio,
    tap::motor::MotorId agitatorMotorId,
    tap::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    float jammingDistance,
    uint32_t jammingTime,
    bool jamLogicEnabled)
    : tap::control::Subsystem(drivers),
      agitatorPositionPid(pidParams),
      jamChecker(this, jammingDistance, jammingTime),
      gearRatio(agitatorGearRatio),
      jamLogicEnabled(jamLogicEnabled),
      agitatorMotor(
          drivers,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          "agitator motor"),
      agitatorTestCommand(this)
{
    assert(jammingDistance >= 0);
    this->setTestCommand(&agitatorTestCommand);
}

void AgitatorSubsystem::initialize() { agitatorMotor.initialize(); }

void AgitatorSubsystem::refresh()
{
    if (!agitatorIsCalibrated)
    {
        calibrateHere();
    }

    agitatorRunPositionPid();
    if (jamChecker.check())
    {
        subsystemJamStatus = true;
    }
}

void AgitatorSubsystem::agitatorRunPositionPid()
{
    if (!agitatorIsCalibrated)
    {
        agitatorPositionPid.reset();
    }
    else if (!agitatorMotor.isMotorOnline())
    {
        agitatorPositionPid.reset();
        agitatorIsCalibrated = false;
    }
    else
    {
        // dt doesn't need to be exact since we don't use an integral term and we calculate
        // the velocity ourselves, so it currently isn't used.
        agitatorPositionPid.runController(
            desiredAgitatorAngle - getCurrentValue(),
            getVelocity(),
            2.0f);
        agitatorMotor.setDesiredOutput(agitatorPositionPid.getOutput());
    }
}

bool AgitatorSubsystem::calibrateHere()
{
    if (!isOnline())
    {
        return false;
    }
    agitatorCalibratedZeroAngle = getUncalibratedAgitatorAngle();
    agitatorIsCalibrated = true;
    desiredAgitatorAngle = 0.0f;
    clearJam();
    return true;
}

float AgitatorSubsystem::getCurrentValue() const
{
    if (!agitatorIsCalibrated)
    {
        return 0.0f;
    }
    return getUncalibratedAgitatorAngle() - agitatorCalibratedZeroAngle;
}

float AgitatorSubsystem::getJamSetpointTolerance() const
{
    return jamChecker.getJamSetpointTolerance();
}

float AgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    // position is equal to the following equation:
    // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
    return (2.0f * M_PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           agitatorMotor.getEncoderUnwrapped() / gearRatio;
}

}  // namespace agitator

}  // namespace aruwsrc
