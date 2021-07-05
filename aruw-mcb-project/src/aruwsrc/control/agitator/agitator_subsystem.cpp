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

#include "aruwlib/algorithms/math_user_utils.hpp"
#include "aruwlib/control/subsystem.hpp"
#include "aruwlib/drivers.hpp"
#include "aruwlib/errors/create_errors.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwlib/mock/dji_motor_mock.hpp"
#else
#include "aruwlib/motor/dji_motor.hpp"
#endif

#include "modm/math/filter/pid.hpp"

using namespace aruwlib::motor;

namespace aruwsrc
{
namespace agitator
{
AgitatorSubsystem::AgitatorSubsystem(
    aruwlib::Drivers* drivers,
    const aruwlib::algorithms::PidConfigStruct& pidConfig,
    float agitatorGearRatio,
    aruwlib::motor::MotorId agitatorMotorId,
    aruwlib::can::CanBus agitatorCanBusId,
    bool isAgitatorInverted,
    bool jamLogicEnabled,
    float jamDistanceTolerance,
    uint32_t jamTemporalTolerance)
    : aruwlib::control::Subsystem(drivers),
      jamChecker(this, jamDistanceTolerance, jamTemporalTolerance),
      agitatorPositionPid(pidConfig),
      gearRatio(agitatorGearRatio),
      jamLogicEnabled(jamLogicEnabled),
      agitatorMotor(
          drivers,
          agitatorMotorId,
          agitatorCanBusId,
          isAgitatorInverted,
          "agitator motor")
{
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

float AgitatorSubsystem::getUncalibratedAgitatorAngle() const
{
    // position is equal to the following equation:
    // position = 2 * PI / encoder resolution * unwrapped encoder value / gear ratio
    return (2.0f * aruwlib::algorithms::PI / static_cast<float>(DjiMotor::ENC_RESOLUTION)) *
           agitatorMotor.getEncoderUnwrapped() / gearRatio;
}

void AgitatorSubsystem::runHardwareTests()
{
    if (aruwlib::algorithms::compareFloatClose(
            this->getSetpoint(),
            this->getCurrentValue(),
            aruwlib::algorithms::PI / 16))
    {
        this->setHardwareTestsComplete();
    }
}

void AgitatorSubsystem::onHardwareTestStart()
{
    this->setSetpoint(this->getCurrentValue() + aruwlib::algorithms::PI / 2);
}

}  // namespace agitator

}  // namespace aruwsrc
