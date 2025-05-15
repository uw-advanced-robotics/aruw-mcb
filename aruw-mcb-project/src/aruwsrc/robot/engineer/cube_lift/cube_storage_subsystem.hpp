/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef CUBE_STORAGE_SUBSYSTEM_HPP_
#define CUBE_STORAGE_SUBSYSTEM_HPP_

#include <tap/motor/motor_interface.hpp>

#include "tap/control/subsystem.hpp"

#include "aruwsrc/control/bounded-subsystem/one_sided_bounded_subsystem_interface.hpp"
#include "aruwsrc/control/bounded-subsystem/trigger/trigger_interface.hpp"
#include "aruwsrc/robot/engineer/cube_lift/engineer_lift_constants.hpp"
#include "aruwsrc/robot/engineer/engineer_drivers.hpp"

namespace aruwsrc::robot::engineer
{
class CubeStorageSubsystem : public aruwsrc::control::OneSidedBoundedSubsystemInterface
{
public:
    CubeStorageSubsystem(
        tap::Drivers* drivers,
        tap::motor::MotorInterface& storageLiftMotor,
        aruwsrc::control::TriggerInterface& trigger,
        uint64_t length);

    void initialize() override;

    void setDesiredOutput(int16_t power);

    bool isLimitSwitched();

    void refresh() override;

    void refreshSafeDisconnect() override;

    uint64_t getUpperBound() const;

    uint64_t getLowerBound() const;

    bool homedAndBounded() const;

    void moveTowardLowerBound();

    void setPositionSetpoint(float newSetpoint);

    float getPositionSetpoint();

    void setVelocitySetpoint(float newSetpoint);

    float getVelocitySetpoint();

    void setPIDState(PIDState state);

    PIDState getPIDState();

    const char* getName() const override { return "Cube Storage"; }

protected:
    tap::motor::MotorInterface& motor;

    void stopDuringHoming();

    void setHome(uint64_t encoderPosition);

    void setUpperBound(uint64_t encoderPosition);

    void setLowerBound(uint64_t encoderPosition);

private:
    bool isLimitSwitch = false;
    // bool isPIDControl = true;
    PIDState pidState = PIDState::NONE;
    float setpoint = 0;
    float lastTime = 0;
    float motorDesiredOutput = 0;
    int16_t homingOutput = 1000;
    uint64_t home = 0;
    uint64_t upperBound = LIFT_UPPER_BOUND;
    uint64_t lowerBound = 0;
    tap::algorithms::SmoothPid pid =
        tap::algorithms::SmoothPid(aruwsrc::robot::engineer::LIFT_MOTOR_PID_CONFIG);
    tap::algorithms::SmoothPid homingPID =
        tap::algorithms::SmoothPid(aruwsrc::robot::engineer::LIFT_HOMING_PID_CONFIG);
    float velocitySetpoint = 100;
    CalibrationState caliState = CalibrationState::AWAITING_CALIBRATE;
};  // class CUBE_STORAGE

}  // namespace aruwsrc::robot::engineer
#endif  // CUBE_STORAGE_SUBSYSTEM_HPP_
