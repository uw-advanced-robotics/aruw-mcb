/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef BALSTD_CHASSIS_SUBSYSTEM_HPP_
#define BALSTD_CHASSIS_SUBSYSTEM_HPP_

#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::control::balstd
{

class BalstdChassisSubsystem : public chassis::HolonomicChassisSubsystem
{
public:
    BalstdChassisSubsystem(
        tap::Drivers* drivers,
        tap::motor::DjiMotor* leftMotor,
        tap::motor::DjiMotor* leftMidMotor,
        tap::motor::DjiMotor* rightMidMotor,
        tap::motor::DjiMotor* rightMotor,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor);

    void initialize() override;

    void refresh() override;

    void refreshSafeDisconnect() override;

    void setDesiredOutput(float x, float y, float r) override;

    void setZeroRPM() override;

    bool allMotorsOnline() const override;

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

    void limitChassisPower() override;

    const char* getName() const override { return "BalstdChassisSubsystem"; }

private:
    void balstdDriveCalculate(float x, float y, float z, float maxWheelSpeed);

    tap::motor::DjiMotor* motors[4];

};  // class BalstdChassisSubsystem

}  // namespace aruwsrc::control::balstd
#endif  // BALSTD_CHASSIS_SUBSYSTEM_HPP_
