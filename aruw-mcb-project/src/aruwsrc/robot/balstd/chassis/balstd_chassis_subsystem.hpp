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

#include "tap/control/chassis/chassis_subsystem_interface.hpp"

#include "aruwsrc/robot/balstd/chassis/balstd_leg.hpp"
#include "aruwsrc/robot/balstd/chassis/controllers/chassis_controller_interface.hpp"

namespace aruwsrc::control::balstd
{

class BalstdChassisSubsystem : public tap::control::chassis::ChassisSubsystemInterface
{
public:
    BalstdChassisSubsystem(
        tap::Drivers* drivers,
        BalstdLeg& leftLeg,
        BalstdLeg& rightLeg,
        tap::communication::sensors::imu::ImuInterface& chassisImu);

    void initialize() override;

    void refresh() override;

    void updateState();

    inline const BalstdChassisState& getChassisState() const { return currState; }

    void setOutputs(const BalstdChassisOutput& output);

    inline void attachController(BalstdChassisControllerInterface* newController)
    {
        this->controller = newController;
    }

    void refreshSafeDisconnect() override
    {
        controller = nullptr;
        setZeroRPM();
    }

    void setZeroRPM();

    bool allMotorsOnline() const override;

    inline void resetVirtualWheelPos() { currState.virtualWheelPos = 0; }

    inline modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override
    {
        return modm::Matrix<float, 3, 1>::zeroMatrix();
    }

    inline int getNumChassisMotors() const override { return 6; }

    const char* getName() const override { return "BalstdChassisSubsystem"; }

private:
    BalstdLeg& leftLeg;
    BalstdLeg& rightLeg;

    tap::communication::sensors::imu::ImuInterface& chassisImu;

    BalstdChassisControllerInterface* controller;

    BalstdChassisState currState;

    BalstdChassisOutput currOutput;

    static constexpr float WHEEL_RADIUS_M = 0.0762f;

};  // class BalstdChassisSubsystem

}  // namespace aruwsrc::control::balstd
#endif  // BALSTD_CHASSIS_SUBSYSTEM_HPP_
