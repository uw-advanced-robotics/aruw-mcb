/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALANCING_CHASSIS_BEYBLADE_COMMAND_HPP_
#define BALANCING_CHASSIS_BEYBLADE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

#include "balancing_chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{

/**
 * Various modes for autorotation. Strict means autorotation is executed immediately. Lazy means a
 * timeout runs between when desired rotation ends and autorotation begins. Keep chassis angle only
 * rotates the chassis to how it needs to translate, and leaves it there indefinitely.
 *
 */
enum BeybladeMode : uint8_t
{
    NORMAL_SPIN = 0,
    UP_DOWN_SPIN,
    NUM_BEYBLADE_MODES,
};

class BalancingChassisBeybladeCommand : public tap::control::Command
{
public:
    BalancingChassisBeybladeCommand(
        tap::Drivers* drivers,
        BalancingChassisSubsystem* chassis,
        aruwsrc::control::ControlOperatorInterface& operatorInterface,
        const aruwsrc::control::turret::TurretMotor* yawMotor);

    const char* getName() const override { return "Balancing Chassis Autorotation Drive Command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    void setBeybladeMode(BeybladeMode mode)
    {
        if (mode < NUM_BEYBLADE_MODES)
        {
            this->beybladeMode = mode;
        }
    }

private:
    float plotPath(float turretAngleFromCenter);
    void updateAutorotateState();
    float getAutorotationSetpoint(float turretAngleFromCenter);
    void runRotationController(float chassisRotationSetpoint, float dt);

    tap::Drivers* drivers;
    BalancingChassisSubsystem* chassis;
    control::ControlOperatorInterface& operatorInterface;
    const aruwsrc::control::turret::TurretMotor* yawMotor;

    tap::arch::MilliTimeout upDownTimeout = tap::arch::MilliTimeout(UP_DOWN_PERIOD);
    float DELTA_H = CHASSIS_HEIGHTS.first - CHASSIS_HEIGHTS.second;

    uint32_t prevTime = 0;

    BeybladeMode beybladeMode = NORMAL_SPIN;
    static constexpr uint32_t UP_DOWN_PERIOD = 500;

    modm::Vector2f motionDesiredTurretRelative;

    bool chassisUp = false;

};  // class BalancingChassisBeybladeCommand

}  // namespace chassis

}  // namespace aruwsrc

#endif  // BALANCING_CHASSIS_BEYBLADE_COMMAND_HPP_