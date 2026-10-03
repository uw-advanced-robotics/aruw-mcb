/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef EXTERNAL_VELOCITY_COMMAND_HPP_
#define EXTERNAL_VELOCITY_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"

namespace aruwsrc::control::chassis
{
/**
 * Drives the chassis with chassis-relative velocities sent by the vision coprocessor
 * (CV_MESSAGE_TYPE_CHASSIS_VELOCITY). The outer loop is closed off-board (e.g. on motion capture),
 * so this command never reads odometry. If no command arrives for COMMAND_TIMEOUT_MS the chassis
 * is stopped.
 */
class ExternalVelocityCommand : public tap::control::Command
{
public:
    static constexpr uint32_t COMMAND_TIMEOUT_MS = 150;

    /// Hard limits applied on the MCB regardless of what the coprocessor asks for.
    static constexpr float MAX_TRANSLATION_SPEED = 2.5f;  // m/s
    static constexpr float MAX_ROTATION_SPEED = 6.0f;     // rad/s

    /**
     * @param[in] wheelRadius wheel radius (m), used to convert m/s to the chassis' wheel rpm units
     * @param[in] wheelbaseRadius distance from chassis center to a wheel (m)
     */
    ExternalVelocityCommand(
        HolonomicChassisSubsystem& chassis,
        const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor,
        float wheelRadius,
        float wheelbaseRadius);

    void initialize() override;
    void execute() override;
    void end(bool) override;
    bool isFinished() const override { return false; }
    const char* getName() const override { return "external velocity"; }

private:
    HolonomicChassisSubsystem& chassis;
    const aruwsrc::communication::serial::VisionCoprocessor& visionCoprocessor;
    const float metersPerSecondToWheelRpm;
    const float radiansPerSecondToWheelRpm;
};
}  // namespace aruwsrc::control::chassis

#endif  // EXTERNAL_VELOCITY_COMMAND_HPP_
