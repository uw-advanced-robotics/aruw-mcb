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

#ifndef BALANCING_CHASSIS_AUTOROTATE_COMMAND_HPP_
#define BALANCING_CHASSIS_AUTOROTATE_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

#include "balancing_chassis_subsystem.hpp"

namespace aruwsrc
{
namespace chassis
{
/** When the turret yaw setpoint and measured value is < M_PI - this value, autorotation will be
 * paused until the difference is within this value again. */
static constexpr float TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION = modm::toRadian(1.0f);

static constexpr uint32_t LAZY_ROTATION_TIMEOUT_MS = 500;
/**
 * Various modes for autorotation. Strict means autorotation is executed immediately. Lazy means a
 * timeout runs between when desired rotation ends and autorotation begins. Keep chassis angle only
 * rotates the chassis to how it needs to translate, and leaves it there indefinitely.
 *
 */
enum AutorotationMode : uint8_t
{
    STRICT_PLATE_FORWARD = 0,
    LAZY_PLATE_FORWARD,
    STRICT_SIDE_FORWARD,
    LAZY_SIZE_FORWARD,
    KEEP_CHASSIS_ANGLE,
    NUM_AUTOROTATION_STATES,  // This automatically is the number of states since we start at 0
};

class BalancingChassisAutorotateCommand : public tap::control::Command
{
public:
    BalancingChassisAutorotateCommand(
        tap::Drivers* drivers,
        BalancingChassisSubsystem* chassis,
        aruwsrc::control::ControlOperatorInterface& operatorInterface,
        const aruwsrc::control::turret::TurretMotor* yawMotor);

    const char* getName() const override { return "Balancing Chassis Autorotation Drive Command"; }

    void initialize() override;

    void execute() override;

    void end(bool interrupted) override;

    bool isFinished() const override;

    void setAutorotationMode(AutorotationMode mode)
    {
        if (mode < NUM_AUTOROTATION_STATES)
        {
            this->autorotationMode = mode;
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

    tap::arch::MilliTimeout lazyTimeout = tap::arch::MilliTimeout(LAZY_ROTATION_TIMEOUT_MS);

    uint32_t prevTime = 0;

    AutorotationMode autorotationMode = KEEP_CHASSIS_ANGLE;

    modm::Vector2f motionDesiredTurretRelative;

    const float maxAngleFromCenter = M_PI_2;
    // 180 degree symmetry guaranteed for balacing standards as of 2023

    /** Autorotation setpoint, smoothed using a low pass filter. */
    float desiredRotationAverage = 0;

    float DESIRED_ROTATION_SCALAR = -.002;

    /**
     * `true` if the chassis is currently actually autorotating, `false` otherwise
     * (in which case on rotation may happen). Autorotation may not happen if the
     * user requests a user input that moves the turret from the front of the chassis
     * to the back. If the chassis front and back are identical, then there is no
     * reason to autorotate until the turret is done turning around.
     */
    bool chassisAutorotating;

    /**
     * `true` if the chassis's rotation is being used for motion planning. Rotation of the chassis
     * for motion planning superceeds rotation of the chassis for autorotation.
     */
    bool chassisMotionPlanning;

    float debug1;
    float debug2;
};  // class BalancingChassisAutorotateCommand

}  // namespace chassis

}  // namespace aruwsrc

#endif  // BALANCING_CHASSIS_AUTOROTATE_COMMAND_HPP_