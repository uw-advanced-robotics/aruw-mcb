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

#ifndef DRONE_TURRET_VECTOR_COMMAND_HPP_
#define DRONE_TURRET_VECTOR_COMMAND_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/algorithms/transforms/transform.hpp"
#include "tap/control/command.hpp"

#include "aruwsrc/control/control_operator_interface.hpp"
#include "aruwsrc/control/turret/algorithms/chassis_frame_turret_controller.hpp"
#include "aruwsrc/robot/drone/drone_imu.hpp"
#include "aruwsrc/robot/drone/drone_turret_subsystem.hpp"

namespace aruwsrc::drone
{
class DroneTurretVectorCommand final : public tap::control::Command
{
public:
    DroneTurretVectorCommand(
        aruwsrc::control::ControlOperatorInterface &controlOperatorInterface,
        DroneTurretSubsystem &turret,
        const DroneIMU &turretImu,
        tap::algorithms::SmoothPid &yawPositionPid,
        tap::algorithms::SmoothPid &yawVelocityPid,
        tap::algorithms::SmoothPid &pitchPositionPid,
        tap::algorithms::SmoothPid &pitchVelocityPid,
        aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
            aruwsrc::control::turret::algorithms::Axis::YAW> &chassisFrameYawController,
        aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
            aruwsrc::control::turret::algorithms::Axis::PITCH> &chassisFramePitchController,
        float userYawInputScalar,
        float userPitchInputScalar,
        uint8_t turretID = 0);

    const char *getName() const override { return "Drone turret vector control"; }

    bool isReady() override;

    void initialize() override;

    void execute() override;

    bool isFinished() const override;

    void end(bool interrupted) override;

private:
    using Vector = tap::algorithms::transforms::Vector;

    Vector getTurretForwardVectorWorldFrame() const;
    Vector getTurretPitchAxisWorldFrame() const;
    Vector getTurretYawAxisWorldFrame(
        Vector turretForwardWorldFrame,
        Vector turretPitchAxisWorldFrame) const;
    Vector rotateVector(Vector vector, Vector axis, float angle) const;
    Vector normalizeOrFallback(Vector vector, Vector fallback) const;
    float clampControllerError(
        float error,
        const aruwsrc::control::turret::TurretMotor &turretMotor) const;
    float limitUserInputAtMotorLimits(
        float input,
        const aruwsrc::control::turret::TurretMotor &turretMotor) const;
    void runWorldFrameControl(float yawInput, float pitchInput, float dt);
    void runChassisFrameFallback(float yawInput, float pitchInput, float dt);
    void resetChassisFrameFallback();
    bool stopAtMotorLimits(
        float &motorOutput,
        const aruwsrc::control::turret::TurretMotor &turretMotor) const;

    static constexpr float AXIS_SOLVE_DAMPING = 0.05f;
    static constexpr float MAX_CONTROLLER_ERROR = 0.35f;
    static constexpr float MIN_YAW_AXIS_AUTHORITY = 0.15f;
    static constexpr float LIMIT_INPUT_BUFFER = 0.02f;

    aruwsrc::control::ControlOperatorInterface &controlOperatorInterface;
    DroneTurretSubsystem &turret;
    const DroneIMU &turretImu;
    tap::algorithms::SmoothPid &yawPositionPid;
    tap::algorithms::SmoothPid &yawVelocityPid;
    tap::algorithms::SmoothPid &pitchPositionPid;
    tap::algorithms::SmoothPid &pitchVelocityPid;
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
        aruwsrc::control::turret::algorithms::Axis::YAW> &chassisFrameYawController;
    aruwsrc::control::turret::algorithms::ChassisFrameTurretController<
        aruwsrc::control::turret::algorithms::Axis::PITCH> &chassisFramePitchController;
    float userYawInputScalar;
    float userPitchInputScalar;
    uint8_t turretID;

    Vector desiredForwardWorldFrame = Vector(1.0f, 0.0f, 0.0f);
    Vector lastPitchInputAxisWorldFrame = Vector(0.0f, 1.0f, 0.0f);
    Vector lastYawAxisWorldFrame = Vector(0.0f, 0.0f, 1.0f);
    bool usingChassisFrameFallback = false;
    uint32_t prevTime = 0;
};
}  // namespace aruwsrc::drone

#endif  // DRONE_TURRET_VECTOR_COMMAND_HPP_
