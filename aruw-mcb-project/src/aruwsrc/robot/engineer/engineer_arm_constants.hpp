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

#ifndef ENGINEER_ARM_CONSTANTS_HPP_
#define ENGINEER_ARM_CONSTANTS_HPP_

#include "arm/joint_subsystem.hpp"

namespace aruwsrc::engineer::arm
{
static constexpr JointSubsystemConfig xAxisConfig = {
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .maxErrorSum = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .feedforward = 0,
    .setpointTolerance = 0.0f,
    .setpointToEncoderScalar = 1.0f,
    .lowerBound = 0.0f,
    .upperBound = 1000.0f,
};

static constexpr JointSubsystemConfig LiftConfig = {
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .maxErrorSum = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .feedforward = 1,  // TODO: Find out correct value for this
    .setpointTolerance = 0.0f,
    .setpointToEncoderScalar = 1.0f,
    .lowerBound = 0.0f,
    .upperBound = 1000.0f,
};

static constexpr JointSubsystemConfig yawConfig = {
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .maxErrorSum = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .feedforward = 0,
    .setpointTolerance = 0.0f,
    .setpointToEncoderScalar = 1.0f,
    .lowerBound = -135.0f,
    .upperBound = 135.0f,
};

static constexpr JointSubsystemConfig pitchConfig = {
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .maxErrorSum = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .feedforward = 0,  // TODO: Figure out correct value for this
    .setpointTolerance = 0.0f,
    .setpointToEncoderScalar = 1.0f,
    .lowerBound = -90.0f,
    .upperBound = 0.0f};

static constexpr JointSubsystemConfig rollConfig = {
    .p = 0.0f,
    .i = 0.0f,
    .d = 0.0f,
    .maxErrorSum = 0.0f,
    .maxOutput = tap::motor::DjiMotor::MAX_OUTPUT_C620,
    .feedforward = 0,
    .setpointTolerance = 0.0f,
    .setpointToEncoderScalar = 1.0f,
    .lowerBound = -45.0f,
    .upperBound = 45.0f};

static constexpr Position homePosition = {
    .lift = 0.0f,
    .reach = 0.0f,
    .yaw = 0.0f,
    .pitch = 0.0f,
    .roll = 0.0f,
};

static constexpr Position intakePosition = {
    .lift = 0.0f,
    .reach = 0.0f,
    .yaw = 0.0f,
    .pitch = 0.0f,
    .roll = 0.0f,
};

static constexpr Position centerTunnelPosition = {
    .lift = 0.0f,
    .reach = 0.0f,
    .yaw = 0.0f,
    .pitch = 0.0f,
    .roll = 0.0f,
};

}  // namespace aruwsrc::engineer::arm

#endif
