/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BALSTD_CHASSIS_CONSTANTS_HPP_
#define BALSTD_CHASSIS_CONSTANTS_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/motor/dji_motor.hpp"

#include "aruwsrc/control/motor/tmotor_ak80_9_encoder.hpp"
#include "controllers/balance_controller.hpp"

#include "balstd_leg.hpp"

namespace aruwsrc::control::balstd
{
static constexpr float OUTER_HARD_STOP = modm::toRadian(-20);
static constexpr float OUTER_SOFT_STOP = modm::toRadian(-15);

static constexpr float INNER_SOFT_STOP = modm::toRadian(90);

static constexpr int32_t FRONT_HIP_MOTOR_HOME = static_cast<int32_t>(
    -OUTER_HARD_STOP / M_TWOPI * aruwsrc::control::motor::Tmotor_AK809Encoder::ENC_RESOLUTION * 9);
static constexpr int32_t BACK_HIP_MOTOR_HOME = static_cast<int32_t>(
    (OUTER_HARD_STOP - M_PI) / M_TWOPI *
    aruwsrc::control::motor::Tmotor_AK809Encoder::ENC_RESOLUTION * 9);

BalstdLegConfig LEG_CONFIG{
    .upperLinkLength = 0.15,
    .lowerLinkLength = 0.25,
    .fixedLinkLength = 0.108,
    .frontHipOuterLimit = OUTER_SOFT_STOP,
    .frontHipInnerLimit = INNER_SOFT_STOP,
    .backHipOuterLimit = M_PI - OUTER_SOFT_STOP,
    .backHipInnerLimit = INNER_SOFT_STOP,
};

tap::algorithms::SmoothPidConfig HEIGHT_CONTROLLER_PID_CONFIG{
    .kp = 500.0f,
    .ki = 0.0f,
    .kd = -20.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 90.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

tap::algorithms::SmoothPidConfig SPLIT_CONTROLLER_PID_CONFIG{
    .kp = 50.0f,
    .ki = 0.0f,
    .kd = 20.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 4.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

tap::algorithms::SmoothPidConfig ROLL_CONTROLLER_PID_CONFIG{
    .kp = 500.0f,
    .ki = 0.0f,
    .kd = 20.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 20.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

tap::algorithms::SmoothPidConfig YAW_CONTROLLER_PID_CONFIG{
    .kp = 10.0f,
    .ki = 0.0f,
    .kd = -2.0f,
    .maxICumulative = 0.0f,
    .maxOutput = 1.0f,
    .tQDerivativeKalman = 1.0f,
    .tRDerivativeKalman = 0.0f,
    .tQProportionalKalman = 1.0f,
    .tRProportionalKalman = 0.0f,
    .errDeadzone = 0.0f,
    .errorDerivativeFloor = 0.0f,
};

aruwsrc::control::balstd::BalanceController::Config BALANCE_CONTROLLER_CONFIG{
    .heightControllerConfig = HEIGHT_CONTROLLER_PID_CONFIG,
    .splitControllerConfig = SPLIT_CONTROLLER_PID_CONFIG,
    .rollControllerConfig = ROLL_CONTROLLER_PID_CONFIG,
    .yawControllerConfig = YAW_CONTROLLER_PID_CONFIG,
    .minHeight = 0.11,
    .maxHeight = 0.2,  // lowball until tested
};

}  // namespace aruwsrc::control::balstd

#endif  // BALSTD_CHASSIS_CONSTANTS_HPP_
