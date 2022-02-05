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
#include "tap/motor/dji_motor.hpp"

// Do not include this file directly, use agitator_consants.hpp

namespace aruwsrc::control::agitator::constants{
     // Hero's waterwheel constants
    static constexpr float PID_HERO_WATERWHEEL_P = 100000.0f;
    static constexpr float PID_HERO_WATERWHEEL_I = 0.0f;
    static constexpr float PID_HERO_WATERWHEEL_D = 10.0f;
    static constexpr float PID_HERO_WATERWHEEL_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_HERO_WATERWHEEL_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId HERO_WATERWHEEL_MOTOR_ID = tap::motor::MOTOR3;
    static constexpr tap::can::CanBus HERO_WATERWHEEL_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr bool HERO_WATERWHEEL_INVERTED = true;

    // PID terms for the hero kicker
    static constexpr float PID_HERO_KICKER_P = 50000.0f;
    static constexpr float PID_HERO_KICKER_I = 0.0f;
    static constexpr float PID_HERO_KICKER_D = 10.0f;
    static constexpr float PID_HERO_KICKER_MAX_ERR_SUM = 0.0f;
    // max out added by Tenzin since it wasn't here. This should
    // also be changed by someone who know's what they're doing!
    static constexpr float PID_HERO_KICKER_MAX_OUT = 16000.0f;

    // There are two kicker motors that drive the shaft.
    static constexpr tap::motor::MotorId HERO_KICKER1_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::motor::MotorId HERO_KICKER2_MOTOR_ID = tap::motor::MOTOR8;
    static constexpr tap::can::CanBus HERO_KICKER1_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr tap::can::CanBus HERO_KICKER2_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
    static constexpr bool HERO_KICKER_INVERTED = false;

    /**
     * The jamming constants for waterwheel. Waterwheel is considered jammed if difference between
     * setpoint and current angle is > `JAM_DISTANCE_TOLERANCE_WATERWHEEL` radians for >=
     * `JAM_TEMPORAL_TOLERANCE_WATERWHEEL` ms;
     */
    static constexpr float JAM_DISTANCE_TOLERANCE_WATERWHEEL = M_PI / 14.0f;
    static constexpr uint32_t JAM_TEMPORAL_TOLERANCE_WATERWHEEL = 100.0f;
}
