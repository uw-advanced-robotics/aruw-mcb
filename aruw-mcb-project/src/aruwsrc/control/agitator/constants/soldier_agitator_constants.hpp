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
#include "modm/math/geometry/angle.hpp"

// Do not include this file directly, use agitator_consants.hpp

namespace aruwsrc::control::agitator::constants{
    // position PID terms
    // PID terms for soldier
    static constexpr float PID_17MM_P = 300'000.0f;
    static constexpr float PID_HOPPER_P = 100000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 50.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::can::CanBus AGITATOR_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;

    static constexpr bool isAgitatorInverted = false;

    /**
     * The jamming constants. Agitator is considered jammed if difference between setpoint
     * and current angle is > `JAMMING_DISTANCE` radians for >= `JAMMING_TIME` ms;
     *
     * @warning: `JAMMING_DISTANCE` must be less than the smallest movement command
     *
     * This should be positive or else weird behavior can occur
     */
    static constexpr float AGITATOR_JAMMING_DISTANCE = M_PI / 20;
    static constexpr uint32_t JAMMING_TIME = 250;
}
