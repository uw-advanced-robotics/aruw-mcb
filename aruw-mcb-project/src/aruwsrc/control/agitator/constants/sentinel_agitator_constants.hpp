/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#if defined(TARGET_SENTINEL)
    // position PID terms
    // PID terms for sentinel
    static constexpr float PID_17MM_P = 120000.0f;
    static constexpr float PID_17MM_I = 0.0f;
    static constexpr float PID_17MM_D = 50.0f;
    static constexpr float PID_17MM_MAX_ERR_SUM = 0.0f;
    static constexpr float PID_17MM_MAX_OUT = 16000.0f;

    static constexpr tap::motor::MotorId AGITATOR_MOTOR_ID = tap::motor::MOTOR7;
    static constexpr tap::can::CanBus AGITATOR_MOTOR_CAN_BUS = tap::can::CanBus::CAN_BUS1;
#endif