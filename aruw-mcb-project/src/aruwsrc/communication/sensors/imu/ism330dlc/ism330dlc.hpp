/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ISM330DLC_HPP_
#define ISM330DLC_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"

class Ism330dlc : public tap::communication::sensors::imu::ImuInterface {
    struct ImuData {
        enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float accRaw[3] = {};
        float gyroRaw[3] = {};
        float accOffsetRaw[3] = {};
        float gyroOffsetRaw[3] = {};
        float accG[3] = {};
        float gyroDegPerSec[3] = {};

        float temperature;
    } data;

    inline const char *getName() const;
    inline float getAx();
    inline float getAy();
    inline float getAz();
    inline float getGx();
    inline float getGy();
    inline float getGz();
    inline float getTemp();
    inline float getYaw();
    inline float getPitch();
    inline float getRoll();

private:
    ImuState imuState = ImuState::IMU_NOT_CONNECTED;
};

#endif //ISM330DLC_HPP_
