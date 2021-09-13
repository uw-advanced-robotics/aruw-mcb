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

#ifndef BNO055_INTERFACE_HPP_
#define BNO055_INTERFACE_HPP_

#ifndef PLATFORM_HOSTED
#include "modm/driver/inertial/bno055.hpp"
#endif

#include "tap/algorithms/MahonyAHRS.h"
#include "tap/architecture/timeout.hpp"
#include "tap/board/board.hpp"

#include "modm/processing.hpp"

namespace aruwsrc::sensors
{
/**
 * An interface for the BNO055 that reads raw acc/gyro/mag
 * data from the sensor and uses the Mahony ARHS algorithm
 * to find euler angles.
 */
class Bno055Interface : public modm::pt::Protothread
{
public:
#ifndef PLATFORM_HOSTED
    using Bno055I2CMasterScl = GpioF1;
    using Bno055I2CMasterSda = GpioF0;
    using Bno055I2CMaster = I2cMaster2;
#endif

    static constexpr uint8_t BNO055_ADDR = 0x28;

    Bno055Interface();

    void initialize();

    bool update();

    /**
     * For optimal performance, call at 500 Hz.
     */
    void periodicIMUUpdate();

    inline float getYaw() { return ahrsAlgorithm.getYaw(); }
    inline float getRoll() { return ahrsAlgorithm.getRoll(); }
    inline float getPitch() { return ahrsAlgorithm.getPitch(); }
    /**
     * @note: Units in deg/s
     */
    inline float getGx() const { return r.gyroscope[0] / LSB_PER_DEGREE; };
    inline float getGy() const { return r.gyroscope[1] / LSB_PER_DEGREE; };
    inline float getGz() const { return r.gyroscope[2] / LSB_PER_DEGREE; };
    /**
     * @note: Units in m/s^2
     */
    inline float getAx() const { return r.acceleration[0] / 100.0f; };
    inline float getAy() const { return r.acceleration[1] / 100.0f; };
    inline float getAz() const { return r.acceleration[2] / 100.0f; };
    inline float getMx() const { return r.magnetometer[0] / LSB_PER_MICRO_T; }
    inline float getMy() const { return r.magnetometer[1] / LSB_PER_MICRO_T; }
    inline float getMz() const { return r.magnetometer[2] / LSB_PER_MICRO_T; }
    inline bool isReady() const { return ready; }

private:
    /**
     * See table 3-29 of BNO055 datasheet:
     * https://cdn-shop.adafruit.com/datasheets/BST_BNO055_DS000_12.pdf
     */
    static constexpr float LSB_PER_DEGREE = 16.0f;

    static constexpr float LSB_PER_MICRO_T = 100.0f;

    modm_packed struct RawData
    {
        int16_t acceleration[3];
        int16_t magnetometer[3];
        int16_t gyroscope[3];
    } r;

    static constexpr int DELAY_BTWN_CALC_AND_READ_REG = 1550;

    tap::arch::MicroTimeout timer;

#ifndef PLATFORM_HOSTED
    modm::bno055::Data unusedData;
    modm::Bno055<Bno055I2CMaster> imu;
#endif

    Mahony ahrsAlgorithm;

    bool ready;
};
}  // namespace aruwsrc::sensors

#endif  // BNO055_INTERFACE_HPP_
