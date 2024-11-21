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

#ifndef ISM330_HPP_
#define ISM330_HPP_

#include "tap/architecture/periodic_timer.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/processing/resumable.hpp"

#include "ism330_data.hpp"

namespace aruwsrc::communication::sensors::imu
{

template <class I2cMaster>
class ISM330 : public modm::I2cDevice<I2cMaster>
{
public:
    ISM330() : modm::I2cDevice<I2cMaster>(DEVICE_ADDRESS), updateTimeout(timeout) {}

    void init()
    {
        RF_CALL_BLOCKING(writeRegister(CTRL1_XL, ACCELEROMETER_CONFIG));
        RF_CALL_BLOCKING(writeRegister(CTRL2_G, GYRO_CONFIG));

        // Check Who Am I
        RF_CALL_BLOCKING(readRegister(0x0F, 3, rxConfig));
    }

    void read()
    {
        if (!updateTimeout.execute())
        {
            return;
        }

        updateTimeout.restart(timeout);

        pinged = RF_CALL_BLOCKING(this->ping());

        readWorking = RF_CALL_BLOCKING(readRegister(READ_START, READ_LENGTH, rxBuff));
        processData();
    }

    void processData()
    {
        if (!readWorking)
        {
            return;
        }

        imuData.temperature = (bigEndianInt16ToFloat(rxBuff) / 256.0f) + 25.0f;

        imuData.gyroRaw[ImuData::X] = gyroValueToDegPerSec(rxBuff + 2);
        imuData.gyroRaw[ImuData::Y] = gyroValueToDegPerSec(rxBuff + 4);
        imuData.gyroRaw[ImuData::Z] = gyroValueToDegPerSec(rxBuff + 6);

        imuData.accRaw[ImuData::X] = accelValueToG(rxBuff + 8);
        imuData.accRaw[ImuData::Y] = accelValueToG(rxBuff + 10);
        imuData.accRaw[ImuData::Z] = accelValueToG(rxBuff + 12);
    }

private:
    modm::ResumableResult<bool> readRegister(uint8_t reg, int length, uint8_t *rxBuffer)
    {
        RF_BEGIN();

        txBuff[0] = reg;

        RF_WAIT_WHILE(!this->transaction.configureWriteRead(txBuff, 1, rxBuffer, length));

        RF_END_RETURN_CALL(this->runTransaction());
    };

    modm::ResumableResult<bool> writeRegister(uint8_t reg, uint8_t data)
    {
        RF_BEGIN();

        txBuff[0] = reg;
        txBuff[1] = data;

        RF_WAIT_WHILE(!this->transaction.configureWrite(txBuff, 2));

        RF_END_RETURN_CALL(this->runTransaction());
    };

    bool pinged;
    bool readWorking;

    uint8_t txBuff[2];
    uint8_t rxBuff[30];

    uint8_t rxConfig[10];

    int timeout = 2;

    tap::arch::PeriodicMilliTimer updateTimeout;

    struct ImuData
    {
        enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float gyroRaw[3] = {};
        float accRaw[3] = {};
        float temperature;
    };

    ImuData imuData;

    /**
     * Convert int16_t stored in big endian format in buff to a floating point value.
     *
     * @param[in] buff Buffer containing two bytes representing an int16_t in big endian format.
     * @return A float, the converted int16_t in floating point form.
     */
    inline float bigEndianInt16ToFloat(const uint8_t *buff)
    {
        return static_cast<float>(static_cast<int16_t>((*(buff)) | (*(buff + 1) << 8)));
    }

    // We at 250DPS
    float gryoScale = 8.75f;
    float gyroValueToDegPerSec(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * gryoScale / 1000.0f;
    }

    float accelScale = 0.061f;
    float accelValueToG(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * accelScale / 1000.0f;
    }
};

}  // namespace aruwsrc::communication::sensors::imu

#endif  // ISM330_HPP_
