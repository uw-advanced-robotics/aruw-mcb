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
#include "tap/algorithms/math_user_utils.hpp"
#include "modm/processing/resumable.hpp"
#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"

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
        setAccelRange(2);
        setGyroRange(250);

        // Check Who Am I
        RF_CALL_BLOCKING(readRegister(WHO_AM_I, 3, rxConfig));
    }

    void readAndProcessData()
    {
        if (!updateTimeout.execute())
        {
            return;
        }

        updateTimeout.restart(timeout);

        pinged = RF_CALL_BLOCKING(this->ping());

        // Read temp
        readWorking = RF_CALL_BLOCKING(readRegister(OUT_TEMP_H, READ_LENGTH, rxBuff[0]));
        if (!readWorking)
        {
            return;
        }
        RF_CALL_BLOCKING(readRegister(OUT_TEMP_L, READ_LENGTH, rxBuff[1]));
        imuData.temperature = (bigEndianInt16ToFloat(rxBuff) / 256.0f) + 25.0f; //where tf

        // Read gyro
        RF_CALL_BLOCKING(readRegister(OUTX_L_G, READ_LENGTH, rxBuff[0]));
        RF_CALL_BLOCKING(readRegister(OUTX_H_G, READ_LENGTH, rxBuff[1]));
        imuData.gyroRaw[ImuData::X] = gyroValueToDegPerSec(rxBuff);

        RF_CALL_BLOCKING(readRegister(OUTY_L_G, READ_LENGTH, rxBuff[0]));
        RF_CALL_BLOCKING(readRegister(OUTY_H_G, READ_LENGTH, rxBuff[1]));
        imuData.gyroRaw[ImuData::Y] = gyroValueToDegPerSec(rxBuff);

        RF_CALL_BLOCKING(readRegister(OUTZ_L_G, READ_LENGTH, rxBuff[0]));
        RF_CALL_BLOCKING(readRegister(OUTZ_H_G, READ_LENGTH, rxBuff[1]));
        imuData.gyroRaw[ImuData::Z] = gyroValueToDegPerSec(rxBuff);

        // Read accel
        RF_CALL_BLOCKING(readRegister(OUTX_L_XL, READ_LENGTH, rxBuff[0]));
        RF_CALL_BLOCKING(readRegister(OUTX_H_XL, READ_LENGTH, rxBuff[1]));
        imuData.accRaw[ImuData::X] = accelValueToG(rxBuff);

        RF_CALL_BLOCKING(readRegister(OUTY_L_XL, READ_LENGTH, rxBuff[0]));
        RF_CALL_BLOCKING(readRegister(OUTY_H_XL, READ_LENGTH, rxBuff[1]));
        imuData.accRaw[ImuData::Y] = accelValueToG(rxBuff);

        RF_CALL_BLOCKING(readRegister(OUTZ_L_XL, READ_LENGTH, rxBuff[0]));
        RF_CALL_BLOCKING(readRegister(OUTZ_H_XL, READ_LENGTH, rxBuff[1]));
        imuData.accRaw[ImuData::Z] = accelValueToG(rxBuff);
    }

    void setAccelRange(int num) {
        uint8_t current_reg;
        RF_CALL_BLOCKING(readRegister(CTRL1_XL, READ_LENGTH, current_reg));
        switch(num) {
            case 2:
                RF_CALL_BLOCKING(writeRegister(CTRL1_XL, current_reg & G2_CONFIG));
                accelScale = 0.061;
                break;
            case 4:
                RF_CALL_BLOCKING(writeRegister(CTRL1_XL, current_reg & G4_CONFIG));
                accelScale = 0.122;
                break;
            case 8:
                RF_CALL_BLOCKING(writeRegister(CTRL1_XL, current_reg & G8_CONFIG));
                accelScale = 0.244;
                break;
            case 16:
                RF_CALL_BLOCKING(writeRegister(CTRL1_XL, current_reg & G16_CONFIG));
                accelScale = 0.488;
                break;
            default:
                break;
        }
    }

    void setGyroRange(int num) {
        uint8_t current_reg;
        RF_CALL_BLOCKING(readRegister(CTRL2_G, READ_LENGTH, current_reg));
        switch(num) {
            case 250:
                RF_CALL_BLOCKING(writeRegister(CTRL2_G, current_reg & DPS250_CONFIG));
                gyroScale = 8.75f;
                break;
            case 500:
                RF_CALL_BLOCKING(writeRegister(CTRL2_G, current_reg & DPS500_CONFIG));
                gyroScale = 17.50f;
                break;
            case 1000:
                RF_CALL_BLOCKING(writeRegister(CTRL2_G, current_reg & DPS1000_CONFIG));
                gyroScale = 35f;
                break;
            case 2000:
                RF_CALL_BLOCKING(writeRegister(CTRL2_G, current_reg & DPS2000_CONFIG));
                gyroScale = 70f;
                break;
            default:
                break;
        }
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

    float accelScale;
    float accelValueToG(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * accelScale / 1000.0f;
    }

    float gyroScale;
    float gyroValueToDegPerSec(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * gyroScale / 1000.0f;
    }
};
} // namespace aruwsrc::communication::sensors::imu

#endif  // ISM330_HPP_
