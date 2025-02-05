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

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/architecture/periodic_timer.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"
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
        RF_CALL_BLOCKING(writeRegister(CTRL1_XL, (uint8_t) ODR_6660HZ | (uint8_t) G8_CONFIG));
        RF_CALL_BLOCKING(writeRegister(CTRL2_G, (uint8_t) ODR_6660HZ | (uint8_t) DPS2000_CONFIG));

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
        readWorking = RF_CALL_BLOCKING(readRegister(OUT_TEMP_L, READ_LENGTH, rxBuff));
        if (!readWorking)
        {
            return;
        }
        imuData.temperature = tempValueToCelsius(rxBuff);

        imuData.gyroRaw[ImuData::X] = gyroValueToDegPerSec(rxBuff + 2);
        imuData.gyroRaw[ImuData::Y] = gyroValueToDegPerSec(rxBuff + 4);
        imuData.gyroRaw[ImuData::Z] = gyroValueToDegPerSec(rxBuff + 6);

        imuData.accRaw[ImuData::X] = accelValueToG(rxBuff + 8);
        imuData.accRaw[ImuData::Y] = accelValueToG(rxBuff + 10);
        imuData.accRaw[ImuData::Z] = accelValueToG(rxBuff + 12);
    }

    void setAccelRange(XL_Config xl_config)
    {
        RF_CALL_BLOCKING(readRegister(CTRL1_XL, READ_LENGTH, &current_reg_XL));

        RF_CALL_BLOCKING(writeRegister(CTRL1_XL, (current_reg_XL &  (uint8_t) G_CONFIG_BITMASK) | (uint8_t) xl_config));
        switch (xl_config)
        {
            case G2_CONFIG:
                accelScale = 0.061;
                break;
            case G4_CONFIG:
                accelScale = 0.122;
                break;
            case G8_CONFIG:
                accelScale = 0.244;
                break;
            case G16_CONFIG:
                accelScale = 0.488;
                break;
            default:
                break;
        }
    }

    void setGyroRange(Gyro_Config g_config)
    {
        RF_CALL_BLOCKING(readRegister(CTRL2_G, READ_LENGTH, &current_reg_G));

        RF_CALL_BLOCKING(writeRegister(CTRL2_G, (current_reg_G & (uint8_t) DPS_CONFIG_BITMASK) | (uint8_t) g_config));
        switch (g_config)
        {
            case DPS250_CONFIG:
                gyroScale = 8.75;
                break;
            case DPS500_CONFIG:
                gyroScale = 17.50;
                break;
            case DPS1000_CONFIG:
                gyroScale = 35;
                break;
            case DPS2000_CONFIG:
                gyroScale = 70;
                break;
            default:
                break;
        }
    }

    void updateODR(ODR odr)
    {  // Takes in ODR in Hz
        RF_CALL_BLOCKING(readRegister(CTRL2_G, READ_LENGTH, current_reg_G));
        RF_CALL_BLOCKING(readRegister(CTRL1_XL, READ_LENGTH, current_reg_XL));

        RF_CALL_BLOCKING(writeRegister(CTRL1_XL, (current_reg_XL & (uint8_t) ODR_BITMASK) | (uint8_t) odr));
        RF_CALL_BLOCKING(writeRegister(CTRL2_G, (current_reg_G & (uint8_t) ODR_BITMASK) | (uint8_t) odr));
        switch (odr)
        {
            case ODR_416HZ:
                timeout = 3;
                break;
            case ODR_833HZ:
                timeout = 2;
                break;
            case ODR_1660HZ:
                timeout = 1;
                break;
            case ODR_3330HZ:
                timeout = 1;
                break;
            case ODR_6660HZ:
                timeout = 1;
                break;
            default:
                break;
        }
    }

    ImuData imuData;  // TODO: remove

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

    uint8_t current_reg_G;
    uint8_t current_reg_XL;

    tap::arch::PeriodicMilliTimer updateTimeout;

    // ImuData imuData;

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

    float accelScale = 0.488;
    float accelValueToG(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * accelScale / 1000.0f;
    }

    float gyroScale = 70;
    float gyroValueToDegPerSec(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * gyroScale / 1000.0f;
    }

    float tempValueToCelsius(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return (raw / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;
    }
};
}  // namespace aruwsrc::communication::sensors::imu

#endif  // ISM330_HPP_
