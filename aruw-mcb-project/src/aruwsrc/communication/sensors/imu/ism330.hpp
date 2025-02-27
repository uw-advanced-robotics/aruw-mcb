/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"
#include "modm/processing/resumable.hpp"

#include "ism330_data.hpp"

namespace aruwsrc::communication::sensors::imu
{
template <class I2cMaster>
class ISM330 : public modm::I2cDevice<I2cMaster>,
               public tap::communication::sensors::imu::AbstractIMU
{
public:
    ISM330();

    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);

    void read();

    void setAccelRange(XL_Config xl_config);
    void setGyroRange(Gyro_Config g_config);

    void setODR(ODR odr);

    virtual inline const char *getName() const { return "ISM330DHCX"; }
    virtual inline float getAccelerationSensitivity() { return 9.8f; }

private:
    modm::ResumableResult<bool> readRegister(uint8_t reg, int length, uint8_t *rxBuffer)
    {
        uint8_t txBuff = reg;

        RF_BEGIN();

        RF_WAIT_WHILE(!this->transaction.configureWriteRead(&txBuff, 1, rxBuffer, length));

        RF_END_RETURN_CALL(this->runTransaction());
    };

    modm::ResumableResult<bool> writeRegister(uint8_t reg, uint8_t data)
    {
        uint8_t txBuff[2] = {reg, data};

        RF_BEGIN();

        RF_WAIT_WHILE(!this->transaction.configureWrite(txBuff, 2));

        RF_END_RETURN_CALL(this->runTransaction());
    };

    bool pinged;

    uint8_t rxConfig[10];

    int timeout = 1200;

    uint8_t current_reg_G;
    uint8_t current_reg_XL;

    float gyroScale = 70;
    float accelScale{0.488};

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

    float accelValueToMeterPerSec(const uint8_t *buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * accelScale / 1000.0f * getAccelerationSensitivity();
    }

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

    bool inited = false;

    int count = 0, succcess = 0;
};
}  // namespace aruwsrc::communication::sensors::imu

#include "ism330_impl.hpp"

#endif  // ISM330_HPP_
