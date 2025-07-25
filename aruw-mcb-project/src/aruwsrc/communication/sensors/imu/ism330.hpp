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
#include "tap/architecture/timeout.hpp"
#include "tap/communication/sensors/imu/abstract_imu.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/architecture/interface/register.hpp"
#include "modm/math/utils.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

#include "ism330_data.hpp"

namespace aruwsrc::communication::sensors::imu::ism330
{
using namespace tap::communication::sensors::imu;

/**
 * I2C driver for the ISM330DHCX IMU
 *
 * For registers and datasheet, go to:
 * https://www.st.com/en/mems-and-sensors/ism330dhcx.html
 */
template <class I2cMaster>
class ISM330 : public modm::I2cDevice<I2cMaster>, public AbstractIMU, public modm::pt::Protothread
{
public:
    ISM330();

    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);

    bool read();

    void setAccelRange(AccelerometerRangeConfig xl_config);
    void setGyroRange(GyroscopeRangeConfig g_config);
    void setODR(OutputDataRate odr);

    virtual inline const char *getName() const { return "ISM330DHCX"; }
    virtual inline float getAccelerationSensitivity() const override { return GRAVITY_MPS2; }

private:
    modm::ResumableResult<bool> configureWriteReadWithTimeout(
        const uint8_t *writeBuffer,
        std::size_t writeSize,
        uint8_t *readBuffer,
        std::size_t readSize)
    {
        bool success;
        RF_BEGIN();

        // if (errorTimeout.execute())
        // if (true)
        // {
        //     erroredOut = true;
        //     RF_RETURN(true);
        // }
        success =
            this->transaction.configureWriteRead(writeBuffer, writeSize, readBuffer, readSize);
        // if (success)
        // {
        //     erroredOut = false;
        //     errorTimeout.restart(errorTimeoutTime);
        // }
        RF_END_RETURN(success);
    };

    modm::ResumableResult<bool> readRegister(uint8_t reg, int length, uint8_t *rxBuffer)
    {
        txBuff[0] = reg;

        RF_BEGIN();

        // errorTimeout.restart(errorTimeoutTime);
        RF_WAIT_UNTIL(RF_CALL(this->configureWriteReadWithTimeout(txBuff, 1, rxBuffer, length)));
        RF_END_RETURN_CALL(this->runTransaction());
    };

    modm::ResumableResult<bool> writeRegister(uint8_t reg, uint8_t data)
    {
        txBuff[0] = reg;
        txBuff[1] = data;

        RF_BEGIN();

        RF_WAIT_UNTIL(this->transaction.configureWrite(txBuff, 2));

        RF_END_RETURN_CALL(this->runTransaction());
    };

    bool pinged;

    uint8_t rxBuff[15];
    uint8_t txBuff[2];

    uint32_t timeout = 1200;
    uint32_t errorTimeoutTime = timeout * 400;

    // tap::arch::PeriodicMicroTimer errorTimeout;
    // bool erroredOut = false;
    bool debug = false;

    uint8_t current_reg_G;
    uint8_t current_reg_XL;

    float gyroScale;
    float accelScale;

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

    float gyroValueToRadPerSec(const uint8_t *buff)
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
}  // namespace aruwsrc::communication::sensors::imu::ism330

#include "ism330_impl.hpp"

#endif  // ISM330_HPP_
