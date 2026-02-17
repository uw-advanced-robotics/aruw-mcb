/*
 * Copyright (c) 2025-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/sensors/imu/ism330/ism330_data.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"

namespace aruwsrc::communication::sensors::imu::ism330
{
using namespace tap::communication::sensors::imu;

class ISM330 : public AbstractIMU, public modm::pt::Protothread
{
public:
    enum class ChipSelect
    {
        // Uses Board::SpiNss (GpioE4).
        PRIMARY_SPI_NSS,
        // Uses Board::DigitalOutPinF (GpioD14).
        SECONDARY_DIGITAL_OUT_F,
    };

    explicit ISM330(ChipSelect chipSelect = ChipSelect::PRIMARY_SPI_NSS);
    DISALLOW_COPY_AND_ASSIGN(ISM330);
    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);

    /**
     * Read data from the imu. This is a protothread that reads the SPI bus using
     * nonblocking I/O.
     *
     * @return `true` if the function is not done, `false` otherwise
     */
    bool read();

    virtual inline float getAccelerationSensitivity() const override { return GRAVITY_MPS2; }
    virtual inline const char* getName() const { return "ISM330DHCX"; }

    void setAccelRange(AccelerometerRangeConfig xl_config);
    void setGyroRange(GyroscopeRangeConfig g_config);
    void setODR(OutputDataRate odr);

private:
    ChipSelect chipSelect;
    float gyroScale;
    float accelScale;
    ImuState prevImuState = ImuState::IMU_NOT_CONNECTED;
    uint8_t tx;
    uint8_t rx;

    uint8_t counter;

    uint8_t rxBuff[15];
    uint8_t txBuff[15];

    static constexpr OutputDataRate DEFAULT_ODR = ODR_833HZ;
    static constexpr GyroscopeRangeConfig DEFAULT_GYRO_RANGE = DPS1000_CONFIG;
    static constexpr AccelerometerRangeConfig DEFAULT_ACCEL_RANGE = G4_CONFIG;

    // Pre-computed register values for non-blocking writes (protothread use)
    static constexpr uint8_t DEFAULT_CTRL1_XL_VALUE = DEFAULT_ODR | DEFAULT_ACCEL_RANGE;
    static constexpr uint8_t DEFAULT_CTRL2_G_VALUE = DEFAULT_ODR | DEFAULT_GYRO_RANGE;

    // Pull CS low to read / write.
    void ismNssLow();

    // Pull CS high to end
    void ismNssHigh();

    // Read from a register
    uint8_t spiReadRegister(uint8_t reg);

    // Write to register
    void spiWriteRegister(uint8_t reg, uint8_t data);

    /**
     * Convert int16_t stored in big endian format in buff to a floating point value.
     *
     * @param[in] buff Buffer containing two bytes representing an int16_t in big endian format.
     * @return A float, the converted int16_t in floating point form.
     */
    inline float bigEndianInt16ToFloat(const uint8_t* buff)
    {
        return static_cast<float>(static_cast<int16_t>((*(buff)) | (*(buff + 1) << 8)));
    }

    float accelValueToMeterPerSec(const uint8_t* buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * accelScale / 1000.0f * getAccelerationSensitivity();
    }

    float gyroValueToRadPerSec(const uint8_t* buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return raw * gyroScale / 1000.0f;
    }

    float tempValueToCelsius(const uint8_t* buff)
    {
        float raw = bigEndianInt16ToFloat(buff);
        return (raw / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;
    }
};
}  // namespace aruwsrc::communication::sensors::imu::ism330

#endif  // ISM330_HPP_
