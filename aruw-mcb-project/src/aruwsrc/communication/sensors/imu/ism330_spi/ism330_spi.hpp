#ifndef ISM330_SPI_HPP_
#define ISM330_SPI_HPP_

#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "modm/processing/protothread.hpp"
#include "modm/processing/resumable.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/sensors/imu/ism330/ism330_data.hpp"

namespace aruwsrc::communication::sensors::imu::ism330
{
using namespace tap::communication::sensors::imu;

class Ism330Spi : public AbstractIMU, public modm::pt::Protothread
{
public:
    Ism330Spi();
    DISALLOW_COPY_AND_ASSIGN(Ism330Spi);

    virtual void initialize(float sampleFrequency, float mahonyKp, float mahonyKi);

    /**
     * Read data from the imu. This is a protothread that reads the SPI bus using
     * nonblocking I/O.
     *
     * @return `true` if the function is not done, `false` otherwise
     */
    bool read();

    void periodicIMUUpdate() override;

    virtual inline float getAccelerationSensitivity() const override { return GRAVITY_MPS2; }
    virtual inline const char *getName() const { return "ISM330DHCX"; }
    virtual void periodicIMUUpdate();

    void setAccelRange(AccelerometerRangeConfig xl_config);
    void setGyroRange(GyroscopeRangeConfig g_config);
    void setODR(OutputDataRate odr);

private:
    float gyroScale;
    float accelScale;
    uint8_t tx;
    uint8_t rx;

    uint8_t rxBuff[15];
    uint8_t txBuff[2];

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

#endif  // ISM330_SPI_HPP_