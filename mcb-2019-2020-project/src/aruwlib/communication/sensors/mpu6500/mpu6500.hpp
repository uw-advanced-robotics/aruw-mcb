#ifndef MPU6500_HPP_
#define MPU6500_HPP_

#include <cstdint>

#include "aruwlib/algorithms/MahonyAHRS.h"
#include "aruwlib/rm-dev-board-a/board.hpp"

#include "mpu6500_reg.hpp"

namespace aruwlib
{
namespace sensors
{
using namespace modm::literals;
/**
 * A class specifically designed for interfacing with the RoboMaster type A board Mpu6500.
 *
 * To use this class, call Remote::init() to properly initialize and calibrate
 * the MPU6500. Next, call Remote::read() to read acceleration, gyro, and temp
 * values from the imu. Use the getter methods to access imu information.
 *
 * @note if you are shaking the imu while it is initializing, the offsets will likely
 *      be calibrated poorly and unexpectedly bad results may occur.
 */
template <typename Drivers> class Mpu6500
{
public:
    Mpu6500() = default;
    Mpu6500(const Mpu6500 &) = delete;
    Mpu6500 &operator=(const Mpu6500 &) = delete;

    /**
     * Initialize the imu and the SPI line. Uses SPI1, which is internal to the
     * type A board.
     *
     * @note this function blocks for approximately 1 second.
     */
    void init()
    {
#ifndef ENV_SIMULATOR
        Board::ImuNss::GpioOutput();

        // connect GPIO pins to the alternate SPI function
        Board::ImuSpiMaster::
            connect<Board::ImuMiso::Miso, Board::ImuMosi::Mosi, Board::ImuSck::Sck>();

        // initialize SPI with clock speed
        Board::ImuSpiMaster::initialize<Board::SystemClock, 703125_Hz>();

        // set power mode
        spiWriteRegister(MPU6500_PWR_MGMT_1, 0x80);

        modm::delayMilliseconds(100);

        // reset gyro, accel, and temp
        spiWriteRegister(MPU6500_SIGNAL_PATH_RESET, 0x07);
        modm::delayMilliseconds(100);

        // verify mpu register ID
        if (MPU6500_ID != spiReadRegister(MPU6500_WHO_AM_I))
        {
            RAISE_ERROR(
                "failed to initialize the imu properly",
                aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_NOT_RECEIVING_PROPERLY);
            return;
        }

        imuInitialized = true;

        // 0: 250hz; 1: 184hz; 2: 92hz; 3: 41hz; 4: 20hz; 5: 10hz; 6: 5hz; 7: 3600hz
        uint8_t Mpu6500InitData[7][2] = {
            {MPU6500_PWR_MGMT_1, 0x03},      // Auto selects Clock Source
            {MPU6500_PWR_MGMT_2, 0x00},      // all enable
            {MPU6500_CONFIG, 0x02},          // gyro bandwidth 0x00:250Hz 0x04:20Hz
            {MPU6500_GYRO_CONFIG, 0x18},     // gyro range 0x10:+-1000dps 0x18:+-2000dps
            {MPU6500_ACCEL_CONFIG, 0x10},    // acc range 0x10:+-8G
            {MPU6500_ACCEL_CONFIG_2, 0x00},  // acc bandwidth 0x00:250Hz 0x04:20Hz
            {MPU6500_USER_CTRL, 0x20},       // Enable the I2C Master I/F module
                                             // pins ES_DA and ES_SCL are isolated from
                                             // pins SDA/SDI and SCL/SCLK.
        };

        // write init setting to registers
        for (int i = 0; i < 7; i++)
        {
            spiWriteRegister(Mpu6500InitData[i][0], Mpu6500InitData[i][1]);
            modm::delayMilliseconds(1);
        }

        calculateAccOffset();
        calculateGyroOffset();
#endif
    }

    /**
     * Read data from the imu. Call at 500 hz for best performance.
     */
    void read()
    {
#ifndef ENV_SIMULATOR
        if (imuInitialized)
        {
            spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE);
            raw.accel.x = (rxBuff[0] << 8 | rxBuff[1]) - raw.accelOffset.x;
            raw.accel.y = (rxBuff[2] << 8 | rxBuff[3]) - raw.accelOffset.y;
            raw.accel.z = (rxBuff[4] << 8 | rxBuff[5]) - raw.accelOffset.z;

            raw.temp = rxBuff[6] << 8 | rxBuff[7];

            raw.gyro.x = ((rxBuff[8] << 8 | rxBuff[9]) - raw.gyroOffset.x);
            raw.gyro.y = ((rxBuff[10] << 8 | rxBuff[11]) - raw.gyroOffset.y);
            raw.gyro.z = ((rxBuff[12] << 8 | rxBuff[13]) - raw.gyroOffset.z);

            mahonyAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
            tiltAngleCalculated = false;
        }
        else
        {
            RAISE_ERROR(
                "failed to initialize the imu properly",
                aruwlib::errors::Location::MPU6500,
                aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        }
#endif
    }

    /**
     * To be safe, whenever you call the functions below, call this function to insure
     * the data you are about to receive is not garbage.
     */
    bool initialized() const { return imuInitialized; }

    /**
     * Returns the acceleration reading in the x direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    float getAx() const
    {
        return validateReading(
            static_cast<float>(raw.accel.x) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
    }

    /**
     * Returns the acceleration reading in the y direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    float getAy() const
    {
        return validateReading(
            static_cast<float>(raw.accel.y) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
    }

    /**
     * Returns the acceleration reading in the z direction, in
     * \f$\frac{\mbox{m}}{\mbox{second}^2}\f$.
     */
    float getAz() const
    {
        return validateReading(
            static_cast<float>(raw.accel.z) * ACCELERATION_GRAVITY / ACCELERATION_SENSITIVITY);
    }

    /**
     * Returns the gyroscope reading in the x direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    float getGx() const
    {
        return validateReading(static_cast<float>(raw.gyro.x) / LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * Returns the gyroscope reading in the y direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    float getGy() const
    {
        return validateReading(static_cast<float>(raw.gyro.y) / LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * Returns the gyroscope reading in the z direction, in
     * \f$\frac{\mbox{degrees}}{\mbox{second}}\f$.
     */
    float getGz() const
    {
        return validateReading(static_cast<float>(raw.gyro.z) / LSB_D_PER_S_TO_D_PER_S);
    }

    /**
     * Returns the temperature of the imu in degrees C.
     */
    float getTemp() const
    {
        return validateReading(21.0f + static_cast<float>(raw.temp) / 333.87f);
    }

    /**
     * Returns yaw angle. in degrees.
     */
    float getYaw() { return validateReading(mahonyAlgorithm.getYaw()); }

    /**
     * Returns pitch angle in degrees.
     */
    float getPitch() { return validateReading(mahonyAlgorithm.getPitch()); }

    /**
     * Returns roll angle in degrees.
     */
    float getRoll() { return validateReading(mahonyAlgorithm.getRoll()); }

    /**
     * Returns the angle difference between the normal vector of the plane that the
     * type A board lies on and of the angle directly upward.
     */
    float getTiltAngle()
    {
        if (!tiltAngleCalculated)
        {
            tiltAngle = aruwlib::algorithms::radiansToDegrees(acosf(
                cosf(mahonyAlgorithm.getPitchRadians()) * cosf(mahonyAlgorithm.getRollRadians())));
            tiltAngleCalculated = true;
        }
        return validateReading(tiltAngle);
    }

private:
    static constexpr float ACCELERATION_GRAVITY = 9.80665f;

    ///< Use for converting from gyro values we receive to more conventional degrees / second.
    static constexpr float LSB_D_PER_S_TO_D_PER_S = 16.384f;

    ///< Use to convert the raw acceleration into more conventional degrees / second^2
    static constexpr float ACCELERATION_SENSITIVITY = 4096.0f;

    ///< The number of samples we take in order to determine the mpu offsets.
    static constexpr float MPU6500_OFFSET_SAMPLES = 300;

    ///< The number of bytes read to read acceleration, gyro, and temp.
    static const uint8_t ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE = 14;

    /**
     * Storage for the raw data we receive from the mpu6500, as well as offsets
     * that are used each time we receive data.
     */
    struct RawData
    {
        ///< Raw acceleration data.
        struct Accel
        {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        ///< Raw gyroscope data.
        struct Gyro
        {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        ///< Acceleration offset calculated in init.
        struct AccelOffset
        {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        ///< Gyroscope offset calculated in init.
        struct GyroOffset
        {
            int16_t x = 0;
            int16_t y = 0;
            int16_t z = 0;
        };

        Accel accel;
        Gyro gyro;

        ///< Raw temperature.
        uint16_t temp = 0;

        AccelOffset accelOffset;
        GyroOffset gyroOffset;
    };

    bool imuInitialized = false;

    RawData raw;

    Mahony mahonyAlgorithm;

    float tiltAngle = 0.0f;
    bool tiltAngleCalculated = false;

    uint8_t txBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    uint8_t rxBuff[ACC_GYRO_TEMPERATURE_BUFF_RX_SIZE] = {0};

    ///< Compute the gyro offset values. @note this function blocks.
    void calculateGyroOffset()
    {
#ifndef ENV_SIMULATOR
        for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++)
        {
            spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
            raw.gyroOffset.x += (rxBuff[8] << 8) | rxBuff[9];
            raw.gyroOffset.y += (rxBuff[10] << 8) | rxBuff[11];
            raw.gyroOffset.z += (rxBuff[12] << 8) | rxBuff[13];
            modm::delayMilliseconds(2);
        }

        raw.gyroOffset.x /= MPU6500_OFFSET_SAMPLES;
        raw.gyroOffset.y /= MPU6500_OFFSET_SAMPLES;
        raw.gyroOffset.z /= MPU6500_OFFSET_SAMPLES;
#endif
    }

    ///< Calibrate accelerometer offset values. @note this function blocks.
    void calculateAccOffset()
    {
#ifndef ENV_SIMULATOR
        for (int i = 0; i < MPU6500_OFFSET_SAMPLES; i++)
        {
            spiReadRegisters(MPU6500_ACCEL_XOUT_H, rxBuff, 14);
            raw.accelOffset.x += (rxBuff[0] << 8) | rxBuff[1];
            raw.accelOffset.y += (rxBuff[2] << 8) | rxBuff[3];
            raw.accelOffset.z += ((rxBuff[4] << 8) | rxBuff[5]) - 4096;
            modm::delayMilliseconds(2);
        }

        raw.accelOffset.x /= MPU6500_OFFSET_SAMPLES;
        raw.accelOffset.y /= MPU6500_OFFSET_SAMPLES;
        raw.accelOffset.z /= MPU6500_OFFSET_SAMPLES;
#endif
    }

    // Functions for interacting with hardware directly.

    ///< Pull the NSS pin low to initiate contact with the imu.
    void mpuNssLow()
    {
#ifndef ENV_SIMULATOR
        Board::ImuNss::setOutput(modm::GpioOutput::Low);
#endif
    }

    ///< Pull the NSS pin high to end contact with the imu.
    void mpuNssHigh()
    {
#ifndef ENV_SIMULATOR
        Board::ImuNss::setOutput(modm::GpioOutput::High);
#endif
    }

    /**
     * If the imu is not initializes, logs an error and returns 0,
     * otherwise returns the value passed in.
     */
    inline float validateReading(float reading) const
    {
        if (imuInitialized)
        {
            return reading;
        }
        RAISE_ERROR(
            "failed to initialize the imu properly",
            aruwlib::errors::Location::MPU6500,
            aruwlib::errors::ErrorType::IMU_DATA_NOT_INITIALIZED);
        return 0.0f;
    }

    /**
     * Write to a given register.
     */
    uint8_t spiWriteRegister(uint8_t reg, uint8_t data)
    {
#ifndef ENV_SIMULATOR
        mpuNssLow();
        uint8_t tx = reg & 0x7F;
        uint8_t rx = 0;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        tx = data;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        mpuNssHigh();
#endif
        return 0;
    }

    /**
     * Read from a given register.
     */
    uint8_t spiReadRegister(uint8_t reg)
    {
#ifndef ENV_SIMULATOR
        mpuNssLow();
        uint8_t tx = reg | 0x80;
        uint8_t rx = 0;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        mpuNssHigh();
        return rx;
#else
        return 0;
#endif
    }

    /**
     * Read from several registers.
     * regAddr is the first address read, and it reads len number of addresses
     * from that point.
     */
    uint8_t spiReadRegisters(uint8_t regAddr, uint8_t *pData, uint8_t len)
    {
#ifndef ENV_SIMULATOR
        mpuNssLow();
        uint8_t tx = regAddr | 0x80;
        uint8_t rx = 0;
        txBuff[0] = tx;
        Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
        Board::ImuSpiMaster::transferBlocking(txBuff, pData, len);
        mpuNssHigh();
#endif
        return 0;
    }
};

}  // namespace sensors

}  // namespace aruwlib

#endif  // MPU6500_HPP_
