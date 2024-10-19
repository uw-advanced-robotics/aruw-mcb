#include "ism330dlc.hpp"

namespace aruwsrc::communication::sensors::imu
{

template<class I2cMaster>
void Ism330dlc<I2cMaster>::periodicIMUUpdate()
{
    uint8_t rxBuff[6] = {};

    prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();

    // We read starting from the lowest accelerometer register address since
    // all of our registers are contiguous
    read(OUTX_L_XL, rxBuff, 6 * sizeof(uint8_t));
    data.accRaw[ImuData::X] = bigEndianInt16ToFloat(rxBuff);
    data.accRaw[ImuData::Y] = bigEndianInt16ToFloat(rxBuff + 2);
    data.accRaw[ImuData::Z] = bigEndianInt16ToFloat(rxBuff + 4);

    read(OUTX_L_G, rxBuff, 6 * sizeof(uint8_t));
    data.gyroRaw[ImuData::X] = bigEndianInt16ToFloat(rxBuff);
    data.gyroRaw[ImuData::Y] = bigEndianInt16ToFloat(rxBuff + 2);
    data.gyroRaw[ImuData::Z] = bigEndianInt16ToFloat(rxBuff + 4);

    read(OUT_TEMP_L, rxBuff, 2 * sizeof(uint8_t));
    data.temperature = parseTemp(rxBuff[0], rxBuff[1]);

    if (imuState == ImuState::IMU_CALIBRATING)
    {
        computeOffsets();
    }
    else
    {
        data.gyroDegPerSec[ImuData::X] =
            GYRO_DS_PER_GYRO_COUNT * (data.gyroRaw[ImuData::X] - data.gyroOffsetRaw[ImuData::X]);
        data.gyroDegPerSec[ImuData::Y] =
            GYRO_DS_PER_GYRO_COUNT * (data.gyroRaw[ImuData::Y] - data.gyroOffsetRaw[ImuData::Y]);
        data.gyroDegPerSec[ImuData::Z] =
            GYRO_DS_PER_GYRO_COUNT * (data.gyroRaw[ImuData::Z] - data.gyroOffsetRaw[ImuData::Z]);

        data.accG[ImuData::X] =
            AccSensitivityScalar_ * (data.accRaw[ImuData::X] - data.accOffsetRaw[ImuData::X]);
        data.accG[ImuData::Y] =
            AccSensitivityScalar_ * (data.accRaw[ImuData::Y] - data.accOffsetRaw[ImuData::Y]);
        data.accG[ImuData::Z] =
            AccSensitivityScalar_ * (data.accRaw[ImuData::Z] - data.accOffsetRaw[ImuData::Z]);

        mahonyAlgorithm.updateIMU(
            data.gyroDegPerSec[ImuData::X],
            data.gyroDegPerSec[ImuData::Y],
            data.gyroDegPerSec[ImuData::Z],
            data.accG[ImuData::X],
            data.accG[ImuData::Y],
            data.accG[ImuData::Z]);
    }

    imuHeater.runTemperatureController(data.temperature);
}

// void computeOffsets()
// {
//     // calibrationSample++;

//     data.gyroOffsetRaw[ImuData::X] += data.gyroRaw[ImuData::X];
//     data.gyroOffsetRaw[ImuData::Y] += data.gyroRaw[ImuData::Y];
//     data.gyroOffsetRaw[ImuData::Z] += data.gyroRaw[ImuData::Z];
//     data.accOffsetRaw[ImuData::X] += data.accRaw[ImuData::X];
//     data.accOffsetRaw[ImuData::Y] += data.accRaw[ImuData::Y];
//     data.accOffsetRaw[ImuData::Z] +=
//         data.accRaw[ImuData::Z] - (tap::algorithms::ACCELERATION_GRAVITY / ACC_G_PER_ACC_COUNT);

//     if (calibrationSample >= BMI088_OFFSET_SAMPLES)
//     {
//         calibrationSample = 0;
//         data.gyroOffsetRaw[ImuData::X] /= BMI088_OFFSET_SAMPLES;
//         data.gyroOffsetRaw[ImuData::Y] /= BMI088_OFFSET_SAMPLES;
//         data.gyroOffsetRaw[ImuData::Z] /= BMI088_OFFSET_SAMPLES;
//         data.accOffsetRaw[ImuData::X] /= BMI088_OFFSET_SAMPLES;
//         data.accOffsetRaw[ImuData::Y] /= BMI088_OFFSET_SAMPLES;
//         data.accOffsetRaw[ImuData::Z] /= BMI088_OFFSET_SAMPLES;
//         imuState = ImuState::IMU_CALIBRATED;
//         mahonyAlgorithm.reset();
//     }
// }

template<class I2cMaster>
modm::ResumableResult<bool>
Ism330dlc<I2cMaster>::read(ism330dlcData::Register reg, uint8_t* data, int size) {
    RF_BEGIN();

    // idk does this work??
    rxBuff[0] = static_cast<uint8_t>(reg);
    this->transaction.configureWriteRead(rxBuff, 1, data, size);

    RF_CALL(this->runTransaction());

    // T result;

    // if constexpr (std::is_same_v<T, uint8_t>)
    // {
    //     result = buffer[1];
    // } else
    // {
    //     result = buffer[1] << 8 | buffer[2];
    // }

    // RF_END_RETURN(result);
    RF_END_RETURN_CALL(this->runTransaction());
}
}
