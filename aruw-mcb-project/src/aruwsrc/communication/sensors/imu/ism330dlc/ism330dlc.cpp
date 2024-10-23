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
    float temperatureRaw = bigEndianInt16ToFloat(rxBuff);

    data.temperature = temperatureRaw * CELSIUS_PER_COUNT;
    
    data.gyroDegPerSec[ImuData::X] = gyroFs * GYRO_DPS_PER_COUNT * data.gyroRaw[ImuData::X];
    data.gyroDegPerSec[ImuData::Y] = gyroFs * GYRO_DPS_PER_COUNT * data.gyroRaw[ImuData::Y];
    data.gyroDegPerSec[ImuData::Z] = gyroFs * GYRO_DPS_PER_COUNT * data.gyroRaw[ImuData::Z];

    data.acc[ImuData::X] = accFs * ACC_PER_COUNT * data.accRaw[ImuData::X];
    data.acc[ImuData::Y] = accFs * ACC_PER_COUNT * data.accRaw[ImuData::Y];
    data.acc[ImuData::Z] = accFs * ACC_PER_COUNT * data.accRaw[ImuData::Z];
}

template<class I2cMaster>
modm::ResumableResult<bool>
Ism330dlc<I2cMaster>::read(ism330dlcData::Register reg, uint8_t* data, int size) {
    RF_BEGIN();

    data[0] = static_cast<uint8_t>(reg);
    this->transaction.configureWriteRead(rxBuff, 1, data, size);

    RF_CALL_END_RETURN(this->runTransaction());
}


// Data to write should start at data[1], as data[0] will be overwritten with register address
template<std::unsigned_integral T>
modm::ResumableResult<bool>
write(ism330dlcData::Register reg, uint8_t* data, int size)
{
    RF_BEGIN();
    data[0] = static_cast<uint8_t>(reg);

    this->transaction.configureWrite(data, size);

    RF_END_RETURN_CALL(this->runTransaction());
}
}
