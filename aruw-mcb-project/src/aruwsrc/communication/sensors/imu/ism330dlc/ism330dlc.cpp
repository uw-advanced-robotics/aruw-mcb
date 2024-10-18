#include "ism330dlc.hpp"

namespace aruwsrc::communication::sensors::imu
{

void Ism330dlc::periodicIMUUpdate()
{
    
    read
}

float Ism330dlc::getAx()
{
    return data.accG[ImuData::X];
}

float Ism330dlc::getAy()
{
    return data.accG[ImuData::Y];
}

float Ism330dlc::getAz()
{
    return data.accG[ImuData::Z];
}

float Ism330dlc::getGx() {
    return data.gyroDegPerSec[ImuData::X];
}

float Ism330dlc::getGy() {
    return data.gyroDegPerSec[ImuData::Y];
}

float Ism330dlc::getGz() {
    return data.gyroDegPerSec[ImuData::Z];
}

float Ism330dlc::getRoll() {
    return data.angularRate[ImuData::X];
}

float Ism330dlc::getPitch()
{
    return data.angularRate[ImuData::Y];
}

float Ism330dlc::getYaw()
{
    return data.angularRate[ImuData::Z];
}


float Ism330dlc::getTemp()
{
    return data.temperature;
}


modm::ResumableResult<bool> Ism330dlc::read(ism330dlcData::Register reg, int size) {
    RF_BEGIN();

    rxBuff[0] = reg;
    this->transaction.configureWriteRead(buffer, 1, buffer + 1, size);

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
