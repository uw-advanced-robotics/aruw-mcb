/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ISM330DLC_HPP_
#define ISM330DLC_HPP_

#include <modm/architecture/interface/i2c_device.hpp>
#include <modm/architecture/interface/register.hpp>
#include "modm/processing/resumable.hpp"
#include "aruwsrc/communication/sensors/imu/ism330dlc/ism330dlc_data.hpp"

#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/util_macros.hpp"
#include "tap/algorithms/math_user_utils.hpp"

#define LITTLE_ENDIAN_INT16_TO_FLOAT(buff) \
    (static_cast<float>(static_cast<int16_t>((*(buff) << 8) | *(buff + 1))))

using namespace modm;

namespace aruwsrc::communication::sensors::imu
{
template<class I2cMaster>
class Ism330dlc : public tap::communication::sensors::imu::ImuInterface, public ism330dlcData, public modm::I2cDevice<I2cMaster> {
public:
    Ism330dlc(uint8_t address) : modm::I2cDevice<I2cMaster>(address) {}

    struct ImuData {
        enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float gyroRaw[3] = {};
        float accRaw[3] = {};
        float acc[3] = {};
        float gyro[3] = {};
        float temperature;
    } data;

    mockable void periodicIMUUpdate();

    inline void setAccSensitivity(int index) {accFsSetting = static_cast<accFs>(1 << index);}
    void computeOffsets();

    inline const char *getName() const override { return "ism330dlc"; };
    inline float getAx() override {return data.acc[ImuData::X];}
    inline float getAy() override {return data.acc[ImuData::Y];}
    inline float getAz() override {return data.acc[ImuData::Z];}
    inline float getGx() override {return data.gyro[ImuData::X];}
    inline float getGy() override {return data.gyro[ImuData::Y];}
    inline float getGz() override {return data.gyro[ImuData::Z];}
    inline float getTemp() override {return data.temperature;}
    inline float getYaw() override {return 0.f;}
    inline float getPitch() override {return 0.f;}
    inline float getRoll() override {return 0.f;}


private:
    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    accFs accFsSetting = accFs::FS_2G;
    gyroFs gyroFsSetting = gyroFs::FS_125DPS;

    uint32_t prevIMUDataReceivedTime;

    uint8_t rxBuff[14] = {};
    
	modm::ResumableResult<bool>
    read(ism330dlcData::Register reg, uint8_t* data, int size);

    modm::ResumableResult<bool>
    write(ism330dlcData::Register reg, uint8_t* data, int size);

    
    uint8_t allData[30] = {};
    bool worked = false;

};

template<class I2cMaster>
void Ism330dlc<I2cMaster>::periodicIMUUpdate()
{
    // uint8_t rxBuff[6] = {};

    prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();

    worked = RF_CALL_BLOCKING(read(OUT_TEMP_L, allData, 30));

    // We read starting from the lowest accelerometer register address since
    // all of our registers are contiguous
    RF_CALL_BLOCKING(read(OUTX_L_XL, rxBuff, 6 * sizeof(uint8_t)));
    data.accRaw[ImuData::X] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    data.accRaw[ImuData::Y] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    data.accRaw[ImuData::Z] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    // RF_CALL_BLOCKING(read(OUTX_L_XL, rxBuff, 2 * sizeof(uint8_t)));
    // data.accRaw[ImuData::X] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    // RF_CALL_BLOCKING(read(OUTY_L_XL, rxBuff, 2 * sizeof(uint8_t)));
    // data.accRaw[ImuData::Y] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    // RF_CALL_BLOCKING(read(OUTZ_L_XL, rxBuff, 2 * sizeof(uint8_t)));
    // data.accRaw[ImuData::Z] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);

    // RF_CALL_BLOCKING(read(OUTX_L_G, rxBuff, 6 * sizeof(uint8_t)));
    // data.gyroRaw[ImuData::X] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    // data.gyroRaw[ImuData::Y] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 2);
    // data.gyroRaw[ImuData::Z] = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff + 4);

    // RF_CALL_BLOCKING(read(OUT_TEMP_L, rxBuff, 2 * sizeof(uint8_t)));
    // float temperatureRaw = LITTLE_ENDIAN_INT16_TO_FLOAT(rxBuff);
    // data.temperature = temperatureRaw * CELSIUS_PER_COUNT;
    
    data.gyro[ImuData::X] = static_cast<float>(gyroFsSetting) * GYRO_DPS_PER_COUNT * data.gyroRaw[ImuData::X];
    data.gyro[ImuData::Y] = static_cast<float>(gyroFsSetting) * GYRO_DPS_PER_COUNT * data.gyroRaw[ImuData::Y];
    data.gyro[ImuData::Z] = static_cast<float>(gyroFsSetting) * GYRO_DPS_PER_COUNT * data.gyroRaw[ImuData::Z];

    data.acc[ImuData::X] = static_cast<float>(accFsSetting) * ACC_MPS_PER_COUNT * data.accRaw[ImuData::X];
    data.acc[ImuData::Y] = static_cast<float>(accFsSetting) * ACC_MPS_PER_COUNT * data.accRaw[ImuData::Y];
    data.acc[ImuData::Z] = static_cast<float>(accFsSetting) * ACC_MPS_PER_COUNT * data.accRaw[ImuData::Z];
}

template<class I2cMaster>
modm::ResumableResult<bool> Ism330dlc<I2cMaster>::read(ism330dlcData::Register reg, uint8_t* data, int size) {
    RF_BEGIN();

    data[0] = static_cast<uint8_t>(reg);
    RF_WAIT_WHILE(!this->transaction.configureWriteRead(data, 1, data, 6 * sizeof(uint8_t)));

    RF_END_RETURN_CALL(this->runTransaction());
}


// Data to write should start at data[1], as data[0] will be overwritten with register address
// template<std::unsigned_integral T>
template<class I2cMaster>
modm::ResumableResult<bool> Ism330dlc<I2cMaster>::write(ism330dlcData::Register reg, uint8_t* data, int size)
{
    RF_BEGIN();
    data[0] = static_cast<uint8_t>(reg);

    this->transaction.configureWrite(data, size);

    RF_END_RETURN_CALL(this->runTransaction());
}

} // namespace aruwsrc::communication::sensors::imu

#endif //ISM330DLC_HPP_
