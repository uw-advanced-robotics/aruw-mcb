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

using namespace modm;

namespace aruwsrc::communication::sensors::imu
{
template<class I2cMaster>
class Ism330dlc : public tap::communication::sensors::imu::ImuInterface, public ism330dlcData, public modm::I2cDevice<I2cMaster> {

    struct ImuData {
        enum Axis
        {
            X = 0,
            Y = 1,
            Z = 2,
        };

        float gyroRaw[3] = {};
        float accOffsetRaw[3] = {};
        float gyroOffsetRaw[3] = {};
        float accG[3] = {};
        float gyroDegPerSec[3] = {};
        float angularRate[3] = {};
        float temperature;
    } data;

    mockable void periodicIMUUpdate();

    inline void setAccSensitivity(int index) {AccSensitivityScalar = AccCountToAccTable[i];}
    void computeOffsets();

    inline const char *getName() const override;
    inline float getAx() override {return data.accG[ImuData::X];}
    inline float getAy() override {return data.accG[ImuData::Y];}
    inline float getAz() override {return data.accG[ImuData::Z];}
    inline float getGx() override {return data.gyroDegPerSec[ImuData::X];}
    inline float getGy() override {return data.gyroDegPerSec[ImuData::Y];}
    inline float getGz() override {return data.gyroDegPerSec[ImuData::Z];}
    inline float getTemp() override {return 0.f;}
    inline float getYaw() override {return 0.f;}
    inline float getPitch() override {return 0.f;}
    inline float getRoll() override {return 0.f;}


private:
    ImuState imuState = ImuState::IMU_NOT_CONNECTED;

    float AccSensitivityScalar_;

    uint8_t rxBuff[14] = {};
    
	modm::ResumableResult<bool>
    read(ism330dlcData::Register reg, uint8_t* data, int size);

};
}

#endif //ISM330DLC_HPP_
