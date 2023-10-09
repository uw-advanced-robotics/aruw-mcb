/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VIRTUAL_IMU_INTERFACE_HPP_
#define VIRTUAL_IMU_INTERFACE_HPP_

#include "tap/communication/sensors/imu/imu_interface.hpp"
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "aruwsrc/communication/mcb-lite/message_types.hpp"

using namespace tap::communication::sensors::imu::mpu6500;

namespace aruwsrc::virtualMCB
{

// Struct of message coming from MCBLite
struct IMUMessage
{
    float pitch, roll, yaw;
    float Gx, Gy, Gz;
    float Ax, Ay, Az;
    Mpu6500::ImuState imuState;
    float temperature;
} modm_packed;

// Struct of messages being sent to MCBLite
struct CalibrateIMUMessage
{
    uint8_t calibrateIMU;  // Value is ignored, message is only used to trigger calibration
} modm_packed;

class VirtualIMUInterface : public tap::communication::sensors::imu::ImuInterface
{
    friend class SerialMCBLite;

public:
    VirtualIMUInterface() : calibrateIMUMessage()
    {
        calibrateIMUMessage.messageType = CALIBRATE_IMU_MESSAGE;
        calibrateIMUMessage.data[0] = 0;
        calibrateIMUMessage.setCRC16();
    }

    float getPitch() override { return pitch; }
    float getRoll() override { return roll; }
    float getYaw() override { return yaw; }
    float getGx() override { return Gx; }
    float getGy() override { return Gy; }
    float getGz() override { return Gz; }
    float getAx() override { return Ax; }
    float getAy() override { return Ay; }
    float getAz() override { return Az; }
    float getTemp() override { return temperature; }
    Mpu6500::ImuState getImuState() { return imuState; }

    virtual inline const char* getName() const { return "Virtual IMU"; }

    void requestCalibration() { requestCalibrationFlag = true; };

private:
    float pitch, roll, yaw;
    float Gx, Gy, Gz;
    float Ax, Ay, Az;
    Mpu6500::ImuState imuState;
    float temperature;

    bool requestCalibrationFlag = false;

    tap::communication::serial::DJISerial::DJISerial::SerialMessage<sizeof(CalibrateIMUMessage)>
        calibrateIMUMessage;
};

}  // namespace aruwsrc::virtualMCB

#endif
