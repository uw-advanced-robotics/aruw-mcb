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

#include "message_types.hpp"

using namespace tap::communication::sensors::imu::mpu6500;
using namespace tap::communication::serial;

namespace aruwsrc::virtualMCB
{
class VirtualIMUInterface : public tap::communication::sensors::imu::ImuInterface
{
    friend class MCBLite;

public:
    VirtualIMUInterface() : calibrateIMUMessage()
    {
        calibrateIMUMessage.messageType = MessageTypes::CALIBRATE_IMU_MESSAGE;
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
    virtual inline const char* getName() const { return "Virtual MPU6500"; }
    void requestCalibration() { sendIMUCalibrationMessage = true; }

private:
    void processIMUMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        IMUMessage* imuMessage = (IMUMessage*)completeMessage.data;
        pitch = imuMessage->pitch;
        roll = imuMessage->roll;
        yaw = fmodf(imuMessage->yaw + 180, 360);
        Gx = imuMessage->Gx;
        Gy = imuMessage->Gy;
        Gz = imuMessage->Gz;
        Ax = imuMessage->Ax;
        Ay = imuMessage->Ay;
        Az = imuMessage->Az;
        imuState = imuMessage->imuState;
        temperature = imuMessage->temperature;
    }

    float pitch, roll, yaw;
    float Gx, Gy, Gz;
    float Ax, Ay, Az;
    Mpu6500::ImuState imuState;
    float temperature;

    DJISerial::DJISerial::SerialMessage<1> calibrateIMUMessage;
    bool sendIMUCalibrationMessage = false;
};

}  // namespace aruwsrc::virtualMCB

#endif
