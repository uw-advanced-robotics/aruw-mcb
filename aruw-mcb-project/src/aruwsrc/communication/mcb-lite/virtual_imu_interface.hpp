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

#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::mcb_lite
{
class VirtualIMUInterface : public tap::communication::sensors::imu::AbstractIMU
{
    friend class MCBLite;

public:
    VirtualIMUInterface() : calibrateIMUMessage()
    {
        calibrateIMUMessage.messageType = MessageTypes::CALIBRATE_IMU_MESSAGE;
        calibrateIMUMessage.setCRC16();
    }

    float getPitch() const override { return pitch; }
    float getRoll() const override { return roll; }
    float getYaw() const override { return yaw; }

    void periodicIMUUpdate() override{};

    virtual inline const char* getName() const { return "Virtual IMU"; }

    void requestCalibration() { sendIMUCalibrationMessage = true; }

    float getAccelerationSensitivity() const override
    {
        // We don't know what IMU the Mcb Lite is using. Also, `periodicIMUUpdate` logic is handled
        // on the MCB Lite, so this value has no actual use on this MCB
        return 0.0f;
    }

private:
    void processIMUMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        IMUMessage* imuMessage = (IMUMessage*)completeMessage.data;
        pitch = imuMessage->pitch;
        roll = imuMessage->roll;
        yaw = imuMessage->yaw;

        this->imuData.gyroRadPerSec = {imuMessage->Gx, imuMessage->Gy, imuMessage->Gz};
        this->imuData.accG = {imuMessage->Ax, imuMessage->Ay, imuMessage->Az};
        this->imuData.temperature = imuMessage->temperature;
        this->imuState = imuMessage->imuState;
    }

    float pitch, roll, yaw;

    DJISerial::DJISerial::SerialMessage<1> calibrateIMUMessage;
    bool sendIMUCalibrationMessage = false;
};

}  // namespace aruwsrc::communication::mcb_lite

#endif
