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
#include "tap/communication/sensors/imu/mpu6500/mpu6500.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "message_types.hpp"

using namespace tap::communication::sensors::imu::mpu6500;
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

    float getGx() const override { return imuData.gyroRadPerSec.x(); }
    float getGy() const override { return imuData.gyroRadPerSec.y(); }
    float getGz() const override { return imuData.gyroRadPerSec.z(); }
    float getAx() const override { return imuData.accG.x(); }
    float getAy() const override { return imuData.accG.y(); }
    float getAz() const override { return imuData.accG.z(); }
    float getTemp() const { return imuData.temperature; }
    void periodicIMUUpdate() override {};

    AbstractIMU::ImuState getImuState() const { return imuState; }

    virtual inline const char* getName() const { return "Virtual MPU6500"; }

    void requestCalibration() { sendIMUCalibrationMessage = true; }

    float getAccelerationSensitivity() const override
    {
        return 2 * 1.5f * tap::algorithms::ACCELERATION_GRAVITY / 32768.0f;
    }  // copied this from turretmcb bc both are bmi i think?

private:
    void processIMUMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        IMUMessage* imuMessage = (IMUMessage*)completeMessage.data;
        pitch = imuMessage->pitch;
        roll = imuMessage->roll;
#ifdef TARGET_SENTRY_ECLIPSE
        // IMUs initalize yaw at 180 degrees for some reason, must be resolved as tech debt
        yaw = fmodf(imuMessage->yaw + 180, 360);
#else
        yaw = imuMessage->yaw;
#endif
        imuData.gyroRadPerSec = {imuMessage->Gx, imuMessage->Gy, imuMessage->Gz};
        imuData.accG = {imuMessage->Ax, imuMessage->Ay, imuMessage->Az};
        imuData.temperature = imuMessage->temperature;
        imuState = imuMessage->imuState;
    }

    float pitch, roll, yaw;

    AbstractIMU::ImuState imuState;
    float temperature;

    DJISerial::DJISerial::SerialMessage<1> calibrateIMUMessage;
    bool sendIMUCalibrationMessage = false;
};

}  // namespace aruwsrc::communication::mcb_lite

#endif
