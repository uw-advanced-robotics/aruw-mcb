/*
 * Copyright (c) 2023-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef VIRTUAL_IMU_HPP_
#define VIRTUAL_IMU_HPP_

#include "tap/communication/sensors/imu/abstract_imu.hpp"
#include "tap/communication/serial/dji_serial.hpp"

#include "aruwsrc/communication/sensors/imu/notch_filter.hpp"

#include "message_types.hpp"

using namespace tap::communication::serial;

namespace aruwsrc::communication::mcb_lite
{
class VirtualIMU : public tap::communication::sensors::imu::AbstractIMU
{
    friend class MCBLite;

public:
    VirtualIMU() : calibrateIMUMessage()
    {
        calibrateIMUMessage.messageType = MessageTypes::CALIBRATE_IMU_MESSAGE;
        calibrateIMUMessage.setCRC16();
        mountingTransformMessage.messageType = MessageTypes::IMU_MOUNTING_TRANSFORM_MESSAGE;
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

    void sendMountingTransform(const tap::algorithms::transforms::Transform& transform)
    {
        IMUMountingTransformMessage transformMessage;
        transformMessage.x = transform.getTranslation().x();
        transformMessage.y = transform.getTranslation().y();
        transformMessage.z = transform.getTranslation().z();
        transformMessage.roll = transform.getRoll();
        transformMessage.pitch = transform.getPitch();
        transformMessage.yaw = transform.getYaw();
        memcpy(
            mountingTransformMessage.data,
            &transformMessage,
            sizeof(IMUMountingTransformMessage));
        mountingTransformMessage.setCRC16();
        hasNewMountingTransform = true;
    }

    void processMountingTransform() { hasNewMountingTransform = false; }

    /**
     * Enables and configures a notch filter applied to gyro and accel data as it's
     * received from the MCB Lite, to attenuate a known narrowband vibration frequency
     * (e.g. from a pump or other rotating component).
     *
     * @param[in] notchFrequencyHz Center frequency to attenuate, in Hz.
     * @param[in] sampleFrequencyHz Rate at which IMU messages arrive, in Hz.
     * @param[in] qFactor Quality factor controlling notch width (higher = narrower).
     */
    void setNotchFilter(float notchFrequencyHz, float sampleFrequencyHz, float qFactor = 0.707f)
    {
        for (auto& f : gyroNotchFilter) f.configure(notchFrequencyHz, sampleFrequencyHz, qFactor);
        for (auto& f : accelNotchFilter) f.configure(notchFrequencyHz, sampleFrequencyHz, qFactor);
        notchFilterEnabled = true;
    }

    /// Disables the notch filter, if enabled. Raw (unfiltered) samples are used again.
    void disableNotchFilter() { notchFilterEnabled = false; }

private:
    void processIMUMessage(const DJISerial::ReceivedSerialMessage& completeMessage)
    {
        IMUMessage* imuMessage = (IMUMessage*)completeMessage.data;
        pitch = imuMessage->pitch;
        roll = imuMessage->roll;
        yaw = imuMessage->yaw;

        this->imuData.gyroRadPerSec = {imuMessage->Gx, imuMessage->Gy, imuMessage->Gz};
        this->imuData.accG = {imuMessage->Ax, imuMessage->Ay, imuMessage->Az};

        if (notchFilterEnabled)
        {
            this->imuData.gyroRadPerSec = {
                gyroNotchFilter[0].filter(this->imuData.gyroRadPerSec.x()),
                gyroNotchFilter[1].filter(this->imuData.gyroRadPerSec.y()),
                gyroNotchFilter[2].filter(this->imuData.gyroRadPerSec.z())};
            this->imuData.accG = {
                accelNotchFilter[0].filter(this->imuData.accG.x()),
                accelNotchFilter[1].filter(this->imuData.accG.y()),
                accelNotchFilter[2].filter(this->imuData.accG.z())};
        }

        this->imuData.temperature = imuMessage->temperature;
        this->imuState = imuMessage->imuState;
    }

    float pitch, roll, yaw;

    DJISerial::DJISerial::SerialMessage<1> calibrateIMUMessage;
    bool sendIMUCalibrationMessage = false;

    DJISerial::SerialMessage<sizeof(IMUMountingTransformMessage)> mountingTransformMessage;
    bool hasNewMountingTransform = false;

    bool notchFilterEnabled = false;
    aruwsrc::communication::sensors::imu::ism330::NotchFilter gyroNotchFilter[3];
    aruwsrc::communication::sensors::imu::ism330::NotchFilter accelNotchFilter[3];
};

}  // namespace aruwsrc::communication::mcb_lite

#endif