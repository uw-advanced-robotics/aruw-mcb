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

#ifndef MESSAGE_TYPES_HPP_
#define MESSAGE_TYPES_HPP_

#include "stdint.h"

#include "tap/drivers.hpp"

namespace aruwsrc::virtualMCB
{
enum MessageTypes : uint8_t
{
    CANBUS1_MESSAGE = 0,
    CANBUS2_MESSAGE = 1,
    IMU_MESSAGE = 2,
    CALIBRATE_IMU_MESSAGE = 3,
    DIGITAL_OUTPUT_MESSAGE = 4,
    DIGITAL_PIN_MODE_MESSAGE = 5,
    DIGITAL_PID_READ_MESSAGE = 6,
    ANALOG_PIN_READ_MESSAGE = 7,
    PWM_PIN_DUTY_MESSAGE = 8,
    PWM_TIMER_FREQUENCY_MESSAGE = 9,
    PWM_TIMER_STARTED_MESSAGE = 10,
    LED_CONTROL_MESSAGE = 11
};

// IMU message Lite -> MCB
struct IMUMessage
{
    float pitch, roll, yaw;
    float Gx, Gy, Gz;
    float Ax, Ay, Az;
    tap::communication::sensors::imu::ImuInterface::ImuState imuState;
    float temperature;
} modm_packed;

// IMU messages MCB -> Lite
struct CalibrateIMUMessage
{
    uint8_t calibrateIMU;  // Value is ignored, message is only used to trigger calibration
} modm_packed;

// Digital message Lite -> MCB
struct DigitalInputPinMessage
{
    bool BPinValue;
    bool CPinValue;
    bool DPinValue;
    bool ButtonPinValue;
} modm_packed;

// Digital messages MCB -> Lite
struct DigitalOutputPinMessage
{
    bool EPinValue;
    bool FPinValue;
    bool GPinValue;
    bool HPinValue;
    bool LaserPinValue;
} modm_packed;

struct DigitalPinModeMessage
{
    uint8_t BPinMode;
    uint8_t CPinMode;
    uint8_t DPinMode;
    uint8_t ButtonPinMode;
} modm_packed;

// Analog message Lite -> MCB
struct AnalogInputPinMessage
{
    uint16_t SPinValue;
    uint16_t TPinValue;
    uint16_t UPinValue;
    uint16_t VPinValue;
    uint16_t OLEDPinValue;
} modm_packed;

// PWM messages MCB -> Lite
struct PWMPinDutyMessage
{
    float WPinDuty;
    float XPinDuty;
    float YPinDuty;
    float ZPinDuty;
    float BuzzerPinDuty;
    float IMUHeaterPinDuty;
} modm_packed;

struct PWNTimerFrequencyMessage
{
    uint32_t timer8Frequency;
    uint32_t timer12Frequency;
    uint32_t timer3Frequency;
} modm_packed;

struct PWMTimerStartedMessage
{
    bool timer8Started;
    bool timer12Started;
    bool timer3Started;
} modm_packed;

// LED messages MCB -> Lite
struct LEDControlMessage
{
    bool ALedOn;
    bool BLedOn;
    bool CLedOn;
    bool DLedOn;
    bool ELedOn;
    bool FLedOn;
    bool GLedOn;
    bool HLedOn;
    bool GreenLedOn;
    bool RedLedOn;
} modm_packed;

}  // namespace aruwsrc::virtualMCB

#endif
