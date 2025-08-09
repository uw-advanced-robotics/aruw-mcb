/*
 * Copyright (c) 2024-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ISM330_IMPL_HPP_
#define ISM330_IMPL_HPP_

namespace aruwsrc::communication::sensors::imu::ism330
{
template <class I2cMaster>
ISM330<I2cMaster>::ISM330(tap::Drivers *drivers)
    : modm::I2cDevice<I2cMaster>(DEVICE_ADDRESS),
      AbstractIMU(),
      modm::pt::Protothread(),
      drivers(drivers),
      errorTimeout(BeginningErrorTimeoutTime),
      errorTimeoutPower(BeginningErrorTimeoutTime)
{
}

template <class I2cMaster>
void ISM330<I2cMaster>::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    AbstractIMU::initialize(sampleFrequency, mahonyKp, mahonyKi);
}

template <class I2cMaster>
modm::ResumableResult<void> ISM330<I2cMaster>::reinitialize()
{
    RF_BEGIN();
    RF_CALL(readRegister(WHO_AM_I, 3, rxBuff));

    RF_CALL(setODR(ODR_1660HZ));
    RF_CALL(setGyroRange(DPS1000_CONFIG));
    RF_CALL(setAccelRange(G4_CONFIG));
    RF_END();
}

template <class I2cMaster>
bool ISM330<I2cMaster>::read()
{    
    PT_BEGIN();
    PT_CALL(reinitialize());

    while (true)
    {
        if (erroredOut)
        {
            hasErrored++;
            erroredOut = false;
            PT_CALL(attemptReconnect());
        }

        PT_CALL(readRegister(WHO_AM_I, 1, rxBuff));
        imuData.deviceId = rxBuff[0];

        PT_CALL(readRegister(OUT_TEMP_L, READ_LENGTH, rxBuff));

        // we have started to read actual data so set to proper timeout
        if (!erroredOut && imuData.deviceId == 0x6B)
        {
            errorTimeout.restart(timeout * 4);
            errorTimeoutPower.restart(errorTimeoutPowerTime);
        }

        processData();
    }
    PT_END();
}

template <class I2cMaster>
void ISM330<I2cMaster>::processData()
{
    imuData.temperature = tempValueToCelsius(rxBuff);

    imuData.gyroRaw = {
        gyroValueToRadPerSec(rxBuff + 2), 
        gyroValueToRadPerSec(rxBuff + 4), 
        gyroValueToRadPerSec(rxBuff + 6)};

    imuData.accRaw = {
        accelValueToMeterPerSec(rxBuff + 8),
        accelValueToMeterPerSec(rxBuff + 10),
        accelValueToMeterPerSec(rxBuff + 12)};

    applyMountingTransformToRaw(imuData);

    imuData.gyroRadPerSec = imuData.gyroRaw - imuData.gyroOffsetRaw;
    imuData.accG = imuData.accRaw - imuData.accOffsetRaw;
    prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();
}

template <class I2cMaster>
modm::ResumableResult<void> ISM330<I2cMaster>::setAccelRange(AccelerometerRangeConfig xl_config)
{
    RF_BEGIN();
    RF_CALL(readRegister(CTRL1_XL, 1, &current_reg_XL));

    RF_CALL(writeRegister(CTRL1_XL, (current_reg_XL & G_CONFIG_BITMASK) | xl_config));
    switch (xl_config)
    {
        case G2_CONFIG:
            accelScale = 0.061;
            break;
        case G4_CONFIG:
            accelScale = 0.122;
            break;
        case G8_CONFIG:
            accelScale = 0.244;
            break;
        case G16_CONFIG:
            accelScale = 0.488;
            break;
        default:
            break;
    }
    RF_END();
}

template <class I2cMaster>
modm::ResumableResult<void> ISM330<I2cMaster>::setGyroRange(GyroscopeRangeConfig g_config)
{
    RF_BEGIN();
    RF_CALL(readRegister(CTRL2_G, 1, &current_reg_G));

    RF_CALL(writeRegister(CTRL2_G, (current_reg_G & DPS_CONFIG_BITMASK) | g_config));
    switch (g_config)
    {
        case DPS250_CONFIG:
            gyroScale = modm::toRadian(8.75);
            break;
        case DPS500_CONFIG:
            gyroScale = modm::toRadian(17.50);
            break;
        case DPS1000_CONFIG:
            gyroScale = modm::toRadian(35);
            break;
        case DPS2000_CONFIG:
            gyroScale = modm::toRadian(70);
            break;
        default:
            break;
    }
    RF_END();
}

template <class I2cMaster>
modm::ResumableResult<void> ISM330<I2cMaster>::setODR(OutputDataRate odr)
{
    RF_BEGIN();
    RF_CALL(readRegister(CTRL1_XL, 1, &current_reg_XL));
    RF_CALL(readRegister(CTRL2_G, 1, &current_reg_G));

    RF_CALL(writeRegister(CTRL1_XL, (current_reg_XL & ODR_BITMASK) | odr));
    RF_CALL(writeRegister(CTRL2_G, (current_reg_G & ODR_BITMASK) | odr));

    switch (odr)
    {
        case ODR_416HZ:
            timeout = 1000000 / 416;
            break;
        case ODR_833HZ:
            timeout = 1000000 / 833;
            break;
        case ODR_1660HZ:
            timeout = 1000000 / 1660;
            break;
        case ODR_3330HZ:
            timeout = 1000000 / 3330;
            break;
        case ODR_6660HZ:
            timeout = 1000000 / 6660;
            break;
        default:
            break;
    }
    RF_END();
}
}  // namespace aruwsrc::communication::sensors::imu::ism330

#endif  // ISM330_IMPL_HPP_
