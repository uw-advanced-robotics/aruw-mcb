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

namespace aruwsrc::communication::sensors::imu
{
template <class I2cMaster>
ISM330<I2cMaster>::ISM330() : modm::I2cDevice<I2cMaster>(DEVICE_ADDRESS),
                              AbstractIMU(nullptr)
{
}

template <class I2cMaster>
void ISM330<I2cMaster>::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    AbstractIMU::initialize(sampleFrequency, mahonyKp, mahonyKi);

    // Check Who Am I
    RF_CALL_BLOCKING(readRegister(WHO_AM_I, 3, rxConfig));

    setODR(ODR_833HZ);
    setGyroRange(DPS1000_CONFIG);
    setAccelRange(G4_CONFIG);

    readTimeout.restart(timeout);
}

template <class I2cMaster>
void ISM330<I2cMaster>::read()
{
    count++;

    if (!readTimeout.execute())
    {
        return;
    }

    succcess++;

    readTimeout.restart(timeout);

    pinged = RF_CALL_BLOCKING(this->ping());

    uint8_t rxBuff[15];
    bool readWorking = RF_CALL_BLOCKING(readRegister(OUT_TEMP_L, READ_LENGTH, rxBuff));
    if (!readWorking)
    {
        return;
    }

    imuData.temperature = tempValueToCelsius(rxBuff);

    float gyroX = gyroValueToDegPerSec(rxBuff + 2);
    float gyroY = gyroValueToDegPerSec(rxBuff + 4);
    float gyroZ = gyroValueToDegPerSec(rxBuff + 6);

    imuData.gyroRaw = {gyroX, gyroY, gyroZ};

    float accX = accelValueToMeterPerSec(rxBuff + 8);
    float accY = accelValueToMeterPerSec(rxBuff + 10);
    float accZ = accelValueToMeterPerSec(rxBuff + 12);

    imuData.accRaw = {accX, accY, accZ};

    imuData.gyroDegPerSec = imuData.gyroRaw - imuData.gyroOffsetRaw;
    imuData.accG = imuData.accRaw - imuData.accOffsetRaw;

    prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();
}

template <class I2cMaster>
void ISM330<I2cMaster>::setAccelRange(XL_Config xl_config)
{
    RF_CALL_BLOCKING(readRegister(CTRL1_XL, 1, &current_reg_XL));

    RF_CALL_BLOCKING(writeRegister(CTRL1_XL, (current_reg_XL & G_CONFIG_BITMASK) | xl_config));
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
            accelScale = 99;
            break;
    }
}

template <class I2cMaster>
void ISM330<I2cMaster>::setGyroRange(Gyro_Config g_config)
{
    RF_CALL_BLOCKING(readRegister(CTRL2_G, 1, &current_reg_G));

    RF_CALL_BLOCKING(writeRegister(CTRL2_G, (current_reg_G & DPS_CONFIG_BITMASK) | g_config));
    switch (g_config)
    {
        case DPS250_CONFIG:
            gyroScale = 8.75;
            break;
        case DPS500_CONFIG:
            gyroScale = 17.50;
            break;
        case DPS1000_CONFIG:
            gyroScale = 35;
            break;
        case DPS2000_CONFIG:
            gyroScale = 70;
            break;
        default:
            break;
    }
}

template <class I2cMaster>
void ISM330<I2cMaster>::setODR(ODR odr)
{
    RF_CALL_BLOCKING(readRegister(CTRL1_XL, 1, &current_reg_XL));
    RF_CALL_BLOCKING(readRegister(CTRL2_G, 1, &current_reg_G));

    RF_CALL_BLOCKING(writeRegister(CTRL1_XL, (current_reg_XL & ODR_BITMASK) | odr));
    RF_CALL_BLOCKING(writeRegister(CTRL2_G, (current_reg_G & ODR_BITMASK) | odr));

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
}
}  // namespace aruwsrc::communication::sensors::imu
