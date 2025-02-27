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
                              updateTimeout(timeout)
{
}

template <class I2cMaster>
void ISM330<I2cMaster>::init()
{
    // Check Who Am I
    RF_CALL_BLOCKING(readRegister(WHO_AM_I, 3, rxConfig));

    setODR(ODR_833HZ);
    setAccelRange(G4_CONFIG);
    setGyroRange(DPS1000_CONFIG);
    updateTimeout.restart(timeout);
}

template <class I2cMaster>
void ISM330<I2cMaster>::read()
{
    if (!updateTimeout.execute())
    {
        return;
    }

    updateTimeout.restart(timeout);

    pinged = RF_CALL_BLOCKING(this->ping());

    uint8_t rxBuff[15];
    bool readWorking = RF_CALL_BLOCKING(readRegister(OUT_TEMP_L, READ_LENGTH, rxBuff));
    if (!readWorking)
    {
        return;
    }

    imuData.temperature = tempValueToCelsius(rxBuff);

    imuData.gyroRaw[ImuData::X] = gyroValueToDegPerSec(rxBuff + 2);
    imuData.gyroRaw[ImuData::Y] = gyroValueToDegPerSec(rxBuff + 4);
    imuData.gyroRaw[ImuData::Z] = gyroValueToDegPerSec(rxBuff + 6);

    imuData.accRaw[ImuData::X] = accelValueToG(rxBuff + 8);
    imuData.accRaw[ImuData::Y] = accelValueToG(rxBuff + 10);
    imuData.accRaw[ImuData::Z] = accelValueToG(rxBuff + 12);
}

template <class I2cMaster>
void ISM330<I2cMaster>::setAccelRange(XL_Config xl_config)
{
    RF_CALL_BLOCKING(readRegister(CTRL1_XL, READ_LENGTH, &current_reg_XL));

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
            break;
    }
}

template <class I2cMaster>
void ISM330<I2cMaster>::setGyroRange(Gyro_Config g_config)
{
    RF_CALL_BLOCKING(readRegister(CTRL2_G, READ_LENGTH, &current_reg_G));

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
    RF_CALL_BLOCKING(readRegister(CTRL1_XL, READ_LENGTH, &current_reg_XL));
    RF_CALL_BLOCKING(readRegister(CTRL2_G, READ_LENGTH, &current_reg_G));

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

// template <class I2cMaster>
// modm::ResumableResult<bool> ISM330<I2cMaster>::readRegister(
//     uint8_t reg,
//     int length,
//     uint8_t *rxBuffer)
// {
//     uint8_t txBuff = reg;

//     RF_BEGIN();

//     RF_WAIT_WHILE(!this->transaction.configureWriteRead(&txBuff, 1, rxBuffer, length));

//     RF_END_RETURN_CALL(this->runTransaction());
// };

// template <class I2cMaster>
// modm::ResumableResult<bool> ISM330<I2cMaster>::writeRegister(
//     uint8_t reg,
//     uint8_t data)
// {
//     uint8_t txBuff[2] = {reg, data};

//     RF_BEGIN();

//     RF_WAIT_WHILE(!this->transaction.configureWrite(txBuff, 2));

//     RF_END_RETURN_CALL(this->runTransaction());
// };

}  // namespace aruwsrc::communication::sensors::imu
