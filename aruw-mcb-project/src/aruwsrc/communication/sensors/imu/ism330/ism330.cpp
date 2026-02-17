/*
 * Copyright (c) 2024-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "ism330.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/board/board.hpp"

#include "ism330_data.hpp"

using namespace modm::literals;

namespace aruwsrc::communication::sensors::imu::ism330
{
using namespace tap::communication::sensors::imu;
ISM330* ISM330::spiOwner = nullptr;
ISM330::ISM330(ChipSelectPin chipSelectPin) : AbstractIMU(), chipSelectPin(chipSelectPin){};

void ISM330::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    AbstractIMU::initialize(sampleFrequency, mahonyKp, mahonyKi);
#ifndef PLATFORM_HOSTED
    switch (chipSelectPin)
    {
        case ChipSelectPin::GPIO_D12_H_ROW: modm::platform::GpioD12::GpioOutput(); break;
        case ChipSelectPin::BOARD_SPI_NSS:
        default: Board::SpiNss::GpioOutput(); break;
    }
    ismNssHigh();
    Board::GenSpiMaster::connect<Board::SpiMiso::Miso, Board::SpiMosi::Mosi, Board::SpiSck::Sck>();
    Board::GenSpiMaster::initialize<Board::SystemClock, 5625000_Hz>();
    Board::GenSpiMaster::setDataMode(Board::GenSpiMaster::DataMode::Mode3);
    modm::delay_ms(10);
    setODR(DEFAULT_ODR);
    setGyroRange(DEFAULT_GYRO_RANGE);
    setAccelRange(DEFAULT_ACCEL_RANGE);
    counter = spiReadRegister(WHO_AM_I);
#endif
}

bool ISM330::read()
{
#ifndef PLATFORM_HOSTED
    float gyroX, gyroY, gyroZ, accX, accY, accZ;

    PT_BEGIN();
    while (true)
    {
        PT_WAIT_UNTIL(readTimeout.execute());
        PT_WAIT_UNTIL((spiOwner == nullptr) || (spiOwner == this));
        spiOwner = this;

        tx = CTRL1_XL | ISM330_READ_BIT;
        rx = 0;
        ismNssLow();
        PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
        ismNssHigh();

        // Sketchy check to see if IMU is was reset.
        if (rx != DEFAULT_CTRL1_XL_VALUE)
        {
            // manually set xl and gyro to default values.
            tx = CTRL1_XL & ISM330_WRITE_BIT;
            rx = 0;
            ismNssLow();
            PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
            tx = DEFAULT_CTRL1_XL_VALUE;
            PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
            ismNssHigh();

            tx = CTRL2_G & ISM330_WRITE_BIT;
            rx = 0;
            ismNssLow();
            PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
            tx = DEFAULT_CTRL2_G_VALUE;
            PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
            ismNssHigh();
            // zero out stuff
            imuData.gyroRaw = {0, 0, 0};
            imuData.accRaw = {0, 0, 0};
            imuData.accG = {0, 0, 0};
            imuData.gyroRadPerSec = {0, 0, 0};
            // Device not connected
            if (imuState != ImuState::IMU_NOT_CONNECTED)
            {
                prevImuState = imuState;
            }
            imuState = ImuState::IMU_NOT_CONNECTED;
            spiOwner = nullptr;
            // We don't want to update IMU received data time.
            continue;
        }

        tx = OUT_TEMP_L | ISM330_READ_BIT;
        rx = 0;

        ismNssLow();
        PT_CALL(Board::GenSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::GenSpiMaster::transfer(txBuff, rxBuff, READ_LENGTH));
        ismNssHigh();

        imuData.temperature = tempValueToCelsius(rxBuff);
        gyroX = gyroValueToRadPerSec(rxBuff + 2);
        gyroY = gyroValueToRadPerSec(rxBuff + 4);
        gyroZ = gyroValueToRadPerSec(rxBuff + 6);

        accX = accelValueToMeterPerSec(rxBuff + 8);
        accY = accelValueToMeterPerSec(rxBuff + 10);
        accZ = accelValueToMeterPerSec(rxBuff + 12);

        imuData.gyroRaw = {gyroX, gyroY, gyroZ};
        imuData.accRaw = {accX, accY, accZ};

        applyMountingTransformToRaw(imuData);

        imuData.gyroRadPerSec = imuData.gyroRaw - imuData.gyroOffsetRaw;
        imuData.accG = imuData.accRaw - imuData.accOffsetRaw;

        prevIMUDataReceivedTime = tap::arch::clock::getTimeMicroseconds();

        if (imuState == ImuState::IMU_NOT_CONNECTED)
        {
            imuState = prevImuState;
        }
        spiOwner = nullptr;
    }
    PT_END();
    return true;
#endif
    return false;
}

void ISM330::spiWriteRegister(uint8_t reg, uint8_t data)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    UNUSED(data);
#else
    ismNssLow();
    uint8_t tx = reg & ISM330_WRITE_BIT;
    uint8_t rx = 0;
    Board::GenSpiMaster::transferBlocking(&tx, &rx, 1);
    tx = data;
    Board::GenSpiMaster::transferBlocking(&tx, &rx, 1);
    ismNssHigh();
#endif
}

uint8_t ISM330::spiReadRegister(uint8_t reg)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    return 0;
#else
    ismNssLow();
    uint8_t tx = reg | ISM330_READ_BIT;
    uint8_t rx = 0;
    Board::GenSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::GenSpiMaster::transferBlocking(&tx, &rx, 1);
    ismNssHigh();
    return rx;
#endif
}

void ISM330::ismNssLow()
{
#ifndef PLATFORM_HOSTED
    switch (chipSelectPin)
    {
        case ChipSelectPin::GPIO_D12_H_ROW:
            modm::platform::GpioD12::setOutput(modm::GpioOutput::Low);
            break;
        case ChipSelectPin::BOARD_SPI_NSS:
        default: Board::SpiNss::setOutput(modm::GpioOutput::Low); break;
    }
#endif
}

void ISM330::ismNssHigh()
{
#ifndef PLATFORM_HOSTED
    switch (chipSelectPin)
    {
        case ChipSelectPin::GPIO_D12_H_ROW:
            modm::platform::GpioD12::setOutput(modm::GpioOutput::High);
            break;
        case ChipSelectPin::BOARD_SPI_NSS:
        default: Board::SpiNss::setOutput(modm::GpioOutput::High); break;
    }
#endif
}

void ISM330::setODR(OutputDataRate odr)
{
    uint8_t current_reg_XL = spiReadRegister(CTRL1_XL);
    uint8_t current_reg_G = spiReadRegister(CTRL2_G);

    spiWriteRegister(CTRL1_XL, (current_reg_XL & ODR_BITMASK) | odr);
    spiWriteRegister(CTRL2_G, (current_reg_G & ODR_BITMASK) | odr);
    uint32_t timeout{1200};
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

    readTimeout.restart(timeout);
}

void ISM330::setAccelRange(AccelerometerRangeConfig xl_config)
{
    uint8_t current_reg_XL = spiReadRegister(CTRL1_XL);
    spiWriteRegister(CTRL1_XL, (current_reg_XL & G_CONFIG_BITMASK) | xl_config);

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

void ISM330::setGyroRange(GyroscopeRangeConfig g_config)
{
    uint8_t current_reg_G = spiReadRegister(CTRL2_G);
    spiWriteRegister(CTRL2_G, (current_reg_G & DPS_CONFIG_BITMASK) | g_config);
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
}

}  // namespace aruwsrc::communication::sensors::imu::ism330
