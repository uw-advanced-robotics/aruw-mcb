#include "ism330_spi.hpp"

#include "tap/algorithms/math_user_utils.hpp"
#include "tap/board/board.hpp"
#include "aruwsrc/communication/sensors/imu/ism330/ism330_data.hpp"
#include <cassert>

using namespace modm::literals;

namespace aruwsrc::communication::sensors::imu::ism330
{
using namespace tap::communication::sensors::imu;
Ism330Spi::Ism330Spi() : AbstractIMU(){};

void Ism330Spi::initialize(float sampleFrequency, float mahonyKp, float mahonyKi)
{
    AbstractIMU::initialize(sampleFrequency, mahonyKp, mahonyKi);
#ifndef PLATFORM_HOSTED
    Board::ImuNss::GpioOutput();
    Board::ImuSpiMaster::connect<Board::ImuMiso::Miso, Board::ImuMosi::Mosi, Board::ImuSck::Sck>();
    Board::ImuSpiMaster::initialize<Board::SystemClock, 5625000_Hz>();

    assert (spiReadRegister(Register::WHO_AM_I) == 0x6B);
    setODR(ODR_833HZ);
    setGyroRange(DPS1000_CONFIG);
    setAccelRange(G4_CONFIG);
#endif
}

bool Ism330Spi::read()
{
#ifndef PLATFORM_HOSTED
    float gyroX, gyroY, gyroZ, accX, accY, accZ;

    PT_BEGIN();
    while (true) {
        PT_WAIT_UNTIL(readTimeout.execute());
        ismNssLow();
        tx = OUT_TEMP_L | ISM330_READ_BIT;
        
        PT_CALL(Board::ImuSpiMaster::transfer(&tx, &rx, 1));
        PT_CALL(Board::ImuSpiMaster::transfer(txBuff, rxBuff, READ_LENGTH));
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
    }
    PT_END();
    return true;
#endif
    return false;
}



void Ism330Spi::spiWriteRegister(uint8_t reg, uint8_t data)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    UNUSED(data);
#else
    ismNssLow();
    uint8_t tx = reg & ~ISM330_WRITE_BIT;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    tx = data;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    ismNssHigh();
#endif
}

uint8_t Ism330Spi::spiReadRegister(uint8_t reg)
{
#ifdef PLATFORM_HOSTED
    UNUSED(reg);
    return 0;
#else
    ismNssLow();
    uint8_t tx = reg | ISM330_READ_BIT;
    uint8_t rx = 0;
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    Board::ImuSpiMaster::transferBlocking(&tx, &rx, 1);
    ismNssHigh();
    return rx;
#endif
}

void Ism330Spi::ismNssLow() {
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::Low);
#endif
}

void Ism330Spi::ismNssHigh() {
#ifndef PLATFORM_HOSTED
    Board::ImuNss::setOutput(modm::GpioOutput::High);
#endif
}

void Ism330Spi::setODR(OutputDataRate odr) {
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

void Ism330Spi::setAccelRange(AccelerometerRangeConfig xl_config) {
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

void Ism330Spi::setGyroRange(GyroscopeRangeConfig g_config) {
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