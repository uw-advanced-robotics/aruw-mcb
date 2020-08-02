#include "Spi.hpp"

namespace aruwlib
{
namespace communication
{
void Spi::writeRegister(SpiPort spi, uint8_t reg, uint8_t data)
{
#ifndef ENV_SIMULATOR
    nssLow(spi);
    uint8_t tx = reg & 0x7F;
    uint8_t rx = 0;
    switch (spi)
    {
        case SpiPort::SPI_4:
            modm::platform::SpiMaster4::transferBlocking(&tx, &rx, 1);
            tx = data;
            modm::platform::SpiMaster4::transferBlocking(&tx, &rx, 1);
            break;
        case SpiPort::SPI_5:
            modm::platform::SpiMaster5::transferBlocking(&tx, &rx, 1);
            tx = data;
            modm::platform::SpiMaster5::transferBlocking(&tx, &rx, 1);
            break;
        default:
            break;
    }
    nssHigh(spi);
#endif  // ENV_SIMULATOR
}

uint8_t Spi::readRegister(SpiPort spi, uint8_t reg)
{
#ifndef ENV_SIMULATOR
    nssLow(spi);
    uint8_t tx = reg | 0x80;
    uint8_t rx = 0;
    switch (spi)
    {
        case SpiPort::SPI_4:
            modm::platform::SpiMaster4::transferBlocking(&tx, &rx, 1);
            modm::platform::SpiMaster4::transferBlocking(&tx, &rx, 1);
            break;
        case SpiPort::SPI_5:
            modm::platform::SpiMaster5::transferBlocking(&tx, &rx, 1);
            modm::platform::SpiMaster5::transferBlocking(&tx, &rx, 1);
            break;
        default:
            break;
    }
    nssHigh(spi);
    return rx;
#else
    return 0;
#endif  // ENV_SIMULATOR
}

void Spi::readRegisters(SpiPort spi, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
#ifndef ENV_SIMULATOR
    nssLow(spi);
    uint8_t tx = regAddr | 0x80;
    uint8_t rx = 0;
    switch (spi)
    {
        case SpiPort::SPI_4:
            modm::platform::SpiMaster4::transferBlocking(&tx, &rx, 1);
            modm::platform::SpiMaster4::transferBlocking(&tx, pData, len);
            break;
        case SpiPort::SPI_5:
            modm::platform::SpiMaster5::transferBlocking(&tx, &rx, 1);
            modm::platform::SpiMaster5::transferBlocking(&tx, pData, len);
            break;
        default:
            break;
    }
    nssHigh(spi);
#endif  // ENV_SIMULATOR
}

void Spi::nssLow(SpiPort spi)
{
#ifndef ENV_SIMULATOR
    switch (spi)
    {
        case SpiPort::SPI_4:
            Spi4Nss::setOutput(modm::GpioOutput::Low);
            break;
        case SpiPort::SPI_5:
            Spi5Nss::setOutput(modm::GpioOutput::Low);
        default:
            break;
    }
#endif  // ENV_SIMULATOR
}

void Spi::nssHigh(SpiPort spi)
{
#ifndef ENV_SIMULATOR
    switch (spi)
    {
        case SpiPort::SPI_4:
            Spi4Nss::setOutput(modm::GpioOutput::High);
            break;
        case SpiPort::SPI_5:
            Spi5Nss::setOutput(modm::GpioOutput::High);
        default:
            break;
    }
#endif  // ENV_SIMULATOR
}
}  // namespace communication
}  // namespace aruwlib
