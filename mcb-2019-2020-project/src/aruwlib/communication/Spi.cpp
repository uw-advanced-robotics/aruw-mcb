#include "Spi.hpp"

namespace aruwlib
{
namespace communication
{
void Spi::writeRegister(SpiPort spi, uint8_t reg, uint8_t data)
{
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
}

uint8_t Spi::readRegister(SpiPort spi, uint8_t reg)
{
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
}

uint8_t Spi::readRegisters(SpiPort spi, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
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
    return 0;
}

void Spi::nssLow(SpiPort spi)
{
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
}

void Spi::nssHigh(SpiPort spi)
{
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
}
}  // namespace communication
}  // namespace aruwlib
