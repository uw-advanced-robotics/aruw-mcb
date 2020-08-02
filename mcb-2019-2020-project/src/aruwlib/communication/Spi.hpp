#ifndef SPI_HPP_
#define SPI_HPP_

#include <cstdint>

#ifndef ENV_SIMULATOR
#include <modm/platform.hpp>
#endif

#include "aruwlib/rm-dev-board-a/board.hpp"

namespace aruwlib
{
namespace communication
{
/**
 * \todo Only spi 4 is integrated into this, must also add other spi ports.
 * \todo Investigate nonblocking spi.
 * \todo See what happens when stuff comes unplugged. I suspect the system will crash since
 * I've seen that happen with modm i2c code before.
 */
class Spi
{
public:
#ifndef ENV_SIMULATOR
    using Spi4Nss = modm::platform::GpioE4;
    using Spi4Sck = modm::platform::GpioE12;
    using Spi4Miso = modm::platform::GpioE5;
    using Spi4Mosi = modm::platform::GpioE6;

    using Spi5Sck = modm::platform::GpioF7;
    using Spi5Miso = modm::platform::GpioF8;
    using Spi5Mosi = modm::platform::GpioF9;
    using Spi5Nss = modm::platform::GpioF6;
#endif  // ENV_SIMULATOR
    enum class SpiPort : uint8_t
    {
        SPI_4,
        SPI_5,
    };

    Spi() = default;
    ~Spi() = default;
    Spi &operator=(const Spi &) = default;

    template <
        class SystemClock,
        modm::baudrate_t baudrate,
        modm::percent_t tolerance = modm::pct(5)>
    void initialize(SpiPort spi)
    {
#ifndef ENV_SIMULATOR
        switch (spi)
        {
            case SpiPort::SPI_4:
                modm::platform::SpiMaster4::connect<Spi4Miso::Miso, Spi4Mosi::Mosi, Spi4Sck::Sck>();
                modm::platform::SpiMaster4::initialize<SystemClock, baudrate, tolerance>();
                Spi4Nss::GpioOutput();
                break;
            case SpiPort::SPI_5:
                modm::platform::SpiMaster5::connect<Spi5Miso::Miso, Spi5Mosi::Mosi, Spi5Sck::Sck>();
                modm::platform::SpiMaster5::initialize<SystemClock, baudrate, tolerance>();
                Spi5Nss::GpioOutput();
            default:
                break;
        }
#endif  // ENV_SIMULATOR
    }

    /**
     * Write to a given register.
     */
    void writeRegister(SpiPort spi, uint8_t reg, uint8_t data);

    /**
     * Read from a given register.
     */
    uint8_t readRegister(SpiPort spi, uint8_t reg);

    /**
     * Read from several registers.
     * regAddr is the first address read, and it reads len number of addresses
     * from that point.
     */
    void readRegisters(SpiPort spi, uint8_t regAddr, uint8_t *pData, uint8_t len);

    void nssLow(SpiPort spi);

    void nssHigh(SpiPort spi);
};  // class Spi
}  // namespace communication
}  // namespace aruwlib

#endif  // SPI_HPP_
