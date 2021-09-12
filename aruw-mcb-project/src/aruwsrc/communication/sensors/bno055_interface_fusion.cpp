#include "bno055_interface_fusion.hpp"

using namespace modm::literals;

namespace aruwsrc::sensors
{
Bno055InterfaceFusion::Bno055InterfaceFusion()
    : r(),
      timer(READ_IMU_DATA_PERIOD),
      unusedData(),
      imu(unusedData, BNO055_ADDR)
{
}

void Bno055InterfaceFusion::initialize()
{
#ifndef PLATFORM_HOSTED
    Bno055I2CMaster::connect<Bno055I2CMasterScl::Scl, Bno055I2CMasterSda::Sda>(
        modm::I2cMaster::PullUps::Internal);
    Bno055I2CMaster::initialize<Board::SystemClock, 400_kHz>();
#endif
}

bool Bno055InterfaceFusion::update()
{
#ifndef PLATFORM_HOSTED
    PT_BEGIN();

    timer.restart(500);

    // ping the device until it responds
    while (true)
    {
        // we wait until the device started
        if (PT_CALL(imu.ping()))
        {
            break;
        }
        PT_WAIT_UNTIL(timer.execute());
    }

    while (true)
    {
        if (PT_CALL(imu.configure()))
        {
            break;
        }

        PT_WAIT_UNTIL(timer.execute());
    }

    timer.restart(READ_IMU_DATA_PERIOD);

    while (true)
    {
        // Read euler angles and gyroscope data.
        PT_WAIT_UNTIL(timer.execute());

        PT_CALL(imu.readRegister(
            modm::bno055::Register::EULER_H_LSB,
            reinterpret_cast<uint8_t *>(&r.e),
            sizeof(r.e)));

        PT_CALL(imu.readRegister(
            modm::bno055::Register::GYRO_DATA_Z_LSB,
            reinterpret_cast<uint8_t *>(&r.g),
            sizeof(r.g)));
    }

    PT_END();
#else
    return false;
#endif
}
}  // namespace aruwsrc::sensors
