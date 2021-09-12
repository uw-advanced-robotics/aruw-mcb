#include "bno055_interface.hpp"

using namespace modm::literals;

namespace aruwsrc::sensors
{
Bno055Interface::Bno055Interface()
    : r(),
      timer(DELAY_BTWN_CALC_AND_READ_REG),
      unusedData(),
#ifndef PLATFORM_HOSTED
      imu(unusedData, BNO055_ADDR),
#endif
      ready(false)
{
}

void Bno055Interface::initialize()
{
    Bno055I2CMaster::connect<Bno055I2CMasterScl::Scl, Bno055I2CMasterSda::Sda>(
        modm::I2cMaster::PullUps::Internal);
    Bno055I2CMaster::initialize<Board::SystemClock, 400_kHz>();
}

bool Bno055Interface::update()
{
    PT_BEGIN();

    timer.restart(500'000);

    // ping the device until it responds
    while (true)
    {
        // we wait until the device started
        if (PT_CALL(imu.ping()))
        {
            break;
        }
        PT_WAIT_UNTIL(timer.execute());
        timer.restart(500'000);
    }

    while (true)
    {
        if (PT_CALL(imu.configure(modm::bno055::OperationMode::AMG)))
        {
            break;
        }

        PT_WAIT_UNTIL(timer.execute());
        timer.restart(500'000);
    }

    timer.restart(DELAY_BTWN_CALC_AND_READ_REG);

    while (true)
    {
        // Read acceleration/gyroscope/magnetometer data
        PT_WAIT_UNTIL(timer.execute());

        PT_CALL(imu.readRegister(
            modm::bno055::Register::ACCEL_DATA_X_LSB,
            reinterpret_cast<uint8_t *>(&r),
            sizeof(r)));

        ready = true;
    }

    PT_END();
}

void Bno055Interface::periodicIMUUpdate()
{
    if (ready)
    {
        // Note: magnetometer units don't matter, Mahony ARHS normalizes!
        ahrsAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        // ahrsAlgorithm.update(getGx(), getGy(), getGz(), getAx(), getAy(), getAz(), getMx(), getMy(), getMz());

        timer.restart(DELAY_BTWN_CALC_AND_READ_REG);
    }
}
}  // namespace aruwsrc::sensors
