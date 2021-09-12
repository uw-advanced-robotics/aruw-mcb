#include "bno055_interface.hpp"
#include "tap/drivers_singleton.hpp"
using namespace modm::literals;

uint32_t t1, t2;

namespace aruwsrc::sensors
{
Bno055Interface::Bno055Interface()
    : r(),
      timer(DELAY_BTWN_CALC_AND_READ_REG),
      unusedData(),
      imu(unusedData, BNO055_ADDR),
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

        t1 = tap::arch::clock::getTimeMicroseconds();

        PT_CALL(imu.readRegister(
            modm::bno055::Register::ACCEL_DATA_X_LSB,
            reinterpret_cast<uint8_t *>(&r),
            1));

        t2 = tap::arch::clock::getTimeMicroseconds() - t1;
        tap::DoNotUse_getDrivers()->terminalSerial.getStream().printf("%li\n", t2);


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


/*

reset device:

    while (true)
    {
        if (PT_CALL(imu.updateRegister(modm::bno055::Register::SYS_TRIGGER, static_cast<modm::bno055::SystemTrigger_t>(0))))
        {
            break;
        }

        PT_WAIT_UNTIL(timer.execute());
        timer.restart(500'000);
    }


*/