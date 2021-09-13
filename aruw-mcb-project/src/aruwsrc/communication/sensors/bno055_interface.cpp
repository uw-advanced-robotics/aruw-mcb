/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "bno055_interface.hpp"

using namespace modm::literals;

namespace aruwsrc::sensors
{
Bno055Interface::Bno055Interface()
    : r(),
      timer(DELAY_BTWN_CALC_AND_READ_REG),
#ifndef PLATFORM_HOSTED
      unusedData(),
      imu(unusedData, BNO055_ADDR),
#endif
      ready(false)
{
}

void Bno055Interface::initialize()
{
#ifndef PLATFORM_HOSTED
    Bno055I2CMaster::connect<Bno055I2CMasterScl::Scl, Bno055I2CMasterSda::Sda>(
        modm::I2cMaster::PullUps::Internal);
    Bno055I2CMaster::initialize<Board::SystemClock, 400_kHz>();
#endif
}

bool Bno055Interface::update()
{
#ifndef PLATFORM_HOSTED
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
#else
    return true;
#endif
}

void Bno055Interface::periodicIMUUpdate()
{
    if (ready)
    {
        // Note: magnetometer units don't matter, Mahony ARHS normalizes!
        ahrsAlgorithm.updateIMU(getGx(), getGy(), getGz(), getAx(), getAy(), getAz());
        // ahrsAlgorithm.update(getGx(), getGy(), getGz(), getAx(), getAy(), getAz(), getMx(),
        // getMy(), getMz());

        timer.restart(DELAY_BTWN_CALC_AND_READ_REG);
    }
}
}  // namespace aruwsrc::sensors
