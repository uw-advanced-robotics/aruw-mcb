/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef ISM330_HPP_
#define ISM330_HPP_

#include "tap/architecture/periodic_timer.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/processing/resumable.hpp"

namespace aruwsrc::communication::sensors::imu
{

template <class I2cMaster>
class ISM330 : public modm::I2cDevice<I2cMaster>
{
public:
    static constexpr int DEVICE_ADDRESS = 0x6A;

    ISM330() : modm::I2cDevice<I2cMaster>(DEVICE_ADDRESS), pinged(false), updateTimeout(500.0f) {}

    void read(){
        if(!updateTimeout.execute()){
            return;
        }

        pinged = RF_CALL_BLOCKING(this->ping());

        RF_CALL_BLOCKING(readRegister(0x20, 14));
    }
private:
    modm::ResumableResult<bool> readRegister(uint8_t reg, int length)
    {
        RF_BEGIN();

        txBuff[0] = reg;

        RF_WAIT_WHILE(!this->transaction.configureWriteRead(txBuff, 1, rxBuff, length));

        RF_END_RETURN_CALL(this->runTransaction());
    };

    bool pinged;
    uint8_t txBuff[2];
    uint8_t rxBuff[30];
    tap::arch::PeriodicMilliTimer updateTimeout;
};

}  // namespace aruwsrc::communication::sensors::imu

#endif  // ISM330_HPP_
