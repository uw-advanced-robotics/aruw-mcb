/*
 * Copyright (c) 2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef AS5600_HPP_
#define AS5600_HPP_

#include "tap/board/board.hpp"

#include "modm/architecture/interface/i2c_device.hpp"
#include "modm/processing/resumable.hpp"

#include "as5600_register.hpp"

namespace aruwsrc::communication::sensors::as5600
{
class AS5600 : public modm::I2cDevice<Board::I2CMaster>
{
public:
    AS5600() : modm::I2cDevice<Board::I2CMaster>(AS5600_ADDRESS){};

    // This will setup the I2C master?? TAMU initalizes the i2c here. I think i do this in the
    // multiplexer Do config here
    void init();

    void read();

    void getPosition();

    void getVelocity();

    void readEverything();

private:
    inline modm::ResumableResult<bool> readRegister(uint8_t reg, int length)
    {
        RF_BEGIN();

        raw_data_buffer[0] = reg;

        while (!transaction.configureWriteRead(raw_data_buffer, 1, everything_possible_potentially, length))
        {
        };

        RF_END_RETURN_CALL(runTransaction());
    };

    modm::ResumableResult<bool> writeRegister(uint8_t reg, uint8_t data)
    {
        RF_BEGIN();

        raw_data_buffer[0] = reg;
        raw_data_buffer[1] = data;

        while (!transaction.configureWrite(raw_data_buffer, 2))
        {
        };

        RF_END_RETURN_CALL(runTransaction());
    };

    uint16_t position_8_low_bits = 0;
    uint8_t position_4_high_bits = 0;  // ZMCO
    uint16_t actual_val;

    uint8_t raw_data_buffer[2];

    uint8_t everything_possible_potentially[28];
};

}  // namespace aruwsrc::communication::sensors::as5600

#endif  // AS5600_HPP_
