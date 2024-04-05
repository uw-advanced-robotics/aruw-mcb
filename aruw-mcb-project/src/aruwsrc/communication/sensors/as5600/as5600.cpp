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

#include "as5600.hpp"

using namespace modm::literals;

namespace aruwsrc::communication::sensors::as5600
{
void AS5600::init()
{
    Board::I2CMaster::connect<Board::I2cScl::Scl, Board::I2CSda::Sda>();
    Board::I2CMaster::initialize<Board::SystemClock, 100'000>();

    writeRegister(AS5600_REG_ZPOS_LOW, AS5600_ZPOS_LOW_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_ZPOS_LOW, AS5600_ZPOS_LOW_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_ZPOS_HIGH, AS5600_ZPOS_HIGH_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_ZPOS_HIGH, AS5600_ZPOS_HIGH_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_MPOS_LOW, AS5600_MPOS_LOW_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_MPOS_LOW, AS5600_MPOS_LOW_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_MPOS_HIGH, AS5600_MPOS_HIGH_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_MPOS_HIGH, AS5600_MPOS_HIGH_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_CONF_LOW, AS5600_CONF_LOW_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_CONF_LOW, AS5600_CONF_LOW_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_CONF_HIGH, AS5600_CONF_HIGH_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_CONF_HIGH, AS5600_CONF_HIGH_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_MANG_HIGH, AS5600_MANG_HIGH_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_MANG_HIGH, AS5600_MANG_HIGH_CONFIG).getResult();
    modm::delay_ms(100);

    writeRegister(AS5600_REG_MANG_LOW, AS5600_MANG_LOW_CONFIG).getResult();
    modm::delay_ms(100);
    writeRegister(AS5600_REG_MANG_LOW, AS5600_MANG_LOW_CONFIG).getResult();
    modm::delay_ms(100);
}

void AS5600::read()
{
    readRegister(AS5600_REG_ZMCO, 1).getResult();
    memcpy(&position_4_high_bits, raw_data_buffer, 1);
    readRegister(AS5600_REG_RAW_ANGLE_HIGH, 2);
    memcpy(&position_8_low_bits, raw_data_buffer, 2);

    actual_val = (position_4_high_bits & 0xF) << 8 | (position_8_low_bits >> 8);
}

void AS5600::readEverything()
{
    readRegister(AS5600_REG_ZMCO, 28);
    actual_val = (everything_possible_potentially[12] & 0xF) << 8 | everything_possible_potentially[13];
}

}  // namespace aruwsrc::communication::sensors::as5600
