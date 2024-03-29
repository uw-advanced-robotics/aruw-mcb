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

namespace aruwsrc::communication::sensors::as5600v2
{
void AS5600::init()
{
    Board::I2CMaster::connect<Board::I2cScl::Scl, Board::I2CSda::Sda>();
    Board::I2CMaster::initialize<Board::SystemClock, 400000_Hz>();

    // Wait to get a result
    while (!readRegister(AS5600_REG_ANGLE_LOW, 2).getResult())
        ;
    modm::delay_ms(1);
    while (!writeRegister(AS5600_REG_ZPOS_LOW, AS5600_ZPOS_LOW_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while (!writeRegister(AS5600_REG_ZPOS_HIGH, AS5600_ZPOS_HIGH_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while (!writeRegister(AS5600_REG_MPOS_LOW, AS5600_MPOS_LOW_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while (!writeRegister(AS5600_REG_MPOS_HIGH, AS5600_MPOS_HIGH_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while(!writeRegister(AS5600_REG_MANG_HIGH, AS5600_MANG_HIGH_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while(!writeRegister(AS5600_REG_MANG_LOW, AS5600_MANG_LOW_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while (!writeRegister(AS5600_REG_CONF_LOW, AS5600_CONF_LOW_CONFIG).getResult())
        ;
    modm::delay_ms(1);
    while (!writeRegister(AS5600_REG_CONF_HIGH, AS5600_CONF_HIGH_CONFIG).getResult())
        ;
    modm::delay_ms(1);
}

void AS5600::read(){
    readRegister(AS5600_REG_ANGLE_HIGH, 2);
}


};  // namespace aruwsrc::communication::sensors::as5600v2
