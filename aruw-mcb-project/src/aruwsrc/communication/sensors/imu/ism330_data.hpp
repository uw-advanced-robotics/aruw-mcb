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

#pragma once
namespace aruwsrc::communication::sensors::ism330
{

#define DEVICE_ADDRESS 0x6B

#define CTRL1_XL 0x10

#define ACCELEROMETER_CONFIG ((0b0111 << 4) | 0x0)  // 833hz, 2g

#define CTRL2_G 0x11

#define GYRO_CONFIG ((0b0111 << 4) | 0x0)  // 833hz

#define READ_START 0x20
#define READ_LENGTH 13

}  // namespace aruwsrc::communication::sensors::ism330
