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

#ifndef RPLIDAR_SERIAL_HPP_
#define RPLIDAR_SERIAL_HPP_

namespace aruwsrc::communication::serial::lidar
{

/**
 *
 * Functionality needed:
 * Send start scan msg
 *
 * Parse receieved data
 */

/**
 * Interface protocol: https://bucket-download.slamtec.com/6494fd238cf5e0d881f56d914c6d1f355c0f582a/LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf
 */
class RPLidarSerial
{
public:
    RPLidarSerial();

    void read();

    void startScan();
};

}  // namespace aruwsrc::communication::serial::lidar

#endif  // RPLIDAR_SERIAL_HPP_
