/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef MPU6500_TERMINAL_SERIAL_HANDLER_HPP_
#define MPU6500_TERMINAL_SERIAL_HANDLER_HPP_

#include "tap/communication/serial/terminal_serial.hpp"
#include "tap/util_macros.hpp"

namespace tap
{
class Drivers;

namespace sensors
{
class Mpu6500TerminalSerialHandler : public communication::serial::ITerminalSerialCallback
{
public:
    static constexpr char HEADER[] = "imu";

    Mpu6500TerminalSerialHandler(Drivers* drivers) : drivers(drivers) {}

    mockable void init();

    bool terminalSerialCallback(
        char* inputLine,
        modm::IOStream& outputStream,
        bool streamingEnabled) override;

    void terminalSerialStreamCallback(modm::IOStream& outputStream) override;

private:
    static constexpr char USAGE[] =
        "Usage: mpu6500 [-h] [angle] [gyro] [acc]\n"
        "  Where:\n"
        "    - [-h] Prints usage\n"
        "    - [angle] Prints angle data\n"
        "    - [gyro] Prints gyro data\n"
        "    - [accel] Prints accel data\n";

    Drivers* drivers;

    bool printingAngles = false;
    bool printingGyro = false;
    bool printingAccel = false;

    void printHeader(modm::IOStream& outputStream);
};  // class Mpu6500TerminalSerialHandler
}  // namespace sensors
}  // namespace tap

#endif  // MPU6500_TERMINAL_SERIAL_HANDLER_HPP_
