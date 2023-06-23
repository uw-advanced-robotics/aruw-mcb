/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include <gtest/gtest.h>

#include "tap/algorithms/crc.hpp"
#include "tap/communication/serial/dji_serial.hpp"
#include "tap/drivers.hpp"

#include "aruw-mcb-project/src/aruwsrc/robot/drone/drone_telemetry_handler.hpp"

using namespace tap::communication::serial;
using namespace tap::arch;
using namespace testing;
using namespace tap;
using namespace tap::algorithms;
using namespace aruwsrc::communication::serial;
