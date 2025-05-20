/*
 * Copyright (c) 2025-2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef ARUCO_RESET_SUBSYSTEM_HPP_
#define ARUCO_RESET_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"

#include "aruwsrc/algorithms/odometry/deadwheel_chassis_kf_odometry.hpp"
#include "aruwsrc/algorithms/odometry/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"

namespace aruwsrc::control::aruco
{
using namespace aruwsrc::algorithms::odometry;
using namespace aruwsrc::serial;
using namespace aruwsrc::algorithms::transforms;
using namespace tap::algorithms::odometry;

class ArucoResetSubsystem : public tap::control::Subsystem
{
public:
    ArucoResetSubsystem(
        tap::Drivers* drivers,
        VisionCoprocessor& vision,
        Odometry2DInterface& odometry,
        TransformerInterface& transformer);

    void initialize() override{};

    void refresh() override;

    const char* getName() const override { return "Aruco Reset Subsystem"; }

private:
    VisionCoprocessor& vision;
    Odometry2DInterface& odometry;
    TransformerInterface& transformer;

    // Higher value here means we trust AruCo measurements more
    float VISION_TRUST = 0.05f;

    void processRealsenseData();
    void processArducamData();

};  // class ArucoResetSubsystem

}  // namespace aruwsrc::control::aruco
#endif  // ARUCO_RESET_SUBSYSTEM_HPP_
