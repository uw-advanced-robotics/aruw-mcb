/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_SCAN_COMMAND_HPP_
#define TURRET_SCAN_COMMAND_HPP_

#include "tap/control/command.hpp"

#include "../algorithms/turret_controller_interface.hpp"

#include "setpoint_scanner.hpp"

namespace aruwsrc::control::turret
{
class TurretSubsystem;
}

namespace aruwsrc::control::turret::cv
{
class TurretScanCommand : public tap::control::Command
{
public:
    struct Config
    {
        /**
         * Low pass filter alpha used to smooth turret scanning
         */
        float scanLowPassAlpha;
        /**
         * The number of times refresh is called without receiving valid CV data to when
         * the command will consider the target lost and start tracking.
         */
        uint32_t aimLostNumCounts;
        SetpointScanner::Config pitchScanConfig;
        SetpointScanner::Config yawScanConfig;
    };

    TurretScanCommand(
        TurretSubsystem &turret,
        algorithms::TurretControllerInterface &yawController,
        algorithms::TurretControllerInterface &pitchController,
        const Config &config);

    const char *getName() const override { return "Turret scan"; }
    bool isReady() override;
    void initialize() override;
    void execute() override;
    void end(bool interrupted) override;
    bool isFinished() const override;

private:
    TurretSubsystem &turret;
    algorithms::TurretControllerInterface &yawController;
    algorithms::TurretControllerInterface &pitchController;
    Config config;

    /**
     * Handles scanning logic in the pitch direction. The scan angle is a chassis frame angle.
     */
    SetpointScanner pitchScanner;

    /**
     * Handles scanning logic in the yaw direction. The scan angle is a chassis frame angle.
     */
    SetpointScanner yawScanner;

    uint32_t prevTime = 0;

    /**
     * A counter that is reset to 0 every time CV starts tracking a target
     * and that keeps track of the number of times `refresh` is called when
     * an aiming solution couldn't be found (either because CV had no target
     * or aiming solution was impossible)
     */
    uint32_t lostTargetCounter = 0;
};
}  // namespace aruwsrc::control::turret::cv

#endif  // TURRET_SCAN_COMMAND_HPP_
