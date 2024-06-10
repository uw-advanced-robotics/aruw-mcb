/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef SENTRY_TURRET_CV_COMMAND_ADAPTER_HPP_
#define SENTRY_TURRET_CV_COMMAND_ADAPTER_HPP_

#include "tap/control/command.hpp"

#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

#include "sentry_turret_cv_command.hpp"
#include "turret_cv_command_interface.hpp"

namespace aruwsrc::control::sentry
{
/**
 * Adapts a SentryTurretCVCommand to overcome design flaws regarding the TurretCVCommandInterface.
 */
class SentryTurretCVCommandAdapter : public aruwsrc::control::turret::cv::TurretCVCommandInterface
{
public:
    SentryTurretCVCommandAdapter(
        aruwsrc::control::sentry::SentryTurretCVCommand& sentryTurretCVCommand,
        uint8_t turretID)
        : sentryTurretCVCommand(sentryTurretCVCommand),
          turretID(turretID)
    {
    }

    void initialize() override {}

    bool isReady() override { return true; }

    void execute() override {}

    bool isFinished() const override { return false; }

    void end(bool) override {}

    const char* getName() const override { return "turret CV"; }

    bool getTurretID() const override { return turretID; }

    bool isAimingWithinLaunchingTolerance() const override
    {
        return sentryTurretCVCommand.isAimingWithinLaunchingTolerance(turretID);
    }

private:
    SentryTurretCVCommand& sentryTurretCVCommand;
    uint8_t turretID;
};
}  // namespace aruwsrc::control::sentry

#endif  // SENTRY_TURRET_CV_COMMAND_ADAPTER_HPP_
