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

#ifndef LAUNCHER_RELEASE_SUBSYSTEM_HPP_
#define LAUNCHER_RELEASE_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/pneumatic/gpio_double_solenoid.hpp"

namespace aruwsrc::robot::dart
{
/**
 * Subsystem whose primary purpose is to drive a linear actuator to release the dart launcher.
 */
class LauncherReleaseSubsystem : public tap::control::Subsystem
{
public:
    /**
     * @param drivers Pointer to robot drivers
     * @param linearActuator The actuator to drive
     */
    LauncherReleaseSubsystem(
        tap::Drivers* drivers,
        aruwsrc::control::pneumatic::GpioDoubleSolenoid* linearActuator);

    void initialize() override { stop(); };

    void refresh() override{};

    void refreshSafeDisconnect() override
    {
        stop();
    }

    const char* getName() override { return "Launcher Release"; }

    /**
     * Extends the actuator
     */
    void lockToMotor();

    /**
     * Retracts the actuator
     */
    void releaseMotor();

    void stop();

private:
    tap::Drivers* drivers;
    aruwsrc::control::pneumatic::GpioDoubleSolenoid* linearActuator;
};

}  // namespace aruwsrc::robot::dart

#endif
