/*
 * Copyright (c) 2020-2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifdef TARGET_TESTBED

#ifndef TMOTOR_SUBSYSTEM_HPP_
#define TMOTOR_SUBSYSTEM_HPP_

#include "tap/control/subsystem.hpp"
#include "tap/errors/create_errors.hpp"

#include "aruwsrc/motor/tmotor_ak80-9.hpp"

namespace aruwsrc::testbed
{
class TMotorSubsystem : public tap::control::Subsystem
{
public:
    explicit TMotorSubsystem(
        tap::Drivers* drivers,
        aruwsrc::motor::Tmotor_AK809* tmotor
    );

    void initialize() override;

    void refresh() override;

    const char* getName() override { return tmotor->getName(); }

    void onHardwareTestStart() override;

    inline void setDesiredOutput(int16_t output) { desiredOutput = output; };

private:
    tap::Drivers* drivers;
    aruwsrc::motor::Tmotor_AK809* tmotor;
    int16_t desiredOutput;
};
}

#endif  // TMOTOR_SUBSYSTEM_HPP_

#endif  // TARGET_TESTBED
