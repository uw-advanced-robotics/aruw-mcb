/*****************************************************************************/
/********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
/*****************************************************************************/

/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "integrable_setpoint_subsystem_mock.hpp"

namespace tap::mock
{
IntegrableSetpointSubsystemMock::IntegrableSetpointSubsystemMock(Drivers *drivers)
    : Subsystem(drivers)
{
    // Default to simulating an online and unjammed setpointSubsystem
    ON_CALL(*this, isOnline).WillByDefault(testing::Return(true));
    ON_CALL(*this, isJammed).WillByDefault(testing::Return(false));
}
IntegrableSetpointSubsystemMock::~IntegrableSetpointSubsystemMock() {}
}  // namespace tap::mock
