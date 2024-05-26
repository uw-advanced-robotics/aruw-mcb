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

#ifndef TURRET_CONTROLLER_INTERFACE_MOCK_HPP_
#define TURRET_CONTROLLER_INTERFACE_MOCK_HPP_

#include <gmock/gmock.h>

#include "aruwsrc/control/turret/algorithms/turret_controller_interface.hpp"

namespace aruwsrc::mock
{
class TurretControllerInterfaceMock
    : public aruwsrc::control::turret::algorithms::TurretControllerInterface
{
public:
    /**
     * @param[in] TurretMotor A `TurretMotor` object accessible for children objects to use.
     */
    TurretControllerInterfaceMock(aruwsrc::control::turret::TurretMotor &turretMotor);
    virtual ~TurretControllerInterfaceMock();

    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, runController, (const uint32_t, const WrappedFloat), (override));
    MOCK_METHOD(void, setSetpoint, (WrappedFloat), (override));
    MOCK_METHOD(WrappedFloat, getSetpoint, (), (const override));
    MOCK_METHOD(WrappedFloat, getMeasurement, (), (const override));
    MOCK_METHOD(bool, isOnline, (), (const override));
    MOCK_METHOD(
        WrappedFloat,
        convertControllerAngleToChassisFrame,
        (WrappedFloat),
        (const override));
    MOCK_METHOD(
        WrappedFloat,
        convertChassisAngleToControllerFrame,
        (WrappedFloat),
        (const override));

protected:
    aruwsrc::control::turret::TurretMotor *turretMotor;
};

}  // namespace aruwsrc::mock

#endif  // TURRET_CONTROLLER_INTERFACE_MOCK_HPP_
