/*
 * Copyright (c) 2020-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef TURRET_SUBSYSTEM_HPP_
#define TURRET_SUBSYSTEM_HPP_

#include "tap/algorithms/contiguous_float.hpp"
#include "tap/algorithms/linear_interpolation_predictor.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/control/turret_subsystem_interface.hpp"
#include "tap/motor/dji_motor.hpp"

#include "turret_motor_config.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/turret_motor_mock.hpp"
#else
#include "turret_motor.hpp"
#endif

#include "tap/util_macros.hpp"

#include "aruwsrc/util_macros.hpp"
#include "modm/math/filter/pid.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::control::turret::algorithms
{
class TurretPitchControllerInterface;
class TurretYawControllerInterface;
}  // namespace aruwsrc::control::turret::algorithms

namespace aruwsrc::control::turret
{
/**
 * Stores software necessary for interacting with two gimbals that control the pitch and
 * yaw of a turret. Provides a convenient API for other commands to interact with a turret.
 *
 * All angles computed using a right hand coordinate system. In other words, yaw is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from above. Pitch is a value from
 * 0-M_TWOPI rotated counterclockwise when looking at the turret from the right side of the turret.
 */
class TurretSubsystem : public tap::control::Subsystem
{
public:
    /**
     * Constructs a TurretSubsystem.
     *
     * @param[in] drivers Pointer to a drivers singleton object.
     * @param[in] pitchMotor Pointer to pitch motor that this `TurretSubsystem` will own.
     * @param[in] yawMotor Pointer to yaw motor that this `TurretSubsystem` will own.
     * @param[in] limitMotorAngles `true` if the yaw should be limited between `YAW_MIN_ANGLE` and
     *      `YAW_MAX_ANGLE` and `false` if the yaw should not be limited (if you have a slip
     *      ring).
     * @param[in] chassisFrontBackIdentical `true` if the front and back of the chassis are
     *      interchangable, indicating that `YAW_START_ANGLE` is identical to `YAW_START_ANGLE +
     *      180` in terms of relation to the chassis.
     */
    explicit TurretSubsystem(
        aruwsrc::Drivers* drivers,
        tap::motor::MotorInterface* pitchMotor,
        tap::motor::MotorInterface* yawMotor,
        const TurretMotorConfig& pitchMotorConfig,
        const TurretMotorConfig& yawMotorConfig);

    void initialize() override;

    void refresh() override;

    const char* getName() override { return "Turret"; }

    void onHardwareTestStart() override;

    mockable inline bool isOnline() const { return pitchMotor.isOnline() && yawMotor.isOnline(); }

#ifdef ENV_UNIT_TESTS
    testing::NiceMock<mock::TurretMotorMock> pitchMotor;
    testing::NiceMock<mock::TurretMotorMock> yawMotor;
#else
    /// Associated with and contains logic for controlling the turret's pitch motor
    TurretMotor pitchMotor;
    /// Associated with and contains logic for controlling the turret's yaw motor
    TurretMotor yawMotor;
#endif

protected:
    Drivers* drivers;
};  // class TurretSubsystem

}  // namespace aruwsrc::control::turret

#endif  // TURRET_SUBSYSTEM_HPP_
