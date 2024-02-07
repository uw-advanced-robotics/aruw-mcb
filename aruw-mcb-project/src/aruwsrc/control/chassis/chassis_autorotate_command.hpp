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

#ifndef CHASSIS_AUTOROTATE_COMMAND_HPP_
#define CHASSIS_AUTOROTATE_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"

#include "aruwsrc/control/turret/turret_motor.hpp"
#include "aruwsrc/robot/control_operator_interface.hpp"

namespace aruwsrc::chassis
{
class ChassisSubsystem;

/**
 * A command that continuously attempts to rotate the chasis so that the turret is
 * aligned with the center of the chassis.
 */
class ChassisAutorotateCommand : public tap::control::Command
{
public:
    /** When the turret yaw setpoint and measured value is < M_PI - this value, autorotation will be
     * paused until the difference is within this value again. */
    static constexpr float TURRET_YAW_SETPOINT_MEAS_DIFF_TO_APPLY_AUTOROTATION =
        modm::toRadian(1.0f);

    /** The symmetry of the chassis. */
    enum class ChassisSymmetry : uint8_t
    {
        /** No symmetry, only one "front". */
        SYMMETRICAL_NONE,
        /** Front and back symmetrical. */
        SYMMETRICAL_180,
        /** Front, back, left, and right are symmetrical. */
        SYMMETRICAL_90,
    };

    /**
     * @param[in] drivers Pointer to global drivers object.
     * @param[in] chassis Chassis to control.
     * @param[in] turret Turret subsytem, used to determine which point the chassis should be
     * autorotating around.
     * @param[in] chassisSymmetry The symmetry of the chassis.
     */
    ChassisAutorotateCommand(
        tap::Drivers* drivers,
        aruwsrc::control::ControlOperatorInterface* operatorInterface,
        ChassisSubsystem* chassis,
        const aruwsrc::control::turret::TurretMotor* yawMotor,
        ChassisSymmetry chassisSymmetry = ChassisSymmetry::SYMMETRICAL_NONE);

    void initialize() override;

    /**
     * Uses a PD controller to calculate the desired chassis rotation RPM based on the
     * difference between the turret angle and the center of the chassis, then
     * applies the desired rotation along with user desired <x, y> components to the
     * chassis subsystem's `setDesiredRpm` function.
     */
    void execute() override;

    void end(bool) override;

    bool isFinished() const override;

    const char* getName() const override { return "chassis autorotate"; }

protected:
    tap::Drivers* drivers;
    aruwsrc::control::ControlOperatorInterface* operatorInterface;
    ChassisSubsystem* chassis;
    const aruwsrc::control::turret::TurretMotor* yawMotor;

    /** Autorotation setpoint, smoothed using a low pass filter. */
    float desiredRotationAverage = 0;

    /**
     * The chassis's symmetry. This only matters if your turret can spin TWOPI radians. If the
     * symmetry is not SYMMETRY_NONE, the chassis will attempt to recenter itself around whichever
     * "center" the turret is closest to, where a "center" is defined by either the front of the
     * chassis or one of the chassis's points that is symmetrical to the front.
     */
    ChassisSymmetry chassisSymmetry;

    /**
     * `true` if the chassis is currently actually autorotating, `false` otherwise
     * (in which case on rotation may happen). Autorotation may not happen if the
     * user requests a user input that moves the turret from the front of the chassis
     * to the back. If the chassis front and back are identical, then there is no
     * reason to autorotate until the turret is done turning around.
     */
    bool chassisAutorotating;

    /**
     * Computes the setpoint to autorotate the chassis towards
     *
     * @param turretAngleFromCenter the current angle of the turret relative to the chassis
     * @param maxAngleFromCenter the maximum angle difference to either side before the autorotation
     * setpoint swaps
     * @return how much to rotate the chassis to get it aligned with the turret
     */
    virtual float computeAngleFromCenterForAutorotation(
        float turretAngleFromCenter,
        float maxAngleFromCenter);

    void updateAutorotateState();
};  // class ChassisAutorotateCommand

}  // namespace aruwsrc::chassis

#endif  // CHASSIS_AUTOROTATE_COMMAND_HPP_
