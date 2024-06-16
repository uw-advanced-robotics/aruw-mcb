/*
 * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef HALF_SWERVE_CHASSIS_SUBSYSTEM_HPP_
#define HALF_SWERVE_CHASSIS_SUBSYSTEM_HPP_

#include <aruwsrc/communication/mcb-lite/motor/virtual_dji_motor.hpp>

#include "tap/communication/sensors/current/current_sensor_interface.hpp"
#include "tap/control/subsystem.hpp"

#include "holonomic_chassis_subsystem.hpp"
#include "swerve_module.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/swerve_module_mock.hpp"
using Module = testing::NiceMock<aruwsrc::mock::SwerveModuleMock>;
#else
#include "aruwsrc/control/chassis/swerve_module.hpp"
using Module = aruwsrc::chassis::SwerveModule;
#endif

namespace aruwsrc::chassis
{
class HalfSwerveChassisSubsystem : public HolonomicChassisSubsystem
{
public:
    HalfSwerveChassisSubsystem(
        tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        Module* moduleOne,
        Module* moduleTwo,
        aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel,
        aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel,
        const float wheelbaseRadius,
        const float forwardMatrixArray[12],
        can::capbank::CapacitorBank* capacitorBank = nullptr);

    void initialize() override;

    void setDesiredOutput(float x, float y, float r) override;

    void limitChassisPower() override;

    void refresh() override;

    inline int getNumChassisMotors() const override { return NUM_MODULES * 2; }

    bool allMotorsOnline() const override;

    void setZeroRPM() override;

    void refreshSafeDisconnect() override { setZeroRPM(); }

    Module* getModule(unsigned int i);

    inline float mpsToRpm(float mps) const override { return modules[0]->wheel.mpsToRpm(mps); }

    /**
     * Used to index into the modules array and desiredModuleSpeeds matrix.
     */
    enum ModuleIndex
    {
        LF = 0,
        RF = 1,
        LB = 2,
        RB = 3,
    };

    modm::Matrix<float, 3, 1> getActualVelocityChassisRelative() const override;

    modm::Matrix<float, 3, 1> getDesiredVelocityChassisRelative() const;

    const char* getName() const override { return "half swerve chassis subsystem"; }

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    const unsigned int NUM_MODULES{2};
    std::array<Module*, 2> modules;

private:
#else
private:
    const unsigned int NUM_MODULES{2};
    std::array<Module*, 2> modules;
#endif

    aruwsrc::virtualMCB::VirtualDjiMotor* parallelWheel;
    aruwsrc::virtualMCB::VirtualDjiMotor* perpendicularWheel;

    /**
     * Stores the desired wheel rpm of each of the modules in a matrix, indexed by ModuleIndex
     */
    modm::Matrix<float, 2, 1> desiredModuleSpeeds;

    const float wheelbaseRadius;

    const modm::Matrix<float, 3, 4> forwardMatrix;

    /**
     * Given the desired x(m/s), y(m/s), and r(rad/s), updates each module with it
     *   for the delegated kinematics calculation, as well as limits the maximum wheel speed
     */
    void swerveDriveCalculate(float x, float y, float r, float maxWheelRPM);

};  // class HalfSwerveChassisSubsystem

}  // namespace aruwsrc::chassis
#endif  // HALF_SWERVE_CHASSIS_SUBSYSTEM_HPP_
