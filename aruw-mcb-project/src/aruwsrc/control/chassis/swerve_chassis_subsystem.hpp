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

#ifndef SWERVE_CHASSIS_SUBSYSTEM_HPP_
#define SWERVE_CHASSIS_SUBSYSTEM_HPP_

#include <array>

#include "tap/algorithms/extended_kalman.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/gpio/analog.hpp"
#include "tap/communication/sensors/current/analog_current_sensor.hpp"
#include "tap/communication/serial/remote.hpp"
#include "tap/control/chassis/chassis_subsystem_interface.hpp"
#include "tap/control/chassis/power_limiter.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/m3508_constants.hpp"
#include "tap/util_macros.hpp"

#include "aruwsrc/communication/sensors/current/acs712_current_sensor_config.hpp"
#include "aruwsrc/control/chassis/holonomic_chassis_subsystem.hpp"
#include "aruwsrc/util_macros.hpp"
#include "constants/chassis_constants.hpp"
#include "modm/math/filter/pid.hpp"
#include "modm/math/matrix.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "aruwsrc/mock/swerve_module_mock.hpp"
using Module = testing::NiceMock<aruwsrc::mock::SwerveModuleMock>;
#else
#include "aruwsrc/control/chassis/swerve_module.hpp"
using Module = aruwsrc::chassis::SwerveModule;
#endif

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc
{
namespace chassis
{
class SwerveChassisSubsystem : public chassis::HolonomicChassisSubsystem
{
public:
    SwerveChassisSubsystem(
        tap::Drivers* drivers,
        tap::communication::sensors::current::CurrentSensorInterface* currentSensor,
        Module* moduleLeftFront,
        Module* moduleRightFront,
        Module* moduleLeftBack,
        Module* moduleRightBack,
        const float forwardMatrixArray[24]);

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

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    const unsigned int NUM_MODULES{4};
    std::array<Module*, 4> modules;

private:
#else
private:
    const unsigned int NUM_MODULES{4};
    std::array<Module*, 4> modules;
#endif

    /**
     * Stores the desired wheel rpm of each of the modules in a matrix, indexed by ModuleIndex
     */
    modm::Matrix<float, 4, 1> desiredModuleSpeeds;

    const modm::Matrix<float, 3, 8> forwardMatrix;

    /**
     * Given the desired x(m/s), y(m/s), and r(rad/s), updates each module with it
     *   for the delegated kinematics calculation, as well as limits the maximum wheel speed
     */
    void swerveDriveCalculate(float x, float y, float r, float maxWheelSpeed);

};  // class SwerveChassisSubsystem

}  // namespace chassis

}  // namespace aruwsrc

#endif  // SWERVE_CHASSIS_SUBSYSTEM_HPP_
