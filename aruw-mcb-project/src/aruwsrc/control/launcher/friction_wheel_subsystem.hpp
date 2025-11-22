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

#ifndef FRICTION_WHEEL_SUBSYSTEM_HPP_
#define FRICTION_WHEEL_SUBSYSTEM_HPP_

#include <utility>
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/algorithms/ramp.hpp"
#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "tap/util_macros.hpp"

#include "friction_wheel_interface.hpp"

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
#include "tap/mock/dji_motor_mock.hpp"
#else
// #include "tap/motor/dji_motor.hpp"
#include "tap/motor/motor_interface.hpp"
#endif

#include "aruwsrc/communication/can/turret_mcb_can_comm.hpp"
#include "modm/math/filter/pid.hpp"

#include "friction_wheel_test_command.hpp"
#include "launcher_constants.hpp"

namespace aruwsrc
{
class Drivers;
}

namespace aruwsrc::can
{
class TurretMCBCanComm;
}

using namespace tap::algorithms;

namespace aruwsrc::control::launcher
{
/**
 * A subsystem which regulates the speed of an n-wheel shooter system using velocity PID
 * controllers. Allows the user to specify the desired launch speed of the shooter.
 * Currently configured to work with hero and idk if it works for other robots.
 */
template <std::size_t NUM_WHEELS>
class FrictionWheelSubsystem : public FrictionWheelInterface
{
    friend class FrictionWheelTestCommand;
private: 
template <size_t... Is>
static std::array<tap::algorithms::SmoothPid, NUM_WHEELS> createVelocityPidArray(std::array<FlywheelConfig, NUM_WHEELS> wheelConfigs, std::index_sequence<Is...>) {
    // std::array<tap::algorithms::SmoothPid, NUM_WHEELS> pids;
    // for (uint8_t i = 0; i < NUM_WHEELS; i++)
    //     {
    //         pids[i] = tap::algorithms::SmoothPid(wheelConfigs[i].velocityPidConfig);
    //     }
    return {{ ((void)Is, tap::algorithms::SmoothPid(wheelConfigs[0].velocityPidConfig))... }};
}

public:
#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
    using Motor = testing::NiceMock<tap::mock::DjiMotorMock>;
#else
    using Motor = tap::motor::MotorInterface;
#endif
    /**
     * Creates a new friction wheel subsystem
     */
    FrictionWheelSubsystem(
        tap::Drivers *drivers,
        std::array<Motor *, NUM_WHEELS> wheels,
        std::array<FlywheelConfig, NUM_WHEELS> wheelConfigs,
        tap::can::CanBus,
        aruwsrc::can::TurretMCBCanComm *turretMCB)
        : FrictionWheelInterface(drivers),
          drivers(drivers),
          launchSpeedLinearInterpolator(
              LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT,
              MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT)),
          flywheelConfigs(wheelConfigs),
          velocityPids(createVelocityPidArray(wheelConfigs, std::make_index_sequence<NUM_WHEELS>{})),
          speedCorrectionPid(
              LAUNCHER_SPEED_CORRECTION_PID_KP,
              LAUNCHER_SPEED_CORRECTION_PID_KI,
              LAUNCHER_SPEED_CORRECTION_PID_KD,
              LAUNCHER_SPEED_CORRECTION_PID_MAX_ERROR_SUM,
              LAUNCHER_SPEED_CORRECTION_PID_MAX_OUTPUT),
          desiredRpmRamp(0),
          wheels(wheels),
          turretMCB(turretMCB),
          frictionTestCommand(this)
    {
        this->setTestCommand(&frictionTestCommand);
    }

    void initialize() override
    {
        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            wheels[i]->initialize();
        }
        prevTime = tap::arch::clock::getTimeMilliseconds();
    }

    /**
     * Set the projectile launch speed - at what speed the pellets
     * will come out of the physical friction wheel launcher.
     * Speed is limited to a range of values defined by constants of
     * this subsystem.
     *
     * @param[in] speed The launch speed in m/s.
     */
    void setDesiredLaunchSpeed(float speed) override
    {
        desiredLaunchSpeed = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
        desiredRpmRamp.setTarget(launchSpeedToFrictionWheelRpm(speed));
        if (turretMCB != nullptr)
        {
            turretMCB->setLaserStatus(!compareFloatClose(desiredLaunchSpeed, 0, 1E-5));
        }
    };

    // also need to changeWheelVelocityState to index, true for this to be used
    void setIndividualVelocity(int index, float velocity) override
    {
        individualWheelVelocities[index] = velocity;
    }

    void changeWheelVelocityState(int index, bool hasIndividualVelocity) override
    {
        isWheelVelocityOverridden[index] = hasIndividualVelocity;
    }

    float getDesiredLaunchSpeed() const override { return desiredLaunchSpeed; }

    float getDesiredFrictionWheelSpeed() const override
    {
        return launchSpeedToFrictionWheelRpm(desiredLaunchSpeed) - speedCorrection;
    }

    float getCurrentCorrectionValue() const override { return speedCorrection; }
    /**
     * @return The average measured friction wheel speed of the launcher in RPM.
     */
    float getCurrentAverageFrictionWheelSpeed() const override
    {
        float sum = 0;
        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            sum += wheels[i]->getEncoder()->getVelocity() * 60.0f / M_TWOPI;
        }
        return sum / NUM_WHEELS;
    }

    /**
     * @return The measured friction wheel speed of the nth flywheel in launcher in RPM.
     */
    float getCurrentIndividualFrictionWheelSpeed(int index) const override
    {
        return wheels[index]->getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }
    /**
     * Updates flywheel RPM ramp by elapsed time and sends motor output.
     */
    void refresh() override
    {
        uint32_t currTime = tap::arch::clock::getTimeMilliseconds();
        if (currTime == prevTime)
        {
            return;
        }
        desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (currTime - prevTime));
#if defined(ALL_STANDARDS)
        if (prevShotTime !=
            drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp)
        {
            prevShotTime =
                drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;
            speedCorrectionPid.update(
                drivers->refSerial.getRobotData().turret.bulletSpeed - LAUNCHER_SPEED);
        }
        speedCorrection = speedCorrectionPid.getValue();
#endif

        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            if (isWheelVelocityOverridden[i])
            {
                velocityPids[i].runControllerDerivateError(
                    individualWheelVelocities[i] - getCurrentIndividualFrictionWheelSpeed(i),
                    currTime - prevTime);
            }
            else
            {
                velocityPids[i].runControllerDerivateError(
                    desiredRpmRamp.getValue() - getCurrentIndividualFrictionWheelSpeed(i) -
                        speedCorrection,
                    currTime - prevTime);
            }
            wheels[i]->setDesiredOutput(static_cast<int32_t>(velocityPids[i].getOutput()));
        }
        prevTime = currTime;
    }

    void refreshSafeDisconnect() override
    {
        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            wheels[i]->setDesiredOutput(0);
        }
    }

    const char *getName() const override { return "Friction wheels"; }

protected:
    /// The maximum launch speed that the user can request. The launch speed is limited between [0,
    /// MAX_DESIRED_LAUNCH_SPEED].
    static constexpr float MAX_DESIRED_LAUNCH_SPEED =
        LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT
            [MODM_ARRAY_SIZE(LAUNCH_SPEED_TO_FRICTION_WHEEL_RPM_LUT) - 1]
                .first;

    /// The maximum measured launch speed if the max desired launch speed is requested. This is a
    /// large overestimate on purpose--it is useful for providing an upper bound on the possible
    /// measured launch speed in case the measured launch speed is garbage.
    static constexpr float MAX_MEASURED_LAUNCH_SPEED = MAX_DESIRED_LAUNCH_SPEED + 10.0f;

    tap::Drivers *drivers;

private:
    modm::interpolation::Linear<modm::Pair<float, float>> launchSpeedLinearInterpolator;

    std::array<FlywheelConfig, NUM_WHEELS> flywheelConfigs;
    std::array<tap::algorithms::SmoothPid, NUM_WHEELS> velocityPids;

    modm::Pid<float> speedCorrectionPid;

    float desiredLaunchSpeed;

    float speedCorrection = 0.0f;

    uint32_t prevTime = 0;

    bool isWheelVelocityOverridden[NUM_WHEELS] = {0};
    float individualWheelVelocities[NUM_WHEELS] = {
        0};  // is zero if wheel is using shared desiredLaunchSpeed

#if defined(PLATFORM_HOSTED) && defined(ENV_UNIT_TESTS)
public:
    tap::algorithms::Ramp desiredRpmRamp;

    std::array<Motor *, NUM_WHEELS> wheels;

private:
#else
    tap::algorithms::Ramp desiredRpmRamp;

    std::array<Motor *, NUM_WHEELS> wheels;
#endif

    aruwsrc::can::TurretMCBCanComm *turretMCB;

    float prevShotTime = 0.0f;

    FrictionWheelTestCommand frictionTestCommand;

    /**
     * @param[in] launchSpeed Some launch speed in m/s. The speed will be
     *      capped between [0, LAUNCH_SPEED_MAX] m/s. If the speed is <= LAUNCH_SPEED_MIN_CUTOFF
     *      m/s, the speed will be rounded down to 0.
     * @return A friction wheel RPM that the given `launchSpeed` maps to.
     */
    float launchSpeedToFrictionWheelRpm(float launchSpeed) const
    {
        return launchSpeedLinearInterpolator.interpolate(launchSpeed);
    };
};

}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_SUBSYSTEM_HPP_