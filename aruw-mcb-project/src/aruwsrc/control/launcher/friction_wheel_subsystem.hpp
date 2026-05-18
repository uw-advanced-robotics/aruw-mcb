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

namespace aruwsrc::communication::can
{
class TurretMCBCanComm;
}

using namespace tap::algorithms;

namespace aruwsrc::control::launcher
{
/**
 * A subsystem which regulates the speed of an n-wheel shooter system using velocity PID
 * controllers. Allows the user to specify the desired launch speed of the shooter.
 */
template <std::size_t NUM_WHEELS>
class FrictionWheelSubsystem : public FrictionWheelInterface
{
    friend class FrictionWheelTestCommand;
    using Motor = tap::motor::MotorInterface;

public:
    /**
     * @brief Construct a new Friction Wheel Subsystem. Designed to be used with any number of
     * wheels. The constructor takes in an array of Motor pointers and an array of FlywheelConfigs
     * corresponding to each wheel.
     *
     * @param drivers The global drivers object
     * @param wheels A array of `Motor` pointers corresponding to each wheel in the friction
     * wheel system. The order of the motors should correspond to the order of the `FlywheelConfigs`
     * in the next parameter.
     * @param wheelConfigs A array of `FlywheelConfig`s corresponding to each wheel in the friction
     * wheel system.
     * @param speedCorrectionPidConfig A `SmoothPidConfig` for the speed correction PID feedback
     * from the launcher. This is to help fight against overspeeding caused by the friction wheels
     * heating up. If `nullptr`, the default constructor will be used for the `SmoothPidConfig`,
     * which has all gains set to 0.
     */
    template <std::size_t LUT_SIZE>
    FrictionWheelSubsystem(
        tap::Drivers *drivers,
        std::array<Motor *, NUM_WHEELS> wheels,
        std::array<FlywheelConfig, NUM_WHEELS> wheelConfigs,
        const modm::Pair<float, float> (&launchSpeedToFrictionWheelRpmLUT)[LUT_SIZE],
        const tap::algorithms::SmoothPidConfig speedCorrectionPidConfig = {})
        : FrictionWheelInterface(drivers),
          drivers(drivers),
          launchSpeedLinearInterpolator(launchSpeedToFrictionWheelRpmLUT, LUT_SIZE),
          flywheelConfigs(wheelConfigs),
          velocityPids(
              createVelocityPidArray(wheelConfigs, std::make_index_sequence<NUM_WHEELS>{})),
          speedCorrectionPid(speedCorrectionPidConfig),
          individualVelocityRamping(
              createWheelRampingArray(std::make_index_sequence<NUM_WHEELS>{})),
          wheels(wheels),
          frictionTestCommand(this)
    {
        this->setTestCommand(&frictionTestCommand);
    }

    // constructor for using a single wheelconfig for all wheels
    template <std::size_t LUT_SIZE>
    FrictionWheelSubsystem(
        tap::Drivers *drivers,
        std::array<Motor *, NUM_WHEELS> wheels,
        FlywheelConfig wheelConfig,
        const modm::Pair<float, float> (&launchSpeedToFrictionWheelRpmLUT)[LUT_SIZE],
        const tap::algorithms::SmoothPidConfig speedCorrectionPidConfig = {})
        : FrictionWheelSubsystem(
              drivers,
              wheels,
              createWheelConfigArray(wheelConfig, std::make_index_sequence<NUM_WHEELS>{}),
              launchSpeedToFrictionWheelRpmLUT,
              speedCorrectionPidConfig)
    {
    }

    void initialize() override
    {
        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            wheels[i]->initialize();
        }
    }

    /**
     * Set the projectile launch speed - at what speed the pellets
     * will come out of the physical friction wheel launcher.
     * Speed is limited to a range of values defined by constants of
     * this subsystem.
     *
     * @param[in] speed The launch speed in m/s.
     * @param[in] directRpm Whether to directly set rpm or set bullet speed.
     */
    void setDesiredLaunchSpeed(float speed, bool directRpm = false) override
    {
        desiredLaunchSpeed = limitVal(speed, 0.0f, MAX_DESIRED_LAUNCH_SPEED);
        if (directRpm)
        {
            desiredRpmRamp.setTarget(speed);
        }
        else
        {
            desiredRpmRamp.setTarget(launchSpeedToFrictionWheelRpm(speed));
        }
    }

    /**
     *  also need to changeWheelVelocityState to index, true for this to be used
     */
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
            sum += abs(wheels[i]->getEncoder()->getVelocity() * 60.0f / M_TWOPI);
        }
        return sum / NUM_WHEELS;
    }

    /**
     * @param[in] index Integer representing desired flywheel, starting at 0.
     * @return The measured friction wheel speed of the nth flywheel in launcher in RPM.
     */
    float getCurrentIndividualFrictionWheelSpeed(int index) const override
    {
        return wheels[index]->getEncoder()->getVelocity() * 60.0f / M_TWOPI;
    }

    float avgVelocity = 0;
    /**
     * Updates flywheel RPM ramp by elapsed time and sends motor output.
     */
    void refresh() override
    {
        desiredRpmRamp.update(FRICTION_WHEEL_RAMP_SPEED * (2.0));
        avgVelocity = getCurrentAverageFrictionWheelSpeed();
        if (drivers->refSerial.getRefSerialReceivingData() &&
            prevShotTime !=
                drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp)
        {
            prevShotTime =
                drivers->refSerial.getRobotData().turret.lastReceivedLaunchingInfoTimestamp;
            speedCorrectionPid.runControllerDerivateError(
                drivers->refSerial.getRobotData().turret.bulletSpeed - LAUNCHER_SPEED,
                0.002f);
        }
        speedCorrection = speedCorrectionPid.getOutput();

        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            if (isWheelVelocityOverridden[i])
            {
                individualVelocityRamping[i].setTarget(individualWheelVelocities[i]);
                individualVelocityRamping[i].update(FRICTION_WHEEL_RAMP_SPEED * (2.0));
                velocityPids[i].runControllerDerivateError(
                    individualVelocityRamping[i].getValue() -
                        getCurrentIndividualFrictionWheelSpeed(i),
                    0.002f);
            }
            else
            {
                velocityPids[i].runControllerDerivateError(
                    desiredRpmRamp.getValue() - getCurrentIndividualFrictionWheelSpeed(i) -
                        speedCorrection,
                    0.002f);
            }
            wheels[i]->setDesiredOutput(static_cast<int32_t>(velocityPids[i].getOutput()));
        }
    }

    void refreshSafeDisconnect() override
    {
        for (uint8_t i = 0; i < NUM_WHEELS; i++)
        {
            wheels[i]->setDesiredOutput(0);
        }
    }

    const char *getName() const override { return "Friction wheels"; }

    const tap::algorithms::Ramp &getDesiredRpmRamp() const { return desiredRpmRamp; }

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

    tap::algorithms::SmoothPid speedCorrectionPid;

    std::array<tap::algorithms::Ramp, NUM_WHEELS> individualVelocityRamping;

    std::array<Motor *, NUM_WHEELS> wheels;

    FrictionWheelTestCommand frictionTestCommand;

    float desiredLaunchSpeed{0.0f};

    float speedCorrection{0.0f};

    float currentRPM{0.0f};

    std::array<bool, NUM_WHEELS> isWheelVelocityOverridden{false};
    std::array<float, NUM_WHEELS> individualWheelVelocities{
        0.0f};  // is zero if wheel is using shared desiredLaunchSpeed

    tap::algorithms::Ramp desiredRpmRamp{0};

    float prevShotTime{0.0f};

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

    template <size_t... Is>
    static std::array<tap::algorithms::SmoothPid, NUM_WHEELS> createVelocityPidArray(
        std::array<FlywheelConfig, NUM_WHEELS> wheelConfigs,
        std::index_sequence<Is...>)
    {
        return {{((void)Is, tap::algorithms::SmoothPid(wheelConfigs[Is].velocityPidConfig))...}};
    }

    template <size_t... Is>
    static std::array<FlywheelConfig, NUM_WHEELS> createWheelConfigArray(
        FlywheelConfig wheelConfig,
        std::index_sequence<Is...>)
    {
        return {{((void)Is, wheelConfig)...}};
    }

    template <size_t... Is>
    static std::array<tap::algorithms::Ramp, NUM_WHEELS> createWheelRampingArray(
        std::index_sequence<Is...>)
    {
        return {{((void)Is, tap::algorithms::Ramp(0))...}};
    }
};

}  // namespace aruwsrc::control::launcher

#endif  // FRICTION_WHEEL_SUBSYSTEM_HPP_
