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

#ifndef CLIENT_DISPLAY_COMMAND_HPP_
#define CLIENT_DISPLAY_COMMAND_HPP_

#include <array>
#include <tuple>

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_transmitter.hpp"
#include "tap/control/command.hpp"

#include "modm/math/geometry/polygon_2d.hpp"
#include "modm/math/utils/misc.hpp"
#include "modm/processing/protothread.hpp"

#include "boolean_hud_indicators.hpp"
#include "cap_bank_indicator.hpp"
#include "chassis_orientation_indicator.hpp"
#include "matrix_hud_indicators.hpp"
#include "reticle_indicator.hpp"
#include "vision_hud_indicators.hpp"


#include "hero_assist_indicator.hpp"

namespace tap::control
{
class Subsystem;
}

namespace tap
{
class Drivers;
}

namespace aruwsrc::control::client_display
{
class ClientDisplaySubsystem;

/**
 * A command that controls what is displayed on the RoboMaster client's interactive HUD.
 *
 * The following graphic features are supported:
 * - Static reticle.
 * - Simulated chassis to represent chassis location.
 * - A list of boolean indicators.
 * - A table of selection indicators (where the graphic can be in one of multiple states).
 * - Turret pitch/yaw angles.
 *
 * @note Only a single ClientDisplayCommand should be instantiated. If more than one is
 * instantiated, this will lead to undefined behavior.
 */
class ClientDisplayCommand : public tap::control::Command, ::modm::pt::Protothread
{
public:
    /**
     * Construct a ClientDisplayCommand.
     *
     * @param[in] drivers Global drivers instance.
     * @param[in] commandScheduler CommandScheduler instance.
     * @param[in] clientDisplay The client display subsystem associated with the command.
     * @param[in] hopperSubsystem Hopper used when checking if the hopper is open/closed. A pointer
     * that may be nullptr if no hopper exists.
     * @param[in] frictionWheelSubsystem Friction wheels used when checking if the friction wheels
     * are on or off.
     * @param[in] agitatorSubsystem Agitator used when checking if the agitator is jammed.
     * @param[in] robotTurretSubsystem Turret used when updating chassis orientation relative
     * to the turret and to print turret angles (if turret chassis relative angles are being
     * printed).
     * @param[in] avoidanceCommand Commands that indicate if the robot is in an avoidance drive
     * mode, such as beyblade or wiggle.
     * @param[in] imuCalibrateCommand IMU calibrate command used when checking if the IMU is being
     * calibrated.
     * @param[in] multiShotHandler Shot handler, used to determine which shooting mode the agitator
     * is in. May be nullptr, if so multi shot mode defaults to single shot (as displayed on the
     * HUD).
     * @param[in] cvOnTargetManager @see MatrixHudIndicators
     * @param[in] chassisBeybladeCmd May be nullptr. If nullptr the chassis beyblade command will
     * never be selected as the current chassis command.
     * @param[in] chassisAutorotateCmd May be nullptr. If nullptr the chassis autorotate command
     * will never be selected as the current chassis command.
     * @param[in] chassisImuDriveCommand May be nullptr. If nullptr the chassis IMU drive command
     * will never be selected as the current chassis command.
     * @param[in] capBank A pointer to the capacitor bank for the robot.
     */
    ClientDisplayCommand(
        tap::Drivers &drivers,
        tap::control::CommandScheduler &commandScheduler,
        aruwsrc::serial::VisionCoprocessor &visionCoprocessor,
        ClientDisplaySubsystem &clientDisplay,
        const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem,
        const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
        tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
        const aruwsrc::control::turret::RobotTurretSubsystem &robotTurretSubsystem,
        const std::vector<tap::control::Command *> avoidanceCommands,
        const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
        const aruwsrc::control::agitator::MultiShotCvCommandMapping *multiShotHandler,
        const aruwsrc::control::governor::CvOnTargetGovernor *cvOnTargetManager,
        const aruwsrc::chassis::BeybladeCommand *chassisBeybladeCmd,
        const aruwsrc::chassis::ChassisAutorotateCommand *chassisAutorotateCmd,
        const aruwsrc::chassis::ChassisImuDriveCommand *chassisImuDriveCommand,
        const can::capbank::CapacitorBank *capBank = nullptr);

    const char *getName() const override { return "client display"; }

    void initialize() override;

    void execute() override;

    void end(bool) override {}

    bool isFinished() const override { return false; }

private:
    tap::Drivers &drivers;
    aruwsrc::serial::VisionCoprocessor &visionCoprocessor;
    tap::control::CommandScheduler &commandScheduler;
    tap::communication::serial::RefSerialTransmitter refSerialTransmitter;
    BooleanHudIndicators booleanHudIndicators;
    CapBankIndicator capBankIndicator;
    ChassisOrientationIndicator chassisOrientationIndicator;
    MatrixHudIndicators positionHudIndicators;
    ReticleIndicator reticleIndicator;
    VisionHudIndicators visionHudIndicators;
    HeroAssistIndicator heroAssistIndicator;

    bool restarting = true;

    bool run();
    void restartHud();
};
}  // namespace aruwsrc::control::client_display

#endif  // CLIENT_DISPLAY_COMMAND_HPP_
