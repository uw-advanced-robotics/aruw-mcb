/*
 * Copyright (c) 2021-2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef BOOLEAN_HUD_INDICATORS_HPP_
#define BOOLEAN_HUD_INDICATORS_HPP_

#include "tap/architecture/periodic_timer.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"

#include "aruwsrc/communication/serial/sentry_response_handler.hpp"
#include "aruwsrc/control/agitator/velocity_agitator_subsystem.hpp"
#include "aruwsrc/control/hopper-cover/turret_mcb_hopper_cover_subsystem.hpp"
#include "aruwsrc/control/imu/imu_calibrate_command.hpp"
#include "aruwsrc/control/launcher/friction_wheel_subsystem.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display
{
/**
 * A list of indicators that represent different boolean (true/false) states.
 */
class BooleanHudIndicators : public HudIndicator, protected modm::Resumable<2>
{
public:
    /**
     * Construct a BooleanHudIndicators object.
     *
     * @param[in] commandScheduler  CommandScheduler instance.
     * @param[in] hopperSubsystem Hopper used when checking if the hopper is open/closed. A pointer
     * that may be nullptr if no hopper exists.
     * @param[in] frictionWheelSubsystem Friction wheels used when checking if the friction wheels
     * are on or off.
     * @param[in] agitatorSubsystem Agitator used when checking if the agitator is jammed.
     * @param[in] imuCalibrateCommand IMU calibrate command used when checking if the IMU is being
     * calibrated.
     * @param[in] refSerial Ref system data to get ammo count.
     */
    BooleanHudIndicators(
        tap::control::CommandScheduler &commandScheduler,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem,
        const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem,
        tap::control::setpoint::SetpointSubsystem &agitatorSubsystem,
        const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand,
        const tap::communication::serial::RefSerial *refSerial);

    modm::ResumableResult<bool> sendInitialGraphics() override final;

    modm::ResumableResult<bool> update() override final;

    void initialize() override final;

private:
    /** X pixel coordinate that the boolean hud indicator circles will be centered around. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_CENTER_X = 280;
    /** Starting y value where boolean hud indicator circles will start. The top most circle in the
     * list will be centered around this point. Subsequent circles will be below this y pixel value.
     */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_START_Y = 760;
    /** Distance between the center of the the boolean indicators, in the y direction. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LIST_DIST_BTWN_BULLETS = 50;
    /** The line width of the indicator circles. Should be approximately twice
     * BOOLEAN_HUD_INDICATOR_RADIUS if you want the circle to be filled completely. There is no way
     * to fill in circle graphics, so the next best thing is to make the line width very large. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_WIDTH = 17;
    /** The radius of the boolean indicator circles. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_RADIUS = 9;
    /** The color that the HUD indicator will be outlined in. */
    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_OUTLINE_COLOR = Tx::GraphicColor::BLACK;
    /** The width of the boolean HUD indicator's outline circle. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_WIDTH = 5;
    /** The radius of the boolean HUD indicator outline circle. */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_OUTLINE_RADIUS = 20;
    /** The color of the textual label associated with the boolean HUD indicators. */
    static constexpr Tx::GraphicColor BOOLEAN_HUD_INDICATOR_LABEL_COLOR = Tx::GraphicColor::ORANGE;
    /** The character size of the textual label associated with the boolean HUD indicators */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_CHAR_SIZE = 15;
    /** The character line width of the textual label associated with the boolean HUD indicators */
    static constexpr uint16_t BOOLEAN_HUD_INDICATOR_LABEL_CHAR_LINE_WIDTH = 3;

    /** Tuple that contains the name of the boolean indicator, the color when the indicator is
     * `true` or on, and the color when the indicator is `false` or off. */
    using BooleanHUDIndicatorTuple = std::tuple<const char *, Tx::GraphicColor, Tx::GraphicColor>;

    /**
     * Enum containing all boolean HUD indicators.
     */
    enum BooleanHUDIndicatorIndex
    {
        /** Indicates systems (such as the IMU) are calibrating. */
        SYSTEMS_CALIBRATING = 0,
        /** Indicates the agitator is online and not jammed. */
        AGITATOR_STATUS_HEALTHY,
        /** Indicates there is ammo. */
        AMMO_AVAILABLE,
        /** Should always be the last value, the number of enum values listed in this enum (as such,
           the first element in this enum should be 0 and subsequent ones should increment by 1
           each). */
        NUM_BOOLEAN_HUD_INDICATORS,
    };

    /**
     * List of `BooleanHUDIndicatorTuple`s of length `NUM_BOOLEAN_HUD_INDICATORS`. Each item
     * associated with an enum value above.
     */
    static constexpr BooleanHUDIndicatorTuple
        BOOLEAN_HUD_INDICATOR_LABELS_AND_COLORS[NUM_BOOLEAN_HUD_INDICATORS]{
            BooleanHUDIndicatorTuple(
                "SYS CALIB ",
                Tx::GraphicColor::PURPLISH_RED,  // Purple/Red when calibrating
                Tx::GraphicColor::GREEN),        // Green when not calibrating
            BooleanHUDIndicatorTuple(
                "AGI ",
                Tx::GraphicColor::GREEN,
                Tx::GraphicColor::PURPLISH_RED),
            BooleanHUDIndicatorTuple(
                "AMMO ",
                Tx::GraphicColor::GREEN,
                Tx::GraphicColor::PURPLISH_RED),
        };

    tap::control::CommandScheduler &commandScheduler;

    /**
     * Hopper subsystem that provides information about whether or not the cover is open or closed.
     */
    const aruwsrc::control::TurretMCBHopperSubsystem *hopperSubsystem;

    /**
     * Friction wheel subsystem that provides info about if they are on/off.
     */
    const aruwsrc::control::launcher::FrictionWheelSubsystem &frictionWheelSubsystem;

    /**
     * Agitator that provides info about if it is jammed or offline.
     *
     * Should be `const` but `isJammed` is not `const`.
     */
    tap::control::setpoint::SetpointSubsystem &agitatorSubsystem;

    /**
     * ImuCalbirateCommand that provides information about if the IMUs are being calibrated.
     */
    const aruwsrc::control::imu::ImuCalibrateCommand &imuCalibrateCommand;

    /**
     * Ref Serial provides referee system data to get whether or not there is ammo remaining.
     */
    const tap::communication::serial::RefSerial *refSerial;

    inline bool haveAmmo()
    {
        return (
            refSerial->getRobotData().turret.bulletsRemaining17 ||
            refSerial->getRobotData().turret.bulletsRemaining42);
    }

    /**
     * Graphic message that will represent a dot on the screen that will be present or not,
     * depending on whether or not the hopper is open or closed.
     */
    Tx::Graphic1Message booleanHudIndicatorGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    /** The objects that will do the actual drawing of the hopper open indicator. */
    tap::communication::referee::BooleanHUDIndicator
        booleanHudIndicatorDrawers[NUM_BOOLEAN_HUD_INDICATORS];

    /** Use this index when iterating through the booleanHudIndicatorDrawers in the update function.
     * Should be a local variable, but since it's in a protothread it can't be local. */
    int booleanHudIndicatorIndexUpdate = 0;
    /** @see booleanHudIndicatorIndexUpdate, similar variable but used in sendInitialGraphics
     * function. */
    int booleanHudIndicatorIndexSendInitialGraphics = 0;

    /**
     * Graphics associated with the the hud indicator graphics that do not change (labels and
     * circles around the indicators).
     */
    Tx::Graphic1Message booleanHudIndicatorStaticGraphics[NUM_BOOLEAN_HUD_INDICATORS];
    Tx::GraphicCharacterMessage booleanHudIndicatorStaticLabelGraphics[NUM_BOOLEAN_HUD_INDICATORS];

    bool outOfAmmo = false;
    // How often to toggle the out of ammo indicator
    static constexpr float OUT_OF_AMMO_TOGGLE_PERIOD_MS = 100.0f;
    tap::arch::PeriodicMilliTimer outOfAmmoTimer;
};
}  // namespace aruwsrc::control::client_display

#endif  //  BOOLEAN_HUD_INDICATORS_HPP_
