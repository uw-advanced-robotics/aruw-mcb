/*
 * Copyright (c) 2026-2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef PREDICTION_INDICATOR_HPP_
#define PREDICTION_INDICATOR_HPP_

#include "tap/algorithms/ballistics.hpp"
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial.hpp"

#include "../projection_utils.hpp"
#include "aruwsrc/algorithms/odometry/transforms/transformer_interface.hpp"
#include "aruwsrc/communication/serial/vision_coprocessor.hpp"
#include "aruwsrc/control/launcher/launch_speed_predictor_interface.hpp"
#include "aruwsrc/control/turret/robot_turret_subsystem.hpp"
#include "modm/processing/resumable.hpp"

#include "hud_indicator.hpp"

namespace aruwsrc::control::client_display::indicators
{
using namespace aruwsrc::algorithms::odometry::transforms;
using namespace tap::communication::serial;
/**
 * Draws a box showing where a shot fired right now would hit on a robot, based on data from vision.
 */
class PredictionIndicator : public HudIndicator, protected modm::Resumable<1>
{
public:
    PredictionIndicator(
        aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor,
        tap::communication::serial::RefSerialTransmitter &refSerialTransmitter,
        const tap::algorithms::odometry::Odometry2DInterface &odometryInterface,
        const control::turret::RobotTurretSubsystem &turretSubsystem,
        const control::launcher::LaunchSpeedPredictorInterface &frictionWheels,
        const Transform &worldToCameraTransform,
        const float defaultLaunchSpeed);

    void initialize() override final;

    modm::ResumableResult<void> update() override final;

private:
    aruwsrc::communication::serial::VisionCoprocessor &visionCoprocessor;
    tap::communication::serial::RefSerialTransmitter &refSerialTransmitter;
    const tap::algorithms::odometry::Odometry2DInterface &odometryInterface;
    const control::turret::RobotTurretSubsystem &turretSubsystem;
    const control::launcher::LaunchSpeedPredictorInterface &frictionWheels;
    const Transform &worldToCameraTransform;
    const float defaultLaunchSpeed;

    modm::Vector3f predictedShotLandingPosition;

    Tx::Graphic1Message hitPredictionGraphic;

    static constexpr uint16_t INDICATOR_LINE_THICKNESS = 3;

    RefSerialData::Tx::GraphicColor INDICATOR_COLOR = RefSerialData::Tx::GraphicColor::CYAN;

    float aex, aey, aez, abx, aby, arx, ary, arz, arp, arw, arw2, adx, ady, adz, at;
};

}  // namespace aruwsrc::control::client_display::indicators

#endif  // PREDICTION_INDICATOR_HPP_
