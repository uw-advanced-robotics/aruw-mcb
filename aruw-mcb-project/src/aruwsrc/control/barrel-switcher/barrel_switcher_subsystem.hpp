/*
 * Copyright (c) 2023 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

// Inspired by TAMU barrel switcher

#ifndef BARREL_SWITCHER_SUBSYSTEM_HPP_
#define BARREL_SWITCHER_SUBSYSTEM_HPP_

#include "tap/algorithms/smooth_pid.hpp"
#include "tap/architecture/clock.hpp"
#include "tap/communication/serial/ref_serial.hpp"
#include "tap/control/subsystem.hpp"
#include "tap/drivers.hpp"
#include "tap/motor/dji_motor.hpp"

namespace aruwsrc::control::barrel_switcher
{
// To the Left is negative encoder counts
// To the Right is positive encoder counts
enum class BarrelSide
{
    LEFT = 0,
    RIGHT,
    CALIBRATING,
};

struct BarrelSwitcherMotorConfig
{
    tap::motor::MotorId motorId;
    tap::can::CanBus canBus;
    bool isInverted;
};

struct BarrelSwitcherConfig
{
    float hardStopOffsetMM;
    float barrelSwapDistanceMM;
    float barrelsAlignedToleranceMM;
    float leadScrewTicksPerMM;
    int16_t leadScrewCurrentSpikeTorque;
    int16_t leadScrewCaliOutput;
    std::array<tap::communication::serial::RefSerialData::Rx::MechanismID, 2> barrelArray;
};

class BarrelSwitcherSubsystem : public tap::control::Subsystem
{
public:
    BarrelSwitcherSubsystem(
        tap::Drivers& drivers,
        const BarrelSwitcherMotorConfig& motorConfig,
        const BarrelSwitcherConfig& config,
        const tap::algorithms::SmoothPidConfig& pidConfig);

    void initialize() override;

    void refresh() override;

    const char* getName() override { return "Barrel Manager Subsystem"; }

    inline bool isOnline() const { return swapMotor.isMotorOnline(); }

    // Runs into hard stops on both sides of lead screw to find their position
    inline void requestCalibration()
    {
        calibrationRequested = true;
        currentBarrelSide = BarrelSide::CALIBRATING;
    }

    // Finds which barrel is equipped
    BarrelSide getSide() const;

    // Will toggle which barrel is equipped
    void toggleSide();

    // Returns true when barrel is aligned with the flywheels (with a tolerance)
    bool isBarrelAligned() const;

    tap::communication::serial::RefSerialData::Rx::MechanismID getCurrentBarrel() const
    {
        return currentBarrel;
    }

private:
    bool calibrationRequested{false};

    tap::Drivers* drivers;

    tap::arch::MilliTimeout currentSpikeTimer{};

    BarrelSide currentBarrelSide{BarrelSide::LEFT};

    tap::motor::DjiMotor swapMotor;

    float leftSideCalibrationPosition{};

    tap::algorithms::SmoothPid positionPid;

    // Constants to be set by at the construction of a new barrel manager instance
    const BarrelSwitcherConfig config;

    tap::communication::serial::RefSerialData::Rx::MechanismID currentBarrel;

    uint32_t prevTime{};

    void performCalibration();

    float getLeftSidePosition() const { return config.hardStopOffsetMM; }

    float getRightSidePosition() const
    {
        return getLeftSidePosition() + config.barrelSwapDistanceMM;
    }

    float getDesiredBarrelPosition() const;

    // Returns raw position of the motor (not calibrated).
    float getRawPosition() const;

    // Returns encoder value of motor
    float getCalibratedMotorPosition() const;
};

}  // namespace aruwsrc::control::barrel_switcher

#endif
