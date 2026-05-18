/*
 * Copyright (c) 2026 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "cv_on_target_governor.hpp"

namespace aruwsrc::control::governor
{
volatile bool cvOnTargetDebugCvOnline = false;
volatile bool cvOnTargetDebugCvRunning = false;
volatile bool cvOnTargetDebugGovernorEnabled = false;
volatile bool cvOnTargetDebugGovernorGating = false;
volatile bool cvOnTargetDebugOnTarget = false;
volatile bool cvOnTargetDebugGateSatisfied = false;
volatile bool cvOnTargetDebugIsReady = false;
volatile uint8_t cvOnTargetDebugLaunchInclination = 0;
volatile bool cvOnTargetDebugAimDataUpdated = false;
volatile bool cvOnTargetDebugTimingDataUpdated = false;
volatile bool cvOnTargetDebugBallisticsSolutionFound = false;
volatile bool cvOnTargetDebugPulseEstimationUsed = false;
volatile bool cvOnTargetDebugValidFlightTime = false;
volatile bool cvOnTargetDebugInShotWindow = false;
volatile uint32_t cvOnTargetDebugAimTimestamp = 0;
volatile uint32_t cvOnTargetDebugPulseOffset = 0;
volatile uint32_t cvOnTargetDebugPulseInterval = 0;
volatile uint32_t cvOnTargetDebugPulseDuration = 0;
volatile float cvOnTargetDebugTimeOfFlight = 0.0f;
volatile uint64_t cvOnTargetDebugNow = 0;
volatile uint64_t cvOnTargetDebugEffectiveFireTime = 0;
volatile uint64_t cvOnTargetDebugShotWindowStart = 0;
volatile uint64_t cvOnTargetDebugShotWindowEnd = 0;
volatile int64_t cvOnTargetDebugCountdownToShotWindowStart = 0;
volatile int64_t cvOnTargetDebugCountdownToShotWindowEnd = 0;
volatile int64_t cvOnTargetDebugOffsetInFiringWindow = 0;
volatile uint32_t cvOnTargetDebugMaxHitTimeError = 0;
}  // namespace aruwsrc::control::governor
