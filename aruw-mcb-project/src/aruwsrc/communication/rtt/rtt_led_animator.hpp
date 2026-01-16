/*
 * Copyright (c) 2025 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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
#ifndef RTT_LED_ANIMATOR_HPP_
#define RTT_LED_ANIMATOR_HPP_

#include <cstdint>

#include "tap/architecture/periodic_timer.hpp"

namespace tap
{
class Drivers;
}  // namespace tap

namespace aruwsrc::communication::rtt
{
class RttLedAnimator
{
public:
    RttLedAnimator();

    void update(
        tap::Drivers* drivers,
        bool activelySendingTelemetry,
        bool recentRttInput,
        uint32_t now);

private:
    // Timer for LED blinking
    tap::arch::PeriodicMilliTimer ledBlinkTimer;

    // Animation state for the A-H LED row
    tap::arch::PeriodicMilliTimer animationTimer;  // drives the moving 'bounce' animation
    uint8_t animationIndex;                        // current lit LED index 0..7
    bool animationDirectionUp;                     // true = moving A->H, false = H->A
    uint32_t animationStepMs;                      // ms between animation steps
    // Group flash state used when no recent message has been received
    bool groupFlashOn;
    // Unidirectional pause state used when no messages are being received
    bool unidirectionalPaused;
    uint32_t unidirectionalPauseDeadlineMillis;
};
}  // namespace aruwsrc::communication::rtt

#endif  // RTT_LED_ANIMATOR_HPP_
