/*
 * Copyright (c) 2022 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef LAUNCH_SPEED_PREDICTOR_INTERFACE_HPP_
#define LAUNCH_SPEED_PREDICTOR_INTERFACE_HPP_

namespace aruwsrc::control::launcher
{
/**
 * Interface for retreiving the predicted launch velocity of a launching mechanism.
 */
class LaunchSpeedPredictorInterface
{
public:
    virtual inline float getPredictedLaunchSpeed() const = 0;
};
}  // namespace aruwsrc::control::launcher

#endif  // LAUNCH_SPEED_PREDICTOR_INTERFACE_HPP_
