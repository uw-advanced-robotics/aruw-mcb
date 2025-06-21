// /*
//  * Copyright (c) 2024-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of aruw-mcb.
//  *
//  * aruw-mcb is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * aruw-mcb is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
//  */
// #ifndef BALSTD_CHASSIS_OBSERVER_HPP_
// #define BALSTD_CHASSIS_OBSERVER_HPP_

// #include "aruwsrc/robot/balstd/chassis/balstd_leg.hpp"

// namespace aruwsrc::balstd::chassis
// {

// class BalstdChassisObserver : public tap::control::chassis::ChassisSubsystemInterface
// {
// public:
//     BalstdChassisObserver(BalstdLeg& leftLeg, BalstdLeg& rightLeg);

//     void initialize() override;

//     void refresh() override;

//     void updateState();

//     inline const BalstdChassisState& getChassisState() const { return currState; }

// private:
//     BalstdLeg& leftLeg;
//     BalstdLeg& rightLeg;

// };  // class BalstdChassisObserver

// }  // namespace aruwsrc::balstd::chassis
// #endif  // BALSTD_CHASSIS_OBSERVER_HPP_
