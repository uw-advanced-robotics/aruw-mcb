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

#ifndef TEST_AUTO_NAV_COMMAND_HPP_
#define TEST_AUTO_NAV_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"
#include <aruwsrc/algorithms/odometry/chassis_cf_odometry.hpp>

namespace aruwsrc::control::chassis
{

    class TestAutoNavCommand : public tap::control::Command
    {
        public:
            TestAutoNavCommand(
                const tap::Drivers& drivers,
                aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
                std::vector<Transform> transforms,
                tap::algorithms::odometry::Odometry2DInterface* odometrySubsystem
            );
            void initialize() override;
            void execute() override;
            void end(bool) override;
            bool isFinished() const override;
            const char* getName() const override { return "test autonav command";}
        
        private:
            const tap::Drivers& drivers;
            aruwsrc::control::chassis::ChassisAutoNavController& autoNavController;
            Position point;
            std::vector<Transform> transforms;
            tap::algorithms::odometry::Odometry2DInterface* odometrySubsystem;
            aruwsrc::algorithms::AutoNavPath path;
    };
}

#endif // PUSH_AUTO_NAV_POINT_COMMAND_HPP_

