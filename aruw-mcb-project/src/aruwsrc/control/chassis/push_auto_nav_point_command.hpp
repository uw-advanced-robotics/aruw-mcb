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

#ifndef PUSH_AUTO_NAV_POINT_COMMAND_HPP_
#define PUSH_AUTO_NAV_POINT_COMMAND_HPP_

#include "tap/control/command.hpp"
#include "tap/drivers.hpp"
#include "aruwsrc/control/chassis/chassis_auto_nav_controller.hpp"

namespace aruwsrc::control::chassis
{

    class PushAutoNavPointCommand : public tap::control::Command
    {
        public:
            PushAutoNavPointCommand(
                const tap::Drivers& drivers,
                aruwsrc::control::chassis::ChassisAutoNavController& autoNavController,
                Position point
            );
            void initialize() override;
            void execute() override;
            void end(bool) override;
            bool isFinished() const override;
            const char* getName() const override { return "push autonav point command";}
        
        private:
            const tap::Drivers& drivers;
            aruwsrc::control::chassis::ChassisAutoNavController& autoNavController;
            Position point;
    };
}

#endif // PUSH_AUTO_NAV_POINT_COMMAND_HPP_

