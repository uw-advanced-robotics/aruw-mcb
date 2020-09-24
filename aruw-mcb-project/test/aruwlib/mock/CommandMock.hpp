/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef COMMAND_MOCK_HPP_
#define COMMAND_MOCK_HPP_

class CommandMock : public aruwlib::control::Command
{
public:
    CommandMock() = default;
    MOCK_METHOD(const std::set<Subsystem*>&, getRequirements, (), (const override));
    MOCK_METHOD(bool, hasRequirement, (Subsystem * requirement), (const override));
    MOCK_METHOD(void, addSubsystemRequirement, (Subsystem * requirement), (override));
    MOCK_METHOD(const char*, getName, (), (const override));
    MOCK_METHOD(void, initialize, (), (override));
    MOCK_METHOD(void, execute, (), (override));
    MOCK_METHOD(void, end, (bool interrupted), (override));
    MOCK_METHOD(bool, isFinished, (), (const override));
};  // class CommandMock

#endif  // COMMAND_MOCK_HPP_
