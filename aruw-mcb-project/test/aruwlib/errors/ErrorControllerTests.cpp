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

#include <aruwlib/Drivers.hpp>
#include <aruwlib/errors/create_errors.hpp>
#include <aruwlib/errors/error_controller.hpp>
#include <gtest/gtest.h>

using aruwlib::Drivers;
using namespace aruwlib::errors;

TEST(ErrorController, getSystemError_returns_nullptr_if_out_of_bounds)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e);

    EXPECT_EQ(nullptr, ec.getSystemError(1));
}

TEST(ErrorController, getSystemError_returns_element_at_index_with_single_element_in_list)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e);

    EXPECT_EQ(e, *ec.getSystemError(0));
}

TEST(ErrorController, getSystemError_returns_element_at_index_with_multiple_elements_in_list)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e2, *ec.getSystemError(1));
    EXPECT_EQ(e3, *ec.getSystemError(2));
}

TEST(ErrorController, removeFront_removes_first_element_in_controller_with_one_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e);

    ec.removeFront();
    EXPECT_EQ(0, ec.getErrorListSize());
}

TEST(ErrorController, removeFront_removes_first_element_in_controller_with_multiple_SystemErrors)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    ec.removeFront();
    EXPECT_EQ(2, ec.getErrorListSize());
    EXPECT_EQ(e2, *ec.getSystemError(0));
    EXPECT_EQ(e3, *ec.getSystemError(1));
}

TEST(ErrorController, removeBack_removes_last_element_in_controller_with_one_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e);

    ec.removeBack();
    EXPECT_EQ(0, ec.getErrorListSize());
}

TEST(ErrorController, removeBack_removes_last_element_in_controller_with_multiple_SystemErrors)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    ec.removeBack();
    EXPECT_EQ(2, ec.getErrorListSize());
    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e2, *ec.getSystemError(1));
}

TEST(ErrorController, removeSystemError_no_SystemErrors_returns_false)
{
    Drivers drivers;
    ErrorController ec(&drivers);

    EXPECT_FALSE(ec.removeSystemError(
        SystemError("desc", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE)));
}

TEST(ErrorController, removeSystemError_single_SystemError_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    ec.addToErrorList(e);

    EXPECT_TRUE(ec.removeSystemError(e));
    EXPECT_EQ(nullptr, ec.getSystemError(0));
}

TEST(
    ErrorController,
    removeSystemError_start_of_multiple_SystemErrors_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_TRUE(ec.removeSystemError(e1));
    EXPECT_EQ(e2, *ec.getSystemError(0));
    EXPECT_EQ(e3, *ec.getSystemError(1));
}

TEST(
    ErrorController,
    removeSystemError_middle_of_multiple_SystemErrors_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_TRUE(ec.removeSystemError(e2));
    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e3, *ec.getSystemError(1));
}

TEST(
    ErrorController,
    removeSystemError_end_of_multiple_SystemErrors_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_TRUE(ec.removeSystemError(e3));
    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e2, *ec.getSystemError(1));
}

TEST(
    ErrorController,
    removeSystemError_SystemError_not_in_error_list_nothing_removed_returned_false)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);

    EXPECT_FALSE(ec.removeSystemError(e3));
    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e2, *ec.getSystemError(1));
}

TEST(ErrorController, removeSystemErrorAtIndex_no_SystemErrors_returns_false)
{
    Drivers drivers;
    ErrorController ec(&drivers);

    EXPECT_FALSE(ec.removeSystemErrorAtIndex(0));
}

TEST(
    ErrorController,
    removeSystemErrorAtIndex_single_SystemError_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    ec.addToErrorList(e);

    EXPECT_TRUE(ec.removeSystemErrorAtIndex(0));
    EXPECT_EQ(nullptr, ec.getSystemError(0));
}

TEST(
    ErrorController,
    removeSystemErrorAtIndex_start_of_multiple_SystemErrors_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_TRUE(ec.removeSystemErrorAtIndex(0));
    EXPECT_EQ(e2, *ec.getSystemError(0));
    EXPECT_EQ(e3, *ec.getSystemError(1));
}

TEST(
    ErrorController,
    removeSystemErrorAtIndex_middle_of_multiple_SystemErrors_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_TRUE(ec.removeSystemErrorAtIndex(1));
    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e3, *ec.getSystemError(1));
}

TEST(
    ErrorController,
    removeSystemErrorAtIndex_end_of_multiple_SystemErrors_returns_true_and_removes_SystemError)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    EXPECT_TRUE(ec.removeSystemErrorAtIndex(2));
    EXPECT_EQ(e1, *ec.getSystemError(0));
    EXPECT_EQ(e2, *ec.getSystemError(1));
}

TEST(ErrorController, removeAllSystemErrors_with_zero_SystemErrors_does_nothing)
{
    Drivers drivers;
    ErrorController ec(&drivers);

    ec.removeAllSystemErrors();
    EXPECT_EQ(0, ec.getErrorListSize());
}

TEST(ErrorController, removeAllSystemErrors_with_three_SystemErrors_removes_all_of_them)
{
    Drivers drivers;
    ErrorController ec(&drivers);
    SystemError e1(
        "err",
        1,
        "file",
        Location::CAN_RX,
        ErrorType::ADDING_COMMAND_WITH_NULL_SUBSYSTEM_DEPENDENCIES);
    SystemError e2("err", 1, "file", Location::CAN_RX, ErrorType::ADDING_NULLPTR_COMMAND);
    SystemError e3("err", 1, "file", Location::CAN_RX, ErrorType::CRC_FAILURE);
    ec.addToErrorList(e1);
    ec.addToErrorList(e2);
    ec.addToErrorList(e3);

    ec.removeAllSystemErrors();
    EXPECT_EQ(0, ec.getErrorListSize());
}
