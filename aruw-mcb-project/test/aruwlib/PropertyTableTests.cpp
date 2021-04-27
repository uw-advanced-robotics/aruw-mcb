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

#include <iostream>
#include <set>
#include <string>

#include <aruwlib/Int32Property.hpp>
#include <aruwlib/PropertyTable.hpp>
#include <gtest/gtest.h>

using aruwlib::BaseProperty;
using aruwlib::IBaseProperty;
using aruwlib::Int32Property;
using aruwlib::PropertyTable;

TEST(
    PropertyTable,
    addProperty_successful_using_Int32Property_getProperty_returns_pointer_to_added_property)
{
    PropertyTable ptable;
    Int32Property property(3, "cool property");

    EXPECT_TRUE(ptable.addProperty(&property));
    const Int32Property *propertyPtr =
        dynamic_cast<const Int32Property *>(ptable.getProperty("cool property"));
    EXPECT_NE(nullptr, propertyPtr);
    EXPECT_EQ(propertyPtr, &property);
}

TEST(PropertyTable, removeProperty_successfully_removes_Int32Property_if_in_table)
{
    PropertyTable ptable;
    Int32Property property(3, "cool property");

    EXPECT_TRUE(ptable.addProperty(&property));
    const Int32Property *propertyPtr =
        dynamic_cast<const Int32Property *>(ptable.removeProperty("cool property"));
    EXPECT_NE(nullptr, propertyPtr);
    EXPECT_EQ(propertyPtr, &property);
}

TEST(PropertyTable, setProperty)
{
    PropertyTable ptable;
    Int32Property p(42, "p");

    ptable.addProperty(&p);
    const Int32Property *ppointer = dynamic_cast<const Int32Property *>(ptable.getProperty("p"));
    EXPECT_EQ(ppointer, &p);
    EXPECT_EQ(42, *ppointer);
    EXPECT_TRUE(ptable.setProperty<int32_t>("p", 41));
    EXPECT_EQ(41, *ppointer);
    EXPECT_EQ(41, p);
}

TEST(PropertyTable, isFull_getSize_reflect_number_of_properties_in_table)
{
    PropertyTable ptable;
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        EXPECT_FALSE(ptable.isFull());
        EXPECT_EQ(i, ptable.getSize());
        Int32Property *p = new Int32Property(i, std::to_string(i).c_str());
        ptable.addProperty(p);
    }

    Int32Property property(PropertyTable::PROPERTY_TABLE_MAX_SIZE, "extra property");
    EXPECT_FALSE(ptable.addProperty(&property));

    EXPECT_TRUE(ptable.isFull());
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        IBaseProperty *p = ptable.removeProperty(std::to_string(i));
        delete p;
    }
}

TEST(
    PropertyTable,
    getPropertyTableBeginning_getPropertyTableEnd_allows_for_proper_iteration_through_elements)
{
    PropertyTable ptable;
    std::set<std::string> ptableContents;
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        EXPECT_FALSE(ptable.isFull());
        EXPECT_EQ(i, ptable.getSize());
        ptableContents.emplace(std::to_string(i));
        Int32Property *p = new Int32Property(i, std::to_string(i).c_str());
        ptable.addProperty(p);
    }

    auto iter = ptable.getPropertyTableBeginning();
    while (iter != ptable.getPropertyTableEnd())
    {
        EXPECT_EQ(1, ptableContents.count(iter->first));
        ptableContents.erase(iter->first);
        iter++;
    }
    EXPECT_EQ(0, ptableContents.size());

    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        IBaseProperty *p = ptable.removeProperty(std::to_string(i));
        delete p;
    }
}
