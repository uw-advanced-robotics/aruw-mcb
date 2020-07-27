#include <iostream>
#include <string>

#include <aruwlib/Int32Property.hpp>
#include <aruwlib/PropertyTable.hpp>

#include "catch/catch.hpp"

using aruwlib::BaseProperty;
using aruwlib::Int32Property;
using aruwlib::PropertyTable;

TEST_CASE("PropertyTable, addProperty/getProperty")
{
    PropertyTable table;
    Int32Property property(3, "cool property");
    REQUIRE(table.addProperty(&property));
    const Int32Property *propertyPtr =
        dynamic_cast<const Int32Property *>(table.getProperty("cool property"));
    REQUIRE(propertyPtr != nullptr);
    REQUIRE(propertyPtr == &property);
    bool success = table.setProperty<int32_t>("cool property", 30);
    REQUIRE(success);
    REQUIRE(property == 30);
}

TEST_CASE("PropertyTable, bit batch insertion/removal")
{
    PropertyTable table;
    bool addSuccess;
    // Insert the maximum amount of properties in a table.
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        std::string propertyName = "";
        for (int j = 0; j < i + 1; j++)
        {
            propertyName += "a";
        }
        Int32Property *property = new Int32Property(i, propertyName.c_str());
        addSuccess = table.addProperty(property);
        REQUIRE(addSuccess == true);
    }

    // Try and insert another property, this will fail.
    Int32Property property(PropertyTable::PROPERTY_TABLE_MAX_SIZE, "j");
    addSuccess = table.addProperty(&property);
    REQUIRE(addSuccess == false);

    // Insure all the elements are accessable and the elements stored in the table
    // are correct.
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        std::string propertyName = "";
        for (int j = 0; j < i + 1; j++)
        {
            propertyName += "a";
        }
        const Int32Property *propertyPtr =
            dynamic_cast<const Int32Property *>(table.getProperty(propertyName.c_str()));
        REQUIRE(propertyPtr != nullptr);
        REQUIRE(*propertyPtr == i);
    }

    // Remove and delete elements from the table.
    for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
    {
        std::string propertyName = "";
        for (int j = 0; j < i + 1; j++)
        {
            propertyName += "a";
        }
        Int32Property *propertyPtr =
            dynamic_cast<Int32Property *>(table.removeProperty(propertyName.c_str()));
        REQUIRE(propertyPtr != nullptr);
        delete propertyPtr;
    }
    REQUIRE(table.getSize() == 0);
}