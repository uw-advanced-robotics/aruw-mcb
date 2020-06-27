#include <iostream>
#include <string>

#include <aruwlib/property-stuff/Int32Property.hpp>
#include <aruwlib/property-stuff/PropertyTable.hpp>
#include "catch/catch.hpp"

using aruwlib::BaseProperty;
using aruwlib::Int32Property;
using aruwlib::PropertyTable;

TEST_CASE("Proeprty Table", "[proprety_table]")
{
    PropertyTable::resetMainPropertyTable();

    SECTION("PropertyTable.addProperty/getProperty")
    {
        Int32Property property(3, "cool property");
        REQUIRE(
            PropertyTable::getMainPropertySystem().addProperty(
                dynamic_cast<BaseProperty *>(&property)) == true);
        const Int32Property *propertyPtr =
            PropertyTable::getMainPropertySystem().getProperty<Int32Property>("cool property");
        REQUIRE(propertyPtr != nullptr);
        REQUIRE(propertyPtr == &property);
        bool success =
            PropertyTable::getMainPropertySystem().setProperty<int32_t>("cool property", 30);
        REQUIRE(success);
        REQUIRE(property == 30);
    }

    SECTION("PropertyTable big batch insertion/removal")
    {
        bool addSuccess;
        // Insert the maximum amount of properties in a table.
        for (int i = 0; i < PropertyTable::PROPERTY_TABLE_MAX_SIZE; i++)
        {
            std::string propertyName = "";
            for (int j = 0; j < i + 1; j++)
            {
                propertyName += "a";
            }
            Int32Property *property = new Int32Property(i, propertyName);
            addSuccess = PropertyTable::getMainPropertySystem().addProperty(property);
            REQUIRE(addSuccess == true);
        }

        // Try and insert another property, this will fail.
        Int32Property property(PropertyTable::PROPERTY_TABLE_MAX_SIZE, "j");
        addSuccess = PropertyTable::getMainPropertySystem().addProperty(&property);
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
                PropertyTable::getMainPropertySystem().getProperty<Int32Property>(propertyName);
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
            Int32Property *propertyPtr;
            bool removeSuccess =
                PropertyTable::getMainPropertySystem().removeProperty<Int32Property>(
                    propertyName,
                    &propertyPtr);
            REQUIRE(removeSuccess == true);
            REQUIRE(propertyPtr != nullptr);
            delete propertyPtr;
        }
        REQUIRE(PropertyTable::getMainPropertySystem().getSize() == 0);
    }
}
