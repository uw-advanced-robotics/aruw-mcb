#include <aruwlib/property-stuff/Int32Property.hpp>
#include "catch/catch.hpp"

using aruwlib::Int32Property;

TEST_CASE("Int32 Property: Constructors", "[Int32Property]")
{
    Int32Property p1;
    REQUIRE(p1 == 0);
    REQUIRE(p1.getPropertyName() == "");
    REQUIRE(p1.toString() == "0");
    Int32Property p2(1234);
    REQUIRE(p2 == 1234);
    REQUIRE(p2.getPropertyName() == "");
    REQUIRE(p2.toString() == "1234");
    Int32Property p3(4321, "the property");
    REQUIRE(p3 == 4321);
    REQUIRE(p3.getPropertyName() == "the property");
    REQUIRE(p3.toString() == "4321");
    Int32Property p4(p3);
    REQUIRE(p4 == 4321);
    REQUIRE(p4.getPropertyName() == "the property");
    REQUIRE(p4.toString() == "4321");
}

TEST_CASE("Int32 Property: Equals operator", "[Int32Property]")
{
    Int32Property p1(1234, "the property");
    Int32Property p2;
    p2 = p1;
    REQUIRE(p1 == 1234);
    REQUIRE(p2.getPropertyName() == "the property");
    REQUIRE(p2.toString() == "1234");
    p2 = 4321;
    REQUIRE(p2 == 4321);
    REQUIRE(p2.getPropertyName() == "the property");
    REQUIRE(p2.toString() == "4321");
}

TEST_CASE("Int32 Property: plus operator", "[Int32Property]")
{
    Int32Property p1(1);
    Int32Property p2(2);
    REQUIRE(p1 + p2 == 3);
    REQUIRE(p1 + 3 == 4);
    REQUIRE(3 + p1 == 4);
    p1 += 3;
    REQUIRE(p1 == 4);
    p1 += p2;
    REQUIRE(p1 == 6);
}

TEST_CASE("Int32 Property: minus operator", "[Int32Property]")
{
    Int32Property p1(1);
    Int32Property p2(2);
    REQUIRE(p1 - p2 == -1);
    REQUIRE(p1 - 3 == -2);
    REQUIRE(3 - p1 == 2);
    p1 -= 3;
    REQUIRE(p1 == -2);
    p1 -= p2;
    REQUIRE(p1 == -4);
}

TEST_CASE("Int32 Property: times operator", "[Int32Property]")
{
    Int32Property p1(2);
    Int32Property p2(3);
    REQUIRE(p1 * p2 == 6);
    REQUIRE(p1 * 4 == 8);
    REQUIRE(4 * p1 == 8);
    p1 *= 4;
    REQUIRE(p1 == 8);
    p1 *= p2;
    REQUIRE(p1 == 24);
}

TEST_CASE("Int32 Property: divide operator", "[Int32Property]")
{
    Int32Property p1(10);
    Int32Property p2(2);
    REQUIRE(p1 / p2 == 5);
    REQUIRE(p1 / 5 == 2);
    REQUIRE(20 / p2 == 10);
    p1 /= 2;
    REQUIRE(p1 == 5);
    p1 /= p2;
    REQUIRE(p1 == 2);
}
