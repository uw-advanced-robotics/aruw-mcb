#include <aruwlib/algorithms/contiguous_float.hpp>
#include <CppUTest/CommandLineTestRunner.h>

using namespace aruwlib::algorithms;

static const float EQUALITY_TOLERANCE = 0.0001f; 

TEST_GROUP(ContiguousFloat)
{
};

TEST(ContiguousFloat, Basic_functionality)
{
    ContiguousFloat testInstance(5, 0, 10);
    DOUBLES_EQUAL(5, testInstance.getValue(), EQUALITY_TOLERANCE);
}

TEST(ContiguousFloat, Wrapping_behavior)
{
    ContiguousFloat testInstance(-4, 0, 10);
    DOUBLES_EQUAL(6, testInstance.getValue(), EQUALITY_TOLERANCE);

    testInstance.setValue(16);
    DOUBLES_EQUAL(6, testInstance.getValue(), EQUALITY_TOLERANCE);

    testInstance.setValue(28);
    DOUBLES_EQUAL(8, testInstance.getValue(), EQUALITY_TOLERANCE);
}

TEST(ContiguousFloat, Difference)
{
    ContiguousFloat testInstance(2, 0, 10);
    DOUBLES_EQUAL(2, testInstance.difference(4), EQUALITY_TOLERANCE);
    DOUBLES_EQUAL(-1, testInstance.difference(11), EQUALITY_TOLERANCE);

    testInstance.setValue(9);
    DOUBLES_EQUAL(2, testInstance.difference(11), EQUALITY_TOLERANCE);

    testInstance.setValue(10);
    DOUBLES_EQUAL(1, testInstance.difference(1), EQUALITY_TOLERANCE);
    testInstance.setValue(1);
    DOUBLES_EQUAL(-1, testInstance.difference(10), EQUALITY_TOLERANCE);
}

TEST(ContiguousFloat, Rotation_bounds)
{
    ContiguousFloat testInstance(150, -180, 180);

    DOUBLES_EQUAL(40, testInstance.difference(190), EQUALITY_TOLERANCE);
    DOUBLES_EQUAL(40, testInstance.difference(-170), EQUALITY_TOLERANCE);

    DOUBLES_EQUAL(40, testInstance.difference(190), EQUALITY_TOLERANCE);
    DOUBLES_EQUAL(40, testInstance.difference(-170), EQUALITY_TOLERANCE);

    testInstance.setValue(180);

    DOUBLES_EQUAL(180, testInstance.getValue(), EQUALITY_TOLERANCE);
    DOUBLES_EQUAL(0, testInstance.difference(-180), EQUALITY_TOLERANCE);

    ContiguousFloat testInstance2(40, -180, 180);
    DOUBLES_EQUAL(-140, testInstance2.difference(-100), EQUALITY_TOLERANCE);
}

TEST(ContiguousFloat, Shifting_value)
{
    ContiguousFloat testInstance(150, -180, 180);

    testInstance.shiftValue(40);
    DOUBLES_EQUAL(-170, testInstance.getValue(), EQUALITY_TOLERANCE);

    testInstance.shiftValue(40);
    DOUBLES_EQUAL(-130, testInstance.getValue(), EQUALITY_TOLERANCE);

    testInstance.shiftValue(360);
    DOUBLES_EQUAL(-130, testInstance.getValue(), EQUALITY_TOLERANCE);

    testInstance.shiftValue(0);
    DOUBLES_EQUAL(-130, testInstance.getValue(), EQUALITY_TOLERANCE);
}

TEST(ContiguousFloat, Bad_bounds)
{
    ContiguousFloat testInstance(150, 180, -180);
    DOUBLES_EQUAL(-180, testInstance.getLowerBound(), EQUALITY_TOLERANCE);
    DOUBLES_EQUAL(180, testInstance.getUpperBound(), EQUALITY_TOLERANCE);
}