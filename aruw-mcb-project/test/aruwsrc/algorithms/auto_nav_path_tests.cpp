#include <gtest/gtest.h>

#include "aruwsrc/algorithms/auto_nav_path.hpp"

using namespace testing;
using namespace aruwsrc::algorithms;

class AutoNavPathTest : public Test {
protected:
    constexpr static float kInterpolationDistance = 2.5f; // number of segments between path points

    AutoNavPathTest() : path() {}

    AutoNavPath path;
};

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_first_point) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(-1.0f, -1.0f, 0.0f);
    float closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.0f, closest);

    // projection is exactly on p1
    current = Position(0.0f, -5.5f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.0f, closest);
    
    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(6.0f, -1.0f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.0f, closest);
}

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_second_point) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(6.0f, -1.0f, 0.0f);
    float closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(1.0f, closest);

    // projection is exactly on p2
    current = Position(5.0f, -5.5f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(1.0f, closest);
    
    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(-1.0f, -1.0f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(1.0f, closest);
}

TEST_F(AutoNavPathTest, getClosestOnSegment_projected_on_segment) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(2.5f, -1.0f, 0.0f);
    float closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.5f, closest);

    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(2.5f, -1.0f, 0.0f);
    closest = path.getClosestParameterOnSegment(current, p1, p2);
    EXPECT_EQ(0.5f, closest);
}

TEST_F(AutoNavPathTest, calculate_setpoint_point_no_point) {
    float interpolationParameter = 1.5f;
    Position current(1, 1, 1);
    path.resetPath();
    Position setpoint = path.calculateSetPoint(current, interpolationParameter);
    
    EXPECT_EQ(Position(1, 1, 1), setpoint);
}

TEST_F(AutoNavPathTest, calculate_setpoint_at_only_point) {
    float interpolationParameter = 1.5f;
    Position current(1, 2, 3);
    path.pushPoint(current);
    Position setpoint = path.calculateSetPoint(current, interpolationParameter);

    EXPECT_EQ(Position(1, 2, 3), setpoint);
}

TEST_F(AutoNavPathTest, calculate_setpoint_single_point) {
    float interpolationParameter = 1.5f;
    Position current(0, 0, 0);
    path.pushPoint(Position(1, 2, 3));
    Position setpoint = path.calculateSetPoint(current, interpolationParameter);
    
    EXPECT_EQ(Position(1, 2, 3), setpoint);
}

TEST_F(AutoNavPathTest, calculate_setpoint_point_from_start) {
    float interpolationParameter = 2.5f;
    Position current(0, 0, 0);
    path.pushPoint(Position(1.0, 1.0, 0.0));
    path.pushPoint(Position(2.0, 2.0, 0.0));
    path.pushPoint(Position(3, 3, 0));
    path.pushPoint(Position(4, 5, 0));
    Position setpoint = path.calculateSetPoint(current, interpolationParameter);

    EXPECT_EQ(Position(3.5, 4, 0), setpoint);
}  

TEST_F(AutoNavPathTest, calculate_setpoint_point_from_middle) {
    float interpolationParameter = 1.0f;
    Position current(3, 2, 0);
    path.pushPoint(Position(1.0, 1.0, 0.0));
    path.pushPoint(Position(2.0, 2.0, 0.0));
    path.pushPoint(Position(3, 3, 0));
    path.pushPoint(Position(4, 5, 0));
    Position setpoint = path.calculateSetPoint(current, interpolationParameter);

    EXPECT_EQ(Position(3.5, 4, 0), setpoint);
}  
