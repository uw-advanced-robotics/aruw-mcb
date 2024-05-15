#include <gtest/gtest.h>

#include "aruwsrc/algorithms/auto_nav_path.hpp"

using namespace testing;
using namespace aruwsrc::algorithms;

class AutoNavPathTest : public Test {
protected:
    constexpr static float kInterpolationDistance = 2.5f; // number of segments between path points

    AutoNavPathTest() : path(kInterpolationDistance) {}

    AutoNavPath path;
};

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_first_point) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(-1.0f, -1.0f, 0.0f);
    Position closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(p1.x(), closest.x());
    EXPECT_EQ(p1.y(), closest.y());

    // projection is exactly on p1
    current = Position(0.0f, -5.5f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(p1.x(), closest.x());
    EXPECT_EQ(p1.y(), closest.y());
    
    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(6.0f, -1.0f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(p1.x(), closest.x());
    EXPECT_EQ(p1.y(), closest.y());
}

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_second_point) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(6.0f, -1.0f, 0.0f);
    Position closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(p2.x(), closest.x());
    EXPECT_EQ(p2.y(), closest.y());

    // projection is exactly on p2
    current = Position(5.0f, -5.5f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(p2.x(), closest.x());
    EXPECT_EQ(p2.y(), closest.y());
    
    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(-1.0f, -1.0f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(p2.x(), closest.x());
    EXPECT_EQ(p2.y(), closest.y());
}

TEST_F(AutoNavPathTest, getClosestOnSegment_projected_on_segment) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(2.5f, -1.0f, 0.0f);
    Position closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(2.5f, closest.x());
    EXPECT_EQ(0.0f, closest.y());

    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(2.5f, -1.0f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(2.5f, closest.x());
    EXPECT_EQ(0.0f, closest.y());
}

TEST_F(AutoNavPathTest, set_interpolated_point) {
    Position current(0, 0, 0);
    path.pushPoint(current);
    path.pushPoint(Position(2.0, 2.0, 0.0));
    path.pushPoint(Position(3, 3, 0));
    path.pushPoint(Position(4, 5, 0));
    Position setpoint = path.setInterpolatedPoint(current);

    EXPECT_EQ(Position(3.5, 4, 0), setpoint);
}  

TEST_F(AutoNavPathTest, set_interpolated_point_no_points) {
    Position current(1, 2, 3);
    path.pushPoint(current);
    Position setpoint = path.setInterpolatedPoint(current);

    EXPECT_EQ(Position(1, 2, 3), setpoint);
}
