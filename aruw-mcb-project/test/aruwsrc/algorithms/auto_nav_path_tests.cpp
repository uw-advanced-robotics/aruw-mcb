#include <gtest/gtest.h>

#include "aruwsrc/algorithms/auto_nav_path.hpp"

using namespace testing;
using namespace aruwsrc::algorithms;

class AutoNavPathTest : public Test {
protected:
    constexpr static float kInterpolationDistance = 5.0f;

    AutoNavPathTest() : path(kInterpolationDistance) {}

    AutoNavPath path;
};

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_first_point) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(-1.0f, -1.0f, 0.0f);
    Position closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), p1.x());
    EXPECT_EQ(closest.y(), p1.y());

    // projection is exactly on p1
    current = Position(0.0f, -5.5f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), p1.x());
    EXPECT_EQ(closest.y(), p1.y());
    
    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(6.0f, -1.0f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), p1.x());
    EXPECT_EQ(closest.y(), p1.y());
}

TEST_F(AutoNavPathTest, getClosestOnSegment_clamp_to_second_point) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(6.0f, -1.0f, 0.0f);
    Position closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), p2.x());
    EXPECT_EQ(closest.y(), p2.y());

    // projection is exactly on p2
    current = Position(5.0f, -5.5f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), p2.x());
    EXPECT_EQ(closest.y(), p2.y());
    
    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(-1.0f, -1.0f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), p2.x());
    EXPECT_EQ(closest.y(), p2.y());
}

TEST_F(AutoNavPathTest, getClosestOnSegment_projected_on_segment) {
    Position p1(0.0f, 0.0f, 0.0f);
    Position p2(5.0f, 0.0f, 0.0f);

    // p2 minus p1 is positive
    Position current(2.5f, -1.0f, 0.0f);
    Position closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), 2.5f);
    EXPECT_EQ(closest.y(), 0.0f);

    // p2 minus p1 is negative
    p1 = Position(5.0f, 0.0f, 0.0f);
    p2 = Position(0.0f, 0.0f, 0.0f);
    current = Position(2.5f, -1.0f, 0.0f);
    closest = path.getClosestOnSegment(current, p1, p2);
    EXPECT_EQ(closest.x(), 2.5f);
    EXPECT_EQ(closest.y(), 0.0f);
}
