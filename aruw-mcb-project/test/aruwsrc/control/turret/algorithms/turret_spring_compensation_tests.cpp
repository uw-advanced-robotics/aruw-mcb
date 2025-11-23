#include <cmath>

#include <gtest/gtest.h>

#include "tap/algorithms/transforms/transform.hpp"

#include "aruwsrc/control/turret/algorithms/turret_spring_compensation.hpp"

using namespace aruwsrc::control::turret::algorithms;
using namespace tap::algorithms::transforms;

// Convenience
using Params = TurretSpringForceOffset::TurretSpringParams;
using State = TurretSpringForceOffset::TurretCompensatorState;

//
// 1) Spring in-line with rotation axis: no torque
//
TEST(TurretSpringSpecialCases, InlineSpring_NoTorque)
{
    Params p;
    p.turretPitchMountX = 0.0f;
    p.turretPitchMountZ = 0.0f;  // pivot at origin
    p.turretYawMountX = 0.0f;
    p.turretYawMountZ = 1.0f;  // spring entirely on rotation axis

    p.springConstant = 1.0f;
    p.springFreeLength = 0.0f;

    TurretSpringForceOffset comp(p, false);

    State s;
    s.pitchChassisFrame = 0.0f;

    EXPECT_NEAR(comp.calculateCompensationEffort(s), 0.0f, 1e-6f);
}

//
// 2) Purely vertical spring: no torque (radial force only)
//
TEST(TurretSpringSpecialCases, VerticalSpring_NoTorque)
{
    Params p;
    p.turretPitchMountX = 0.0f;
    p.turretPitchMountZ = 0.0f;

    p.turretYawMountX = 0.0f;
    p.turretYawMountZ = 1.0f;  // vertical

    p.springConstant = 10.0f;
    p.springFreeLength = 0.0f;

    TurretSpringForceOffset comp(p, false);

    State s;
    s.pitchChassisFrame = 0.0;

    EXPECT_NEAR(comp.calculateCompensationEffort(s), 0.0f, 1e-6f);
}

TEST(TurretSpringSpecialCases, GeometricValidation)
{
    Params p;
    p.turretPitchMountX = 0.0f;
    p.turretPitchMountZ = 1.0f;  // Arm is "up" when pitch is 0

    p.turretYawMountX = 1.0f;  // Anchor is forward 1m
    p.turretYawMountZ = 1.0f;  // Anchor is up 1m

    p.springConstant = 1.0f;
    p.springFreeLength = 0.0f;

    TurretSpringForceOffset comp(p, false);

    State s;

    // Case A: Pitch = 0 radians
    // Arm is at (0,0,1). Anchor at (1,0,1).
    // Spring pulls +X. Arm is +Z.
    // Torque = Z cross X = +Y (Positive Torque by spring).
    // Counter Torque (Output) must be NEGATIVE.
    // Magic Number: -1.0
    s.pitchChassisFrame = 0.0f;
    EXPECT_NEAR(comp.calculateCompensationEffort(s), -1.0f, 1e-5f) << "Failed at 0 degrees";

    // Case B: Pitch = 30 degrees (PI/6)
    // Formula: sin(30) - cos(30) = 0.5 - 0.866025 = -0.366025
    s.pitchChassisFrame = M_PI / 6.0f;
    EXPECT_NEAR(comp.calculateCompensationEffort(s), -0.366025f, 1e-5f) << "Failed at 30 degrees";

    // Case C: Pitch = 45 degrees (PI/4)
    // Arm vector points at (0.707, 0, 0.707).
    // Anchor is (1, 0, 1).
    // The Arm vector and Spring vector are PARALLEL.
    // Cross product must be ZERO.
    s.pitchChassisFrame = M_PI / 4.0f;
    EXPECT_NEAR(comp.calculateCompensationEffort(s), 0.0f, 1e-5f)
        << "Failed at 45 degrees (Parallel vectors should yield 0 torque)";

    // Case D: Pitch = 90 degrees (PI/2)
    // Arm is at (1,0,0). Anchor at (1,0,1).
    // Spring pulls +Z. Arm is +X.
    // Torque = X cross Z = -Y (Negative Torque by spring).
    // Counter Torque (Output) must be POSITIVE.
    s.pitchChassisFrame = M_PI / 2.0f;
    EXPECT_NEAR(comp.calculateCompensationEffort(s), 1.0f, 1e-5f) << "Failed at 90 degrees";
}

//
// 4) Inversion flips sign
//
TEST(TurretSpringSpecialCases, InversionFlipsSign)
{
    Params p;
    p.turretPitchMountX = 0.0f;
    p.turretPitchMountZ = 1.0f;

    p.turretYawMountX = 1.0f;
    p.turretYawMountZ = 1.0f;

    p.springConstant = 2.0f;
    p.springFreeLength = 0.0f;

    TurretSpringForceOffset normal(p, false);
    TurretSpringForceOffset inverted(p, true);

    State s;
    s.pitchChassisFrame = 0.0f;  // Use 0 deg where torque is known to be -2.0

    float tNormal = normal.calculateCompensationEffort(s);
    float tInverted = inverted.calculateCompensationEffort(s);

    EXPECT_FLOAT_EQ(tNormal, -2.0f);
    EXPECT_FLOAT_EQ(tInverted, 2.0f);
}