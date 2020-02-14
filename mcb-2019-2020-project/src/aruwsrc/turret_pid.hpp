#ifndef __TURRET_PID_HPP__
#define __TURRET_PID_HPP__

#include "src/aruwlib/algorithms/extended_kalman.hpp"

namespace aruwsrc
{

namespace algorithms
{

class TurretPid
{
 public:
    TurretPid(
        float kp,
        float ki,
        float kd,
        float maxD,
        float maxICumulative,
        float maxOutput,
        float lowPassAlpha,
        int derivativeStepsToAverage
    ) :
    kp(kp),
    ki(ki),
    kd(kd),
    maxD(maxD),
    maxICumulative(maxICumulative),
    maxOutput(maxOutput),
    lowPassAlpha(lowPassAlpha),
    derivativeStepsToAverage(derivativeStepsToAverage),
    proportionalKalman(1.0f, 0.0f),
    derivativeKalman(1.0f, 0.0f)
    {
        // derivativePreviousStore = new float[derivativeStepsToAverage];
    }

    ~TurretPid()
    {
        // delete[] derivativePreviousStore;
    }

    float runController(float angleError, float rotationalSpeed);

 private:
    // gains and constants, to be set by the user
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float maxD = 0.0f;
    float maxICumulative = 0.0f;
    float maxOutput = 0.0f;
    float lowPassAlpha = 0.0f;

    // while these could be local, debugging pid is much easier if they are not
    float currErrorP = 0.0f;
    float currErrorI = 0.0f;
    float currErrorD = 0.0f;
    float output = 0.0f;
    float lowPassOutput = 0.0f;

    // derivative averaging
    int derivativeStepsToAverage = 0;
    int averageDIndex = 0;
    float derivativePreviousStore[10];
    float currAverageBeforeDividing = 0.0f;

    uint32_t prevTimeMicroseconds = 0;

    aruwlib::algorithms::ExtendedKalman proportionalKalman;
    aruwlib::algorithms::ExtendedKalman derivativeKalman;
};

}

}

#endif
