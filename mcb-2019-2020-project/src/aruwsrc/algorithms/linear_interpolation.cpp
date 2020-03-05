#include "linear_interpolation.hpp"

    LinearInterpolation::LinearInterpolation(float frequency) :
        currInitialPoint(0),
        frequency(frequency),
        yValues{0.0f, 0.0f, 0.0f},
        lastUpdateCallTime(0),
        prevY(0.0f),
        slope(0.0f)
    {

    }

    void LinearInterpolation::update(float newY)
    {
        // yValues[currInitialPoint] = newY;
        // currInitialPoint = (currInitialPoint + 1) % 3;
        uint32_t currTime = modm::Clock::now().getTime();
        slope = (newY - prevY) / (currTime - lastUpdateCallTime);
        prevY = newY;
        lastUpdateCallTime = currTime;
    }

    float wn=0;
float w1, w2, w3;
    float LinearInterpolation::getInterpolatedValue(uint32_t currTime)
    {
        wn = static_cast<float>(currTime - lastUpdateCallTime);
        return slope * wn + prevY;
        // wn = static_cast<float>(currTime - lastUpdateCallTime)
                // / 1000.0f + 2.0f / frequency;
        // float y1 = getY1();
        // float y2 = getY2();
        // float y3 = getY3();
        // return y1
        //         + frequency * (-3.0f * y1 / 2.0f + y2 - y3 / 2.0f) * wn
        //         + powf(frequency, 2.0f)
        //         * (y1 / 2.0f - y2 + y3 / 2.0f)
        //         * powf(wn, 2.0f);
    }

    float LinearInterpolation::getY1()
    {
        w1 = yValues[currInitialPoint];
        return yValues[currInitialPoint];
    }
    
    float LinearInterpolation::getY2()
    {
        w2 = yValues[(currInitialPoint + 1) % 3];
        return yValues[(currInitialPoint + 1) % 3];
    }

    float LinearInterpolation::getY3()
    {
        w3 = yValues[(currInitialPoint + 2) % 3];
        return yValues[(currInitialPoint + 2) % 3];
    }