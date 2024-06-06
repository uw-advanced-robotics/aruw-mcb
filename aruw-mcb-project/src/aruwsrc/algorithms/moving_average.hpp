#ifndef MOVING_AVERAGE_HPP_
#define MOVING_AVERAGE_HPP_

#include <queue>

class MovingAverage
{
public:
    MovingAverage(size_t windowSize) : windowSize(windowSize), sum(0.0f) {}
    void update(float val)
    {
        sum += val;
        vals.push(val);
        if (vals.size() > windowSize)
        {
            sum -= vals.front();
            vals.pop();
        }
    }
    float getVal() const { return sum / windowSize; };

    const size_t windowSize;

private:
    std::queue<float> vals;
    float sum;
};

#endif