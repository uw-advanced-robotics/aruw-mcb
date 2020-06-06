#ifndef LINEAR_INTERPOLATION_HPP_
#define LINEAR_INTERPOLATION_HPP_

#include <cstdint>

namespace aruwlib
{
namespace algorithms
{
/**
 * A class that allows you to linearly interpolate discrete measurements.
 * Useful for increasing the resolution of data measurements. For example,
 * if you receive input data at 10 ms intervals and are running a controller
 * ever 1 ms, you can use this class so that the data you receive won't be in
 * 13 ms steps, but rather will interpolate using past data to predict
 * the data until new data is received.
 *
 * Usecase example, pseudocode:
 *
 * \code
 * LinearInterpolation li;
 * while (true) {
 *     if (new value received) {
 *         li.update(value);
 *     }
 *     runcontroller(li.getInterpolated(aruwlib::arch::clock::getTimeMilliseconds()));
 * }
 * \endcode
 */
class LinearInterpolation
{
public:
    LinearInterpolation();

    /**
     * Updates the interpolation using the newValue.
     *
     * @note only call this when you receive a new value (use remote rx
     *      counter to tell when there is new data from the remote, for
     *      example).
     * @param[in] newValue the new data used in the interpolation.
     */
    void update(float newValue);

    /**
     * Returns the current value, that is: \f$y\f$ in the equation
     * \f$y=slope\cdot (currTime - lastUpdateCallTime) + previousValue\f$.
     *
     * @note use a millisecond-resolution timer, e.g.
     *      aruwlib::arch::clock::getTimeMilliseconds()
     * @param[in] currTime the current clock time, in ms.
     * @return the interpolated value.
     */
    float getInterpolatedValue(uint32_t currTime);

private:
    uint32_t lastUpdateCallTime;  ///< The previous timestamp from when update was called.
    float previousValue;          ///< The previous data value.
    float slope;  ///< The current slope, calculated using the previous and most current data.
};                // class LinearInterpolation

}  // namespace algorithms

}  // namespace aruwlib

#endif  // LINEAR_INTERPOLATION_HPP_
