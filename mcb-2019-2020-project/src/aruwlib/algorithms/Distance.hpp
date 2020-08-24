#ifndef DISTANCE_HPP_
#define DISTANCE_HPP_

#include <cinttypes>

namespace aruwlib
{
namespace algorithms
{
class Distance
{
public:
    enum class Units : uint8_t
    {
        MM,
        CM,
        M,
    };

    Distance();
    Distance(float distance, Units unit);
    Distance(const Distance& other) = default;
    ~Distance() = default;
    Distance& operator=(const Distance& other) = default;

    void set(float val);

    /**
     * @return The distance in mm.
     */
    float asMm() const;

    /**
     * @return The distance in cm.
     */
    float asCm() const;

    /**
     * @return The distance in m.
     */
    float asM() const;

    /**
     * Creates an instance of a Distance class, converting `this`'s distance to
     * millimeters and then returns the Distance.
     *
     * @note the current distance is not changed (hence the `const`).
     */
    Distance toMm() const;

    /**
     * Creates an instance of a Distance class, converting `this`'s distance to
     * centimeters and then returns the Distance.
     *
     * @note the current distance is not changed (hence the `const`).
     */
    Distance toCm() const;

    /**
     * Creates an instance of a Distance class, converting `this`'s distance to
     * meters and then returns the Distance.
     *
     * @note the current distance is not changed (hence the `const`).
     */
    Distance toM() const;

    // Note, addition and subtraction with distances without units doesn't make sense, which
    // is why they are not provided.

    /**
     * Adds `this` and `other`'s distance, returning an instance of a Distance
     * with the same units as `this`.
     *
     * @param[in] other The distance to add.
     * @return The added distance.
     */
    Distance operator+(Distance other) const;

    /**
     * Subtracts `other` from `this`, returning an instance of a Distance
     * with the same units as `this`.
     *
     * @param[in]
     * @return The subtracted distance.
     */
    Distance operator-(Distance other) const;

    /**
     * Multiplies `this`'s distance by `val`, returning the multiplied distance using
     * `this`'s units.
     *
     * @param[in] val The value to multiply `this`'s distance by.
     * @return This multiplied distance.
     */
    Distance operator*(float val) const;

    /**
     * @note The units returned are that of `this`'s.
     * @see operator*(float val)
     */
    Distance operator*(Distance other) const;

    /**
     * Divides `this`'s distance by `val`, returning the divided distance using
     * `this`'s units.
     *
     * @param[in] val The value to divide `this`'s distance by.
     * @return The divided distance.
     */
    Distance operator/(float val) const;

    /**
     * @note The units returned are that of `this`'s.
     * @see operator/(float val)
     */
    Distance operator/(Distance other) const;

    /**
     * Adds on to `this`'s distance from `other`'s distance, returning the new distance
     * using `this`'s units.
     */
    Distance operator+=(Distance other);

    /**
     * Subtracts from `this`'s distance from `other`'s distance, returning the new distance
     * using `this`'s units.
     */
    Distance operator-=(Distance other);

    /**
     * Multiplies to `this`'s distance from `val`'s distance, returning the new distance
     * using `this`'s units.
     */
    Distance operator*=(float val);

    /**
     * Multiplies to `this`'s distance from `other`'s distance, returning the new distance
     * using `this`'s units.
     */
    Distance operator*=(Distance other);

    /**
     * Divides `this`'s distance by `val`'s distance, returning the new distance
     * using `this`'s units.
     */
    Distance operator/=(float val);

    /**
     * Divides `this`'s distance by `other`'s distance, returning the new distance
     * using `this`'s units.
     */
    Distance operator/=(Distance other);

    /**
     * Adds one unit to the `this`'s distance. The units are not changed, so if the units
     * are in meters, one meter is added to `this`, whereas if the units are in millimeters,
     * one millimeter is added to `this`.
     */
    Distance operator++();

    /**
     * Subtracts one unit to the `this`'s distance. The units are not changed, so if the units
     * are in meters, one meter is subtracted from `this`, whereas if the units are in millimeters,
     * one millimeter is subtracted from `this`.
     */
    Distance operator--();

    /*
     * These comparisons are as they seem. In particular, note that `other`'s distance is
     * converted into the same units as `this` for comparison.
     */
    bool operator<(Distance other) const;
    bool operator>(Distance other) const;
    bool operator<=(Distance other) const;
    bool operator>=(Distance other) const;
    /**
     * @note Equality is determined by using `aruwlib::algorithms::compareFloatClose`, with
     *      an epsilon of EQUAL_EPSILON.
     */
    bool operator==(Distance other) const;
    /**
     * @see `operator==` for now equality is determined.
     */
    bool operator!=(Distance other) const;

private:
    static constexpr float MM_PER_CM = 10.0f;
    static constexpr float CM_PER_M = 100.0f;
    static constexpr float EQUAL_EPSILON = 0.000001f;

    float distance;
    Units unit;
};  // class Distance

}  // namespace algorithms
}  // namespace aruwlib

#endif  // DISTANCE_HPP_
