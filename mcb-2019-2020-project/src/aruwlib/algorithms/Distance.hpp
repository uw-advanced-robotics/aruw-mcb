#ifndef DISTANCE_HPP_
#define DISTANCE_HPP_

namespace aruwlib
{
namespace algorithms
{

class Distance
{
public:
    enum Units
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

    float asMm() const;
    float asCm() const;
    float asM() const;

    Distance toMm() const;
    Distance toCm() const;
    Distance toM() const;

    Distance operator+(Distance other) const;
    Distance operator-(Distance other) const;
    Distance operator*(float val) const;
    Distance operator/(float val) const;
    Distance operator+=(Distance other);
    Distance operator-=(Distance other);
    Distance operator*=(float val);
    Distance operator/=(float val);

    bool operator<(Distance other) const;
    bool operator>(Distance other) const;
    bool operator<=(Distance other) const;
    bool operator>=(Distance other) const;
    bool operator==(Distance other) const;
    bool operator!=(Distance other) const;

private:
    static constexpr float MM_PER_CM = 10.0f;
    static constexpr float CM_PER_M = 100.0f;
    static constexpr float EQUAL_EPSILON = 0.0001f;

    float distance;
    Units unit;
};  // class Distance

}  // namespace algorithms
}  // namespace aruwlib

#endif  // DISTANCE_HPP_
