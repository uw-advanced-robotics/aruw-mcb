#ifndef FLOAT_PROPERTY_HPP_
#define FLOAT_PROPERTY_HPP_

#include "PrimativeProperty.hpp"

namespace aruwlib
{

class FloatProperty : public PrimativeProperty
{
public:
    FloatProperty() : data(0.0f) {}
    FloatProperty(float data) : data(data) {}
    FloatProperty(const FloatProperty& other) : data(other.data) {}

    const uint8_t *getData() const override { return reinterpret_cast<const uint8_t *>(&data); }
    uint8_t getDataLength() const override { return sizeof(float); }
    uint8_t getPrimativePropertyType() const override { return FLOAT_PROPERTY_TYPE; }
    float get() { return data; }

    friend bool operator==(FloatProperty p1, FloatProperty p2);
    friend bool operator!=(FloatProperty p1, FloatProperty p2);
    friend bool operator<(FloatProperty p1, FloatProperty p2);
    friend bool operator>(FloatProperty p1, FloatProperty p2);
    friend bool operator<=(FloatProperty p1, FloatProperty p2);
    friend bool operator>=(FloatProperty p1, FloatProperty p2);
    friend FloatProperty operator+(FloatProperty p1, FloatProperty p2);
    friend FloatProperty operator+(FloatProperty p1, float p2);
    friend FloatProperty operator+(float p1, FloatProperty p2);

    FloatProperty operator=(FloatProperty other);
    FloatProperty operator=(float other);
    FloatProperty operator+=(FloatProperty other);
    FloatProperty operator+=(float other);
    FloatProperty operator-=(FloatProperty other);
    FloatProperty operator-=(float other);
    FloatProperty operator*=(FloatProperty other);
    FloatProperty operator*=(float other);
    FloatProperty operator/=(FloatProperty other);
    FloatProperty operator/=(float other);

private:
    float data;
};

}  // namespace aruwlib

#endif  // FLOAT_PROPERTY_HPP_
