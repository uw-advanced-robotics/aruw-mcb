#ifndef INT32_PROPERTY_HPP_
#define INT32_PROPERTY_HPP_

#include <cinttypes>

#include "PrimativeProperty.hpp"

namespace aruwlib
{

class Int32Property : public PrimativeProperty
{
public:
    Int32Property() : data(0) {}
    Int32Property(bool data) : data(data) {}
    Int32Property(const Int32Property &other) : data(other.data) {}

    const uint8_t *getData() const override { return reinterpret_cast<const uint8_t *>(&data); }
    uint8_t getDataLength() const override { return sizeof(int32_t); }
    uint8_t getPrimativePropertyType() const override { return INT32_PROPERTY_TYPE; }
    uint32_t get() { return data; }

    friend bool operator==(Int32Property p1, Int32Property p2);
    friend bool operator!=(Int32Property p1, Int32Property p2);
    friend bool operator<(Int32Property p1, Int32Property p2);
    friend bool operator>(Int32Property p1, Int32Property p2);
    friend bool operator<=(Int32Property p1, Int32Property p2);
    friend bool operator>=(Int32Property p1, Int32Property p2);
    friend Int32Property operator+(Int32Property p1, Int32Property p2);
    friend Int32Property operator+(Int32Property p1, int32_t p2);
    friend Int32Property operator+(int32_t p1, Int32Property p2);

    Int32Property operator=(Int32Property other);
    Int32Property operator=(int32_t other);
    Int32Property operator+=(Int32Property other);
    Int32Property operator+=(int32_t other);
    Int32Property operator-=(Int32Property other);
    Int32Property operator-=(int32_t other);
    Int32Property operator*=(Int32Property other);
    Int32Property operator*=(int32_t other);
    Int32Property operator/=(Int32Property other);
    Int32Property operator/=(int32_t other);

private:
    int32_t data;
};  // class Int32Property

}  // namespace aruwlib

#endif  // INT32_PROPERTY_HPP_
