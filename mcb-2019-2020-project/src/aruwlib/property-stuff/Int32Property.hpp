#ifndef INT32_PROPERTY_HPP_
#define INT32_PROPERTY_HPP_

#include <cinttypes>

#include "BaseProperty.hpp"

namespace aruwlib
{

class Int32Property : public BaseProperty
{
public:
    Int32Property() : BaseProperty(), data(0) {}
    Int32Property(bool data) : BaseProperty(), data(data) {}
    Int32Property(bool data, std::string name) : BaseProperty(name), data(data) {}
    Int32Property(const Int32Property &other) = default;

    virtual ~Int32Property() = default;

    uint8_t *serializeData(uint16_t *size) const override;
    uint8_t getPropertyType() const override { return INT32_PROPERTY_TYPE; }
    std::string toString() const override { return std::to_string(data); }

    operator int32_t() { return data; }
    Int32Property& operator=(Int32Property& other) = default;
    Int32Property& operator=(int32_t other);
    Int32Property& operator+=(Int32Property& other);
    Int32Property& operator+=(int32_t other);
    Int32Property& operator-=(Int32Property& other);
    Int32Property& operator-=(int32_t other);
    Int32Property& operator*=(Int32Property& other);
    Int32Property& operator*=(int32_t other);
    Int32Property& operator/=(Int32Property& other);
    Int32Property& operator/=(int32_t other);

private:
    int32_t data;
};  // class Int32Property

}  // namespace aruwlib

#endif  // INT32_PROPERTY_HPP_
