#ifndef INT32_PROPERTY_HPP_
#define INT32_PROPERTY_HPP_

#include <cinttypes>
#include <string>

#include "BaseProperty.hpp"

namespace aruwlib
{
class Int32Property : public BaseProperty
{
public:
    Int32Property() : BaseProperty(), data(0) {}
    Int32Property(int32_t data) : BaseProperty(), data(data) {}
    Int32Property(int32_t data, const char* name) : BaseProperty(name), data(data) {}
    Int32Property(const Int32Property& other) = default;

    virtual ~Int32Property() = default;

    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return sizeof(int32_t); }
    PROPERTY_TYPE_ID getPropertyType() const override { return PROPERTY_TYPE_ID::INT32; }
    std::string toString() const override { return std::to_string(data).c_str(); }
    bool setProperty(void* data) override;

    operator int32_t() const { return data; }
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
