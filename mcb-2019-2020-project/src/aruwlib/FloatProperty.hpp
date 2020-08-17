#ifndef FLOAT_PROPERTY_HPP_
#define FLOAT_PROPERTY_HPP_

#include <cinttypes>
#include <string>

#include "BaseProperty.hpp"

namespace aruwlib
{
class FloatProperty : public BaseProperty
{
public:
    FloatProperty() : data(0.0f) {}
    FloatProperty(float data) : data(data) {}
    FloatProperty(float data, const char* name) : BaseProperty(name), data(data) {}
    FloatProperty(const FloatProperty& other) = default;

    virtual ~FloatProperty() = default;

    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return sizeof(float); }
    PROPERTY_TYPE_ID getPropertyType() const override { return PROPERTY_TYPE_ID::FLOAT; }
    std::string toString() const override { return std::to_string(data).c_str(); }
    bool setProperty(void* data) override;

    operator float() const { return data; }
    FloatProperty& operator=(FloatProperty& other) = default;
    FloatProperty& operator=(float other);
    FloatProperty& operator+=(FloatProperty& other);
    FloatProperty& operator+=(float other);
    FloatProperty& operator-=(FloatProperty& other);
    FloatProperty& operator-=(float other);
    FloatProperty& operator*=(FloatProperty& other);
    FloatProperty& operator*=(float other);
    FloatProperty& operator/=(FloatProperty& other);
    FloatProperty& operator/=(float other);

private:
    float data;
};

}  // namespace aruwlib

#endif  // FLOAT_PROPERTY_HPP_
