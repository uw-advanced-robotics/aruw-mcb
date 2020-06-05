#ifndef FLOAT_PROPERTY_HPP_
#define FLOAT_PROPERTY_HPP_

#include "BaseProperty.hpp"

namespace aruwlib
{

class FloatProperty : public BaseProperty
{
public:
    FloatProperty() : data(0.0f) {}
    FloatProperty(float data) : data(data) {}
    FloatProperty(float data, std::string name) : BaseProperty(name), data(data) {}
    FloatProperty(const FloatProperty& other) = default;

    virtual ~FloatProperty() = default;

    uint8_t *serializeData(uint16_t *size) const override;
    uint8_t getPropertyType() const override { return FLOAT_PROPERTY_TYPE; }
    std::string toString() const override { return std::to_string(data); }
    bool setProperty(void *data) override;

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
