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
    FloatProperty(const FloatProperty& other) : data(other.data) {}

    virtual ~FloatProperty() = default;

    uint8_t *serializeData(uint16_t *size) const override;
    uint8_t getPropertyType() const override { return FLOAT_PROPERTY_TYPE; }
    std::string toString() const override { return std::to_string(data); }

    operator float() { return data; }
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
