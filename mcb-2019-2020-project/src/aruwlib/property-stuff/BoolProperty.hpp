#ifndef BOOL_PROPERTY_HPP_
#define BOOL_PROPERTY_HPP_

#include <string>

#include "BaseProperty.hpp"

namespace aruwlib
{

class BoolProperty : public BaseProperty
{
public:
    BoolProperty() : data(false) {}
    BoolProperty(bool data) : data(data) {}
    BoolProperty(const BoolProperty& other) : data(other.data) {}

    virtual ~BoolProperty() = default;

    uint8_t *serializeData(uint16_t *size) const override;
    uint8_t getPropertyType() const override { return BOOL_PROPERTY_TYPE; }
    std::string toString() const override { return std::string(data ? "true": "false"); }

    operator bool() { return data; }
    BoolProperty operator=(BoolProperty p2);
    BoolProperty operator=(bool p2);
    BoolProperty operator&=(BoolProperty p1);
    BoolProperty operator&=(bool p1);
    BoolProperty operator|=(BoolProperty p1);
    BoolProperty operator|=(bool p1);

private:
    bool data;
};  // class BoolProperty

}  // namespace aruwlib

#endif  // BOOL_PROPERTY_HPP_
