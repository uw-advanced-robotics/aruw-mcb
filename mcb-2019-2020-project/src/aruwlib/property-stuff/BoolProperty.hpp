#ifndef BOOL_PROPERTY_HPP_
#define BOOL_PROPERTY_HPP_

#include "PrimativeProperty.hpp"

namespace aruwlib
{

class BoolProperty : public PrimativeProperty
{
public:
    BoolProperty() : data(false) {}
    BoolProperty(bool data) : data(data) {}
    BoolProperty(const BoolProperty& other) : data(other.data) {}

    const uint8_t *getData() const override { return reinterpret_cast<const uint8_t *>(&data); }
    uint8_t getDataLength() const override { return sizeof(bool); }
    uint8_t getPrimativePropertyType() const override { return FLOAT_PROPERTY_TYPE; }
    bool get() { return data; }

    friend bool operator==(BoolProperty p1, BoolProperty p2);
    friend bool operator==(bool p1, BoolProperty p2);
    friend bool operator==(BoolProperty p1, bool p2);
    friend bool operator!=(BoolProperty p1, BoolProperty p2);
    friend bool operator!=(BoolProperty p1, bool p2);
    friend bool operator!=(bool p1, BoolProperty p2);
    friend bool operator||(BoolProperty p1, BoolProperty p2);
    friend bool operator||(BoolProperty p1, bool p2);
    friend bool operator||(bool p1, BoolProperty p2);
    friend bool operator&&(BoolProperty p1, BoolProperty p2);
    friend bool operator&&(BoolProperty p1, bool p2);
    friend bool operator&&(bool p1, BoolProperty p2);

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
