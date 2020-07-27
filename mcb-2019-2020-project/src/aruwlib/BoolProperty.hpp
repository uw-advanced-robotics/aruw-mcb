#ifndef BOOL_PROPERTY_HPP_
#define BOOL_PROPERTY_HPP_

#include <string>

#include "BaseProperty.hpp"

namespace aruwlib
{
class BoolProperty : public BaseProperty
{
public:
    BoolProperty() : BaseProperty(), data(false) {}
    BoolProperty(bool data) : BaseProperty(), data(data) {}
    BoolProperty(bool data, const char* name) : BaseProperty(name), data(data) {}
    BoolProperty(const BoolProperty& other) = default;

    virtual ~BoolProperty() = default;

    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return sizeof(bool); }
    PROPERTY_TYPE_ID getPropertyType() const override { return PROPERTY_TYPE_ID::BOOL; }
    const char* toString() const override { return data ? "true" : "false"; }
    bool setProperty(void* data) override;

    operator bool() const { return data; }
    BoolProperty& operator=(BoolProperty& p2) = default;
    BoolProperty& operator=(bool p2);
    BoolProperty& operator&=(BoolProperty& p1);
    BoolProperty& operator&=(bool p1);
    BoolProperty& operator|=(BoolProperty& p1);
    BoolProperty& operator|=(bool p1);

private:
    bool data;
};  // class BoolProperty

}  // namespace aruwlib

#endif  // BOOL_PROPERTY_HPP_
