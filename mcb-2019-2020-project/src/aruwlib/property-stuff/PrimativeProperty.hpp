#ifndef PRIMATIVE_PROPERTY_
#define PRIMATIVE_PROPERTY_

#include <cinttypes>

#include "BaseProperty.hpp"

class PrimativeProperty : public BaseProperty
{
public:
    static const int PRIMATIVE_PROPERTY_HEADER_LENGTH = 1;
    static const uint8_t INT32_PROPERTY_TYPE = 0;
    static const uint8_t FLOAT_PROPERTY_TYPE = 1;
    static const uint8_t BOOL_PROPERTY_TYPE = 2;

    uint8_t getPropertyType() const override { return PRIMATIVE_PROPERTY_TYPE; }

    virtual const uint8_t *getData() const;
    virtual uint8_t getDataLength() const;
    virtual uint8_t getPrimativePropertyType() const;

    uint8_t *serializeData(uint16_t *size) const override {
        *size = PRIMATIVE_PROPERTY_HEADER_LENGTH + getDataLength();
        uint8_t *arr = new uint8_t[*size];
        arr[0] = getPrimativePropertyType();
        arr[1] = getDataLength();
        memcpy(arr + 2, getData(), getDataLength());
        return arr;
    }
};  // class PrimativeProperty

#endif  // PRIMATIVE_PROPERTY_
