#ifndef BASE_PROPERTY_HPP_
#define BASE_PROPERTY_HPP_

#include <cinttypes>
#include <cstring>

namespace aruwlib
{
class BaseProperty
{
public:
    static const uint8_t BASE_PROPERTY_HEADER_LENGTH = 3;
    static const uint8_t INT32_PROPERTY_TYPE = 0;
    static const uint8_t FLOAT_PROPERTY_TYPE = 1;
    static const uint8_t BOOL_PROPERTY_TYPE = 2;

    BaseProperty() : propertyName("") {}
    BaseProperty(const char *name) : propertyName(name) {}
    virtual ~BaseProperty() = default;

    virtual void serializeData(uint8_t *arr) const = 0;
    virtual uint16_t getSerializationArrSize() const = 0;
    virtual uint8_t getPropertyType() const = 0;
    virtual const char *toString() const = 0;
    virtual bool setProperty(void *data) = 0;

    const char *getPropertyName() const { return propertyName; }
    bool getPropertyNameValid() const { return strlen(propertyName) > 0; }

    uint16_t getFullSerializationSize() const;
    void fullSerialization(uint8_t *arr) const;

private:
    const char *propertyName;
};  // class BaseProperty

}  // namespace aruwlib

#endif  // BASE_PROPERTY_HPP_
