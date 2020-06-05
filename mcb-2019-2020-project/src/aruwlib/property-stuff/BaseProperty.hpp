#ifndef BASE_PROPERTY_HPP_
#define BASE_PROPERTY_HPP_

#include <cstring>
#include <string>

namespace aruwlib
{

class BaseProperty {
public:
    static const uint8_t BASE_PROPERTY_HEADER_LENGTH = 3;
    static const uint8_t INT32_PROPERTY_TYPE = 0;
    static const uint8_t FLOAT_PROPERTY_TYPE = 1;
    static const uint8_t BOOL_PROPERTY_TYPE = 2;

    BaseProperty() : propertyName("") {}
    BaseProperty(const std::string& name) : propertyName(name) {}
    virtual ~BaseProperty() = default;

    virtual uint8_t *serializeData(uint16_t *size) const = 0;
    virtual uint8_t getPropertyType() const = 0;
    virtual std::string toString() const = 0;
    virtual bool setProperty(void *data) = 0;

    const std::string &getPropertyName() const { return propertyName; }
    bool getPropertyNameValid() const { return propertyName.length() > 0; }

    uint8_t *fullSerialization() const;

private:
    std::string propertyName;
};  // class BaseProperty

}  // namespace aruwlib

#endif  // BASE_PROPERTY_HPP_
