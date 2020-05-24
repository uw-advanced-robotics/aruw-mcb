#ifndef BASE_PROPERTY_HPP_
#define BASE_PROPERTY_HPP_

#include <cstring>
#include <string>

class BaseProperty {
public:
    static const uint8_t BASE_PROPERTY_HEADER_LENGTH = 3;
    static const uint8_t PRIMATIVE_PROPERTY_TYPE = 0;

    virtual uint8_t *serializeData(uint16_t *size) const = 0;
    virtual uint8_t getPropertyType() const = 0;

    uint8_t *fullSerialization() const
    {
        uint16_t dataLength;
        uint8_t *data = serializeData(&dataLength);
        uint8_t *dataWithHeader = new uint8_t[BASE_PROPERTY_HEADER_LENGTH + dataLength];
        dataWithHeader[0] = getPropertyType();
        dataWithHeader[1] = (dataLength >> 8) & 0xff;
        dataWithHeader[2] = dataLength & 0xff;
        memcpy(dataWithHeader + BASE_PROPERTY_HEADER_LENGTH, data, dataLength);
        delete[] data;
        return dataWithHeader;
    }

private:
    uint16_t propertyId;
    std::string propertyName;
    void *propertyData;
};

#endif  // BASE_PROPERTY_HPP_
