#ifndef I_BASE_PROPERTY_HPP_
#define I_BASE_PROPERTY_HPP_

#include <cinttypes>
#include <string>

namespace aruwlib
{
/**
 * An IBaseProperty is a generic typed wrapper for an actual type or data structure.
 * In particular, a BaseProperty is used in the `PropertyTable` as a way to store
 * a bunch of generic typed values together.
 *
 * The IBaseProperty stores the name of the property.
 * This name will be refered to during insertion into the PropertyTable. The
 * property can then be referenced by name in the table.
 *
 * The IBaseProperty has utilities for serializing a property of generic type. The
 * following serialization protocol is followed:
 *
 * \rst
 * +-------------------+----------------------------------------------+
 * | Byte              | Description                                  |
 * +===================+==============================================+
 * | 0                 | The property id, for identifying the type    |
 * |                   | of property.                                 |
 * +-------------------+----------------------------------------------+
 * | 1 - 2             | Length of the property, in bytes MSB byte 1, |
 * |                   | LSB byte 2.                                  |
 * +-------------------+----------------------------------------------+
 * | 3 - 3 + len(name) | The name of the property                     |
 * +-------------------+----------------------------------------------+
 * | 4 + len(name) +   | The serializaed property.                    |
 * +-------------------+----------------------------------------------+
 * \endrst
 */
class IBaseProperty
{
public:
    /**
     * Using the default constructor, an invalid propertyName is given.
     */
    IBaseProperty() : propertyName(nullptr) {}

    IBaseProperty(const char *name) : propertyName(name) {}

    virtual ~IBaseProperty() = default;

    /**
     * The header contains the the PROPERTY_TYPE_ID and the length (2 bytes).
     */
    static const uint8_t BASE_PROPERTY_HEADER_LENGTH = 3;

    /**
     * The type ID is used to identify the property so one can parse the property correctly.
     */
    enum class PROPERTY_TYPE_ID : uint8_t
    {
        INT32,
        FLOAT,
        BOOL
    };

    /**
     * @return The type of property, one of `PROPERTY_TYPE_ID`.
     */
    virtual PROPERTY_TYPE_ID getPropertyType() const = 0;

    /**
     * @return A string representation of the property.
     */
    virtual std::string toString() const = 0;

    /**
     * @return The name of the property.
     */
    const char *getPropertyName() const { return propertyName; }

    /**
     * @return `true` if `propertyName` is not `nullptr`, `false` otherwise
     */
    bool getPropertyNameValid() const { return propertyName != nullptr; }

    /**
     * Serializes the property (see IBaseProperty's main definition), storing the
     * serialized info in the passed in `arr`.
     *
     * @param[out] arr Pointer to where the data will be serialized.
     */
    void fullSerialization(uint8_t *arr) const;

    /**
     * @return The length (in bytes) that the serialized property takes up.
     *      Use to insure the array passed in in the `fullSerialization` function
     *      has enough space to store the serializaed data.
     */
    uint16_t getFullSerializationSize() const;

protected:
    /**
     * Puts a serialized representation of the data in arr
     *
     * @param[out] arr The location for the serialized data to go.
     *      It is assumed that the array has size to store the data.
     */
    virtual void serializeData(uint8_t *arr) const = 0;

    /**
     * @return the size of the array necessary for storing the serialied
     *      data.
     */
    virtual uint16_t getSerializationArrSize() const = 0;

private:
    const char *propertyName;
};  // class IBaseProperty

}  // namespace aruwlib

#endif  // I_BASE_PROPERTY_HPP_
