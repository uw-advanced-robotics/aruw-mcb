#ifndef __PROPERTY_HPP__
#define __PROPERTY_HPP__

#include <memory>
#include <utility>
#include <string>
#include <modm/container.hpp>
#include <modm/architecture/driver/atomic/queue.hpp>
#include "aruwlib/rm-dev-board-a/board.hpp"
#include "aruwlib/communication/serial/dji_serial.hpp"
#include <map>
#include "aruwlib/algorithms/math_user_utils.hpp"

/**
 * Property Serial Protocol
 * Long Package
 * {
 * [PackageType(1 Byte)]
 * [Initial Sequence Number(1 Byte)]
 * [Expected Package Count(1 Byte)]
 * [Data]
 * 
 * }
 * 
 * Short Package
 * {
 * [Command(1 Byte)]
 * [Data]
 * }
 * 
 */
namespace aruwlib {

// todo(matthew) move this to algorithms? 
static const uint32_t FNV_PRIME = 16777619u;
static const uint32_t OFFSET_BASIS = 2166136261u;

typedef uint32_t fnvhash_t;

fnvhash_t fnvHash(const char* str)
{
    const size_t length = strlen(str) + 1;
    uint32_t hash = OFFSET_BASIS;
    for (size_t i = 0; i < length; ++i)
    {
        hash ^= *str++;
        hash *= FNV_PRIME;
    }
    return hash;
}

class PropertyTable
{
 public:
    typedef uint32_t property_id_t;

    enum PropertyType : uint8_t
    {
        U8_PROPERTY = 1,
        U16_PROPERTY = 2,
        U32_PROPERTY = 3,
        U64_PROPERTY = 4,
        S8_PROPERTY = 4,
        S16_PROPERTY = 5,
        S32_PROPERTY = 6,
        FLOAT_PROPERTY = 7,
        BOOL_PROPERTY = 8
    };

    struct Property
    {
        Property() = default;
        Property& operator=(const Property&) = default; 
        Property(PropertyType t, uint8_t tS, uint32_t bC, std::string n, void *dP)
            : type(t), typeSize(tS), bcount(bC), name(n), dataPointer(dP) {}
        ///< When the destructor is run, we aren't responsible for deleting dataPointer.
        ~Property() = default;
        PropertyType type;
        uint8_t typeSize;
        uint32_t bcount;
        std::string name;
        void *dataPointer;
    };

    PropertyTable& operator=(const PropertyTable&) = delete;

    /**
     * Add a data to PropertyTable
     * 
     * @paramt the type of the data to be stored
     * @param data pointer to data to be managed
     * @param property_name name of property
     * @return alias id of the property corresponding to given data
     */
    template<typename T>
    bool addProperty(T *data, std::string propertyName)
    {
        return addProperty(data, propertyName, sizeof(T));
    }

    /**
     * Add an array to PropertyTable
     *
     * @param array pointer to array to be managed
     * @param length length of array
     * @param property_name name of property
     * @return alias id of the property corresponding to given data
     */
    template<typename T>
    bool addProperty(T *data,
                     uint16_t length,
                     std::string propertyName)
    {
        return addProperty(data, propertyName, length * sizeof(T));
    }

    /**
     * Check if number of properties in PropertyTable reaches maximum
     * @return if PropertyTable is full
     */
    bool isFull() { return propertyTable.size() == PROPERTY_TABLE_MAX_SIZE; }

    template<typename T>
    bool setProperty(std::string propertyName, T data)
    {
        fnvhash_t hash = fnvHash(propertyName.c_str());
        if (propertyTable.count(hash) != 0)
        {
            propertyTable[hash].dataPointer = static_cast<void*>(data);
            return true;
        }
        return false;
    }

    template<typename T>
    bool setArrayPropertyIndex(std::string propertyName, T data, int index)
    {
        fnvhash_t hash = fnvHash(propertyName.c_str());
        if (propertyTable.count(hash) != 0) {
            int arrLen = propertyTable[hash].bcount / sizeof(T);
            if (index < 0 || index >= arrLen)
            {
                return false;
            }
            T *arr = static_cast<T*>(propertyTable[hash]);
            arr[index] = data;
            return true;
        }
        return false;
    }

    bool getProperty(std::string propertyName, Property* p);

    static PropertyTable& getMainPropertySystem();

 private:
    static const int PROPERTY_TABLE_MAX_SIZE = 512;

    static PropertyTable mainPropertySystem;

    std::map<fnvhash_t, Property> propertyTable;

    PropertyTable() {}

    template<typename T>
    bool addProperty(T *data, std::string propertyName, uint32_t bcount)
    {
        if (propertyTable.size() == PROPERTY_TABLE_MAX_SIZE ||
            data == nullptr ||
            propertyName.size() == 0)
        {
            return false;
        }
        Property p(typeToEnum(data), sizeof(T), bcount, propertyName, data);
        fnvhash_t propertyHash = fnvHash(propertyName.c_str());
        if (propertyTable.count(propertyHash) != 0)
        {
            return false;
        }
        propertyTable[propertyHash] = p;
        return true;
    }

    template<class T>
    PropertyType typeToEnum(T* type)
    {
        if (std::is_same<T, uint8_t>::value)
        {
            return U8_PROPERTY;
        }
        if (std::is_same<T, uint16_t>::value)
        {
            return U16_PROPERTY;
        }
        if (std::is_same<T, uint32_t>::value)
        {
            return U32_PROPERTY;
        }
        if (std::is_same<T, int8_t>::value)
        {
            return S8_PROPERTY;
        }
        if (std::is_same<T, int16_t>::value)
        {
            return S16_PROPERTY;
        }
        if (std::is_same<T, int32_t>::value)
        {
            return S32_PROPERTY;
        }
        if (std::is_same<T, float>::value)
        {
            return FLOAT_PROPERTY;
        }
        if (std::is_same<T, bool>::value)
        {
            return BOOL_PROPERTY;
        }
    }
};

}  // namespace aruwlib

#endif
