// #ifndef PROPERTY_TABLE_HPP_
// #define PROPERTY_TABLE_HPP_

// #include <memory>
// #include <map>
// #include <string>
// #include <utility>

// #include <modm/architecture/driver/atomic/queue.hpp>
// #include <modm/container.hpp>

// #include "aruwlib/algorithms/math_user_utils.hpp"
// #include "aruwlib/communication/serial/dji_serial.hpp"
// #include "aruwlib/rm-dev-board-a/board.hpp"


// template<typename T>
// struct ArrayProperty
// {
//     ArrayProperty() = default;
//     ArrayProperty& operator=(const ArrayProperty&) = default;

// };

// template<typename T>
// struct Property
// {
//     Property() = default;
//     Property& operator=(const Property&) = default; 
//     Property(PropertyType t, uint8_t tS, uint32_t bC, std::string n, void *dP)
//         : type(t), name(n), dataPointer(dP) {}
//     ///< When the destructor is run, we aren't responsible for deleting dataPointer.
//     ~Property() = default;
//     PropertyType type;
//     std::string name;
//     void *dataPointer;
// };

// namespace aruwlib
// {

// class PropertyTable
// {
//  public:
//     typedef uint32_t property_id_t;

//     enum PropertyType : uint8_t
//     {
//         U8_PROPERTY = 1,
//         U16_PROPERTY = 2,
//         U32_PROPERTY = 3,
//         U64_PROPERTY = 4,
//         S8_PROPERTY = 4,
//         S16_PROPERTY = 5,
//         S32_PROPERTY = 6,
//         FLOAT_PROPERTY = 7,
//         BOOL_PROPERTY = 8
//     };

//     PropertyTable& operator=(const PropertyTable&) = delete;

//     /**
//      * Add a data to PropertyTable
//      * 
//      * @paramt the type of the data to be stored
//      * @param data pointer to data to be managed
//      * @param property_name name of property
//      * @return alias id of the property corresponding to given data
//      */
//     template<typename T>
//     bool addProperty(T *data, std::string propertyName)
//     {
//         return addProperty(data, propertyName, sizeof(T));
//     }

//     /**
//      * Add an array to PropertyTable
//      *
//      * @param array pointer to array to be managed
//      * @param length length of array
//      * @param property_name name of property
//      * @return alias id of the property corresponding to given data
//      */
//     template<typename T>
//     bool addProperty(T *data,
//                      uint16_t length,
//                      std::string propertyName)
//     {
//         return addProperty(data, propertyName, length * sizeof(T));
//     }

//     /**
//      * Check if number of properties in PropertyTable reaches maximum
//      * @return if PropertyTable is full
//      */
//     bool isFull() const { return propertyTable.size() == PROPERTY_TABLE_MAX_SIZE; }

//     template<typename T>
//     bool setProperty(std::string propertyName, T data)
//     {
//         algorithms::fnvhash_t hash = algorithms::fnvHash(propertyName.c_str());
//         if (propertyTable.count(hash) != 0)
//         {
//             propertyTable[hash].dataPointer = static_cast<void*>(data);
//             return true;
//         }
//         return false;
//     }

//     template<typename T>
//     bool setArrayPropertyIndex(std::string propertyName, T data, int index)
//     {
//         algorithms::fnvhash_t hash = algorithms::fnvHash(propertyName.c_str());
//         if (propertyTable.count(hash) != 0) {
//             int arrLen = propertyTable[hash].bcount / sizeof(T);
//             if (index < 0 || index >= arrLen)
//             {
//                 return false;
//             }
//             T *arr = static_cast<T*>(propertyTable[hash]);
//             arr[index] = data;
//             return true;
//         }
//         return false;
//     }

//     bool getProperty(std::string propertyName, Property* p);

//     static PropertyTable& getMainPropertySystem();

//  private:
//     static const int PROPERTY_TABLE_MAX_SIZE = 512;

//     static PropertyTable mainPropertySystem;

//     std::map<algorithms::fnvhash_t, Property*> propertyTable;

//     PropertyTable() {}

//     template<typename T>
//     bool addProperty(T *data, std::string propertyName, uint32_t bcount)
//     {
//         if (propertyTable.size() == PROPERTY_TABLE_MAX_SIZE ||
//             data == nullptr ||
//             propertyName.size() == 0)
//         {
//             return false;
//         }
//         Property p(typeToEnum(data), sizeof(T), bcount, propertyName, data);
//         algorithms::fnvhash_t propertyHash = algorithms::fnvHash(propertyName.c_str());
//         if (propertyTable.count(propertyHash) != 0)
//         {
//             return false;
//         }
//         propertyTable[propertyHash] = p;
//         return true;
//     }


// };  // class PropertyTable

// }  // namespace aruwlib

// #endif  // PROPERTY_TABLE_HPP_
