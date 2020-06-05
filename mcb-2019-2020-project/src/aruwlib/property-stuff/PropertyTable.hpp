#ifndef PROPERTY_TABLE_HPP_
#define PROPERTY_TABLE_HPP_

#include <map>
#include <string>

#include "BaseProperty.hpp"

namespace aruwlib
{

class PropertyTable
{
public:
    static const int PROPERTY_TABLE_MAX_SIZE = 512;

    static PropertyTable& getMainPropertySystem();

    static void resetMainPropertyTable();

    PropertyTable(const PropertyTable&) = delete;

    PropertyTable& operator=(const PropertyTable&) = default;

    /**
     * Check if number of properties in PropertyTable reaches maximum
     * @return if PropertyTable is full
     */
    bool isFull() const { return propertyTable.size() == PROPERTY_TABLE_MAX_SIZE; }

    /**
     * Add data to the PropertyTable.
     * 
     * @paramt the type of the data to be stored.
     * @param data pointer to data to be managed.
     * @param property_name name of property.
     */
    bool addProperty(BaseProperty *data);

    template<typename T>
    bool removeProperty(const std::string &propertyName, T **removed)
    {
        if (propertyTable.count(propertyName) != 0)
        {
            *removed = dynamic_cast<T *>(propertyTable[propertyName]);
            propertyTable.erase(propertyName);
            return true;
        }
        return false;
    }

    template<typename T>
    T* getProperty(const std::string &propertyName)
    {
        if (propertyTable.count(propertyName) != 0)
        {
            T* property = dynamic_cast<T*>(propertyTable[propertyName]);
            return property;
        }
        return nullptr;
    }

private:
    static PropertyTable mainPropertySystem;

    std::map<std::string, BaseProperty*> propertyTable;

    PropertyTable() {}
};  // class PropertyTable

}  // namespace aruwlib

#endif  // PROPERTY_TABLE_HPP_
