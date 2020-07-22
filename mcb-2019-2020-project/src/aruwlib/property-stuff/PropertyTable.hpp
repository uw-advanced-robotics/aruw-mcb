#ifndef PROPERTY_TABLE_HPP_
#define PROPERTY_TABLE_HPP_

#include <map>
#include <string>

#include "BaseProperty.hpp"

namespace aruwlib
{
/**
 * \todo
 */
class PropertyTable
{
public:
    ///< The max size of the table.
    static const int PROPERTY_TABLE_MAX_SIZE = 512;

    ///<
    PropertyTable() = default;

    ///<
    PropertyTable(const PropertyTable &) = default;

    ///<
    PropertyTable &operator=(const PropertyTable &) = default;

    /**
     * Check if number of properties in PropertyTable reaches maximum.
     *
     * @return `true` if the PropertyTable is full, `false` otherwise.
     */
    bool isFull() const { return propertyTable.size() == PROPERTY_TABLE_MAX_SIZE; }

    ///< @return The number of elements in the table.
    int getSize() const { return propertyTable.size(); }

    /**
     * Adds `data` to the PropertyTable.
     *
     * @param[in] data Pointer to data to be managed.
     */
    bool addProperty(BaseProperty *data);

    /**
     * Removes the property from the table by name. The removed property is returned.
     *
     * @param[in] propertyName The name of the property to be removed.
     */
    BaseProperty *removeProperty(const std::string &propertyName);

    /**
     * Returns a const pointer to the property associated with the propertyName.
     *
     * @param[in] propertyName The name of the property to get.
     * @return The property associated with propertyName. `nullptr` if the property
     *      is not in the table.
     */
    const BaseProperty *getProperty(const std::string &propertyName);

    /**
     * Sets the data of the property associated to the `propertyName` in the table to the
     * passed in data.
     *
     * @note a `reinterpret_cast` is used when setting the property's data. Be sure the
     *      data type is correct.
     * @tparam The data type of the property to be set.
     * @param[in] propertyName The name of the property to be set.
     * @param[in] data The data that the property will be set to.
     * @return `true` if the property was set successfully, `false` otherwise.
     */
    template <typename T> bool setProperty(const std::string &propertyName, T data)
    {
        if (propertyTable.count(propertyName) != 0)
        {
            BaseProperty *property = propertyTable[propertyName];
            if (property == nullptr)
            {
                return false;
            }
            return property->setProperty(&data);
        }
        return false;
    }

private:
    std::map<std::string, BaseProperty *> propertyTable;
};  // class PropertyTable

}  // namespace aruwlib

#endif  // PROPERTY_TABLE_HPP_
