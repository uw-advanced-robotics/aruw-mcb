#include "PropertyTable.hpp"

#include <iostream>

namespace aruwlib
{
bool PropertyTable::addProperty(BaseProperty *data)
{
    if (propertyTable.size() >= PROPERTY_TABLE_MAX_SIZE)
    {
        return false;
    }
    if (!data->getPropertyNameValid())
    {
        return false;
    }
    if (propertyTable.count(data->getPropertyName()))
    {
        return false;
    }
    propertyTable[data->getPropertyName()] = data;
    return true;
}

BaseProperty *PropertyTable::removeProperty(const std::string &propertyName)
{
    if (propertyTable.count(propertyName) != 0)
    {
        BaseProperty *removed = propertyTable[propertyName];
        propertyTable.erase(propertyName);
        return removed;
    }
    return nullptr;
}

const BaseProperty *PropertyTable::getProperty(const std::string &propertyName)
{
    if (propertyTable.count(propertyName) != 0)
    {
        return propertyTable[propertyName];
    }
    return nullptr;
}

std::map<std::string, BaseProperty *>::const_iterator PropertyTable::getPropertyTableBeginning()
    const
{
    return propertyTable.begin();
}

}  // namespace aruwlib
