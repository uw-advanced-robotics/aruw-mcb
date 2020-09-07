#include "PropertyTable.hpp"

#include <iostream>

namespace aruwlib
{
bool PropertyTable::addProperty(IBaseProperty *data)
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

IBaseProperty *PropertyTable::removeProperty(const std::string &propertyName)
{
    if (propertyTable.count(propertyName) != 0)
    {
        IBaseProperty *removed = propertyTable[propertyName];
        propertyTable.erase(propertyName);
        return removed;
    }
    return nullptr;
}

const IBaseProperty *PropertyTable::getProperty(const std::string &propertyName)
{
    if (propertyTable.count(propertyName) != 0)
    {
        return propertyTable[propertyName];
    }
    return nullptr;
}

std::map<std::string, IBaseProperty *>::const_iterator PropertyTable::getPropertyTableBeginning()
    const
{
    return propertyTable.begin();
}

std::map<std::string, IBaseProperty *>::const_iterator PropertyTable::getPropertyTableEnd() const
{
    return propertyTable.end();
}
}  // namespace aruwlib
