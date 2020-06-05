#include "PropertyTable.hpp"


#include <iostream>

namespace aruwlib
{

PropertyTable PropertyTable::mainPropertySystem;

PropertyTable& PropertyTable::getMainPropertySystem() { return mainPropertySystem; }

void PropertyTable::resetMainPropertyTable()
{
    PropertyTable p;
    mainPropertySystem = p;
}


bool PropertyTable::addProperty(BaseProperty *data)
{
    if (propertyTable.size() >= PROPERTY_TABLE_MAX_SIZE)
    {
        return false;
    }
    if (!data->getPropertyNameValid()) {
        return false;
    }
    if (propertyTable.count(data->getPropertyName())) {
        return false;
    }
    propertyTable[data->getPropertyName()] = data;
    return true;
}

}  // namespace aruwlib
