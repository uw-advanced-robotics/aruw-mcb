#include "PropertyTable.hpp"


namespace aruwlib
{

PropertyTable PropertyTable::mainPropertySystem;

PropertyTable& PropertyTable::getMainPropertySystem() { return mainPropertySystem; }

bool PropertyTable::addProperty(BaseProperty *data)
{
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
