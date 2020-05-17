#include "PropertyTable.hpp"


namespace aruwlib
{

PropertyTable PropertyTable::mainPropertySystem;

PropertyTable& PropertyTable::getMainPropertySystem() { return mainPropertySystem; }

bool PropertyTable::getProperty(std::string propertyName, Property* p)
{
    fnvhash_t hash = fnvHash(propertyName.c_str());
    if (propertyTable.count(hash) == 0)
    {
        *p = propertyTable[hash];
        return true;
    }
    return false;
}

}  // namespace aruwlib
