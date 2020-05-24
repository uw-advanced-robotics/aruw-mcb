#include "PropertyTable.hpp"
#include "aruwlib/algorithms/math_user_utils.hpp"

namespace aruwlib
{

namespace arch
{

PropertyTable PropertyTable::mainPropertySystem;

PropertyTable& PropertyTable::getMainPropertySystem() { return mainPropertySystem; }

bool PropertyTable::getProperty(std::string propertyName, Property* p)
{
    algorithms::fnvhash_t hash = algorithms::fnvHash(propertyName.c_str());

    if (propertyTable.count(hash) == 0)
    {
        *p = propertyTable[hash];
        return true;
    }
    return false;
}

}  // namespace arch

}  // namespace aruwlib
