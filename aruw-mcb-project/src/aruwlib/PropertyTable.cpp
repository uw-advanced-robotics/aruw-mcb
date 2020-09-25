/*
 * Copyright (c) 2020 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruw-mcb.
 *
 * aruw-mcb is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruw-mcb is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruw-mcb.  If not, see <https://www.gnu.org/licenses/>.
 */

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
