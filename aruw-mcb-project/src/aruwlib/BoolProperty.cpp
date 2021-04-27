/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#include "BoolProperty.hpp"

namespace aruwlib
{
void BoolProperty::serializeData(uint8_t* arr) const
{
    if (arr == nullptr)
    {
        return;
    }
    arr[0] = data;
}

void BoolProperty::setProperty(bool data) { this->data = data; }

BoolProperty& BoolProperty::operator=(bool p2)
{
    data = p2;
    return *this;
}

BoolProperty& BoolProperty::operator&=(BoolProperty& p1)
{
    data &= p1.data;
    return *this;
}

BoolProperty& BoolProperty::operator&=(bool p1)
{
    data &= p1;
    return *this;
}

BoolProperty& BoolProperty::operator|=(BoolProperty& p1)
{
    data |= p1.data;
    return *this;
}

BoolProperty& BoolProperty::operator|=(bool p1)
{
    data |= p1;
    return *this;
}

};  // namespace aruwlib
