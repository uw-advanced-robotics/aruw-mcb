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

#ifndef BASE_PROPERTY_HPP_
#define BASE_PROPERTY_HPP_

#include <cstring>

#include "IBaseProperty.hpp"

namespace aruwlib
{
template <typename T>
class BaseProperty : public IBaseProperty
{
public:
    BaseProperty() : IBaseProperty() {}

    BaseProperty(const char *name) : IBaseProperty(name) {}

    virtual ~BaseProperty() = default;

    /**
     * Sets the data stored in the property to the passed in `data`.
     *
     * @param[in] data A pointer to the data to be set.
     * @return `true` if the property has been set properly, `false` otherwise.
     */
    virtual void setProperty(T data) = 0;
};  // class BaseProperty

}  // namespace aruwlib

#endif  // BASE_PROPERTY_HPP_
