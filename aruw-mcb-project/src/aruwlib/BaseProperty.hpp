#ifndef BASE_PROPERTY_HPP_
#define BASE_PROPERTY_HPP_

#include <cstring>

#include "IBaseProperty.hpp"

namespace aruwlib
{
template <typename T> class BaseProperty : public IBaseProperty
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
