#ifndef CONTIGUOUS_FLOAT_PROPERTY_HPP_
#define CONTIGUOUS_FLOAT_PROPERTY_HPP_

#include <string>

#include "algorithms/contiguous_float.hpp"
#include "BaseProperty.hpp"
using namespace aruwlib::algorithms;

namespace aruwlib
{
class ContiguousFloatProperty : public BaseProperty
{
public:
    ContiguousFloatProperty() : BaseProperty(), data(ContiguousFloat(0.0, 0.0, 0.0)) {}
    ContiguousFloatProperty(ContiguousFloat data) : data(data) {}
    ContiguousFloatProperty(ContiguousFloat data, const char* name) : BaseProperty(name), data(data) {}
    ContiguousFloatProperty(const ContiguousFloatProperty& other) = default;

    virtual ~ContiguousFloatProperty() = default;
    void serializeData(uint8_t* size) const override;
    PROPERTY_TYPE_ID getPropertyType() const override { return PROPERTY_TYPE_ID::CONTIGUOUS_FLOAT; }
    const char* toString() const override { return std::to_string(data.getValue()).c_str(); }
    bool setProperty(void* data) override;

    operator ContiguousFloat() const { return data; }
    ContiguousFloatProperty& operator=(ContiguousFloatProperty& other) = default;
    ContiguousFloatProperty& operator=(ContiguousFloat& other);
    ContiguousFloatProperty& operator+=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator+=(ContiguousFloat& other);
    ContiguousFloatProperty& operator-=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator-=(ContiguousFloat& other);
    ContiguousFloatProperty& operator*=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator*=(ContiguousFloat& other);
    ContiguousFloatProperty& operator/=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator/=(ContiguousFloat& other);

private:
    ContiguousFloat data;
};  // class ContiguousFloatProperty 
}  // namespace aruwlib
#endif  // CONTIGUOUSFLOAT_PROPERTY_HPP