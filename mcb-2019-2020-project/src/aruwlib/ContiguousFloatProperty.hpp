#ifndef CONTIGUOUS_FLOAT_PROPERTY_HPP_
#define CONTIGUOUS_FLOAT_PROPERTY_HPP_

#include <string>

#include "algorithms/contiguous_float.hpp"

#include "BaseProperty.hpp"


namespace aruwlib
{
class ContiguousFloatProperty : public BaseProperty
{
public:
    ContiguousFloatProperty()
        : BaseProperty(),
          data(aruwlib::algorithms::ContiguousFloat(0.0, 0.0, 0.0))
    {
    }
    ContiguousFloatProperty(aruwlib::algorithms::ContiguousFloat data) : data(data) {}
    ContiguousFloatProperty(aruwlib::algorithms::ContiguousFloat data, const char* name)
        : BaseProperty(name),
          data(data)
    {
    }
    ContiguousFloatProperty(const ContiguousFloatProperty& other) = default;
    ContiguousFloatProperty(float val, float min, float max)
        : data(aruwlib::algorithms::ContiguousFloat(val, min, max))
    {
    }

    virtual ~ContiguousFloatProperty() = default;
    void serializeData(uint8_t* arr) const override;
    uint16_t getSerializationArrSize() const override { return 3 * sizeof(float); }
    PROPERTY_TYPE_ID getPropertyType() const override { return PROPERTY_TYPE_ID::CONTIGUOUS_FLOAT; }
    std::string toString() const override;
    bool setProperty(void* data) override;
    const aruwlib::algorithms::ContiguousFloat& getData() const { return data; };

    operator aruwlib::algorithms::ContiguousFloat() const { return data; }
    ContiguousFloatProperty& operator=(ContiguousFloatProperty& other) = default;
    ContiguousFloatProperty& operator=(aruwlib::algorithms::ContiguousFloat& other);
    ContiguousFloatProperty& operator+=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator+=(aruwlib::algorithms::ContiguousFloat& other);
    ContiguousFloatProperty& operator-=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator-=(aruwlib::algorithms::ContiguousFloat& other);
    ContiguousFloatProperty& operator*=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator*=(aruwlib::algorithms::ContiguousFloat& other);
    ContiguousFloatProperty& operator/=(ContiguousFloatProperty& other);
    ContiguousFloatProperty& operator/=(aruwlib::algorithms::ContiguousFloat& other);

private:
    aruwlib::algorithms::ContiguousFloat data;
};  // class ContiguousFloatProperty
}  // namespace aruwlib
#endif  // CONTIGUOUSFLOAT_PROPERTY_HPP