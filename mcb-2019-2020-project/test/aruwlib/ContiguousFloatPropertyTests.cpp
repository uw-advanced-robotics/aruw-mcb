#include <aruwlib/ContiguousFloatProperty.hpp>

#include <CppUTest/CommandLineTestRunner.h>
#include <CppUTestExt/MockSupport.h>

using aruwlib::ContiguousFloatProperty;

TEST_GROUP(ContiguousFloatProperty)
{
};

TEST(ContiguousFloatProperty, test_default_constructor)
{
    ContiguousFloatProperty property;
    CHECK_FALSE(property.getPropertyNameValid());
    POINTERS_EQUAL(nullptr, property.getPropertyName());
    CHECK_EQUAL(12, property.getFullSerializationSize());
    STRCMP_EQUAL("CF - 0.0 [0.0, 0.0]", property.toString());  // todo this is a placeholder, what exactly do we want it to look like?
}

TEST(ContiguousFloatProperty, test_one_arg_constructor)
{
    ContiguousFloatProperty property(ContiguousFloat(0.0f, -180.0f, 180.0f));
    POINTERS_EQUAL(nullptr, property.getPropertyName());
    CHECK_FALSE(property.getPropertyNameValid());
    STRCMP_EQUAL("CF - 0.0 [180.0, -180.0]", property.toString());
    ContiguousFloat other(0.0f, -180.0f, 180.0f);
    CHECK_EQUAL(other.getValue(), property.getData().getValue());
    CHECK_EQUAL(other.getLowerBound(), property.getData().getLowerBound());
    CHECK_EQUAL(other.getUpperBound(), property.getData().getUpperBound());
}

TEST(ContiguousFloatProperty, test_two_arg_constructor)
{
    ContiguousFloatProperty property(ContiguousFloat(0.0f, -180.0f, 180.0f), "Cool property");
    CHECK(property.getPropertyNameValid());
    STRCMP_EQUAL("Cool property", property.getPropertyName());
    STRCMP_EQUAL("CF - 0.0 [180.0, -180.0]", property.toString());
    ContiguousFloat other(0.0f, -180.0f, 180.0f);
    CHECK_EQUAL(other.getValue(), property.getData().getValue());
    CHECK_EQUAL(other.getLowerBound(), property.getData().getLowerBound());
    CHECK_EQUAL(other.getUpperBound(), property.getData().getUpperBound());
}

TEST(ContiguousFloatProperty, test_copy_constructor)
{
    ContiguousFloatProperty property(ContiguousFloat(0.0f, -180.0f, 180.0f), "Cool property");
    ContiguousFloatProperty property_second(property);
    CHECK(property_second.getPropertyNameValid());
    STRCMP_EQUAL("Cool property", property_second.getPropertyName());
    STRCMP_EQUAL("CF - 0.0 [180.0, -180.0]", property_second.toString());
    CHECK_EQUAL(property.getData().getValue(), property_second.getData().getValue());
    CHECK_EQUAL(property.getData().getLowerBound(), property_second.getData().getLowerBound());
    CHECK_EQUAL(property.getData().getUpperBound(), property_second.getData().getUpperBound());
}

TEST(ContiguousFloatProperty, test_serialization)
{
    const float DOUBLES_EQUAL_THREASHOLD = 0.000001f;
    ContiguousFloatProperty property(ContiguousFloat(0.0f, -180.0f, 180.0f), "Cool property");
    uint8_t *data = new uint8_t(property.getSerializationArrSize());
    property.serializeData(nullptr);
    property.serializeData(data);
    float val, min, max;
    memcpy(&val, data, sizeof(float));
    memcpy(&min, data + sizeof(float), sizeof(float));
    memcpy(&max, data + 2 * sizeof(float), sizeof(float));
    DOUBLES_EQUAL(0.0f, val, DOUBLES_EQUAL_THREASHOLD);
    DOUBLES_EQUAL(-180.0f, min, DOUBLES_EQUAL_THREASHOLD);
    DOUBLES_EQUAL(180.0f, max, DOUBLES_EQUAL_THREASHOLD);
}

TEST(ContiguousFloatProperty, test_set_property)
{
    ContiguousFloatProperty property;
    ContiguousFloat f(0.0f, -180.0f, 180.0f);
    property.setProperty(nullptr);
    property.setProperty(reinterpret_cast<void *>(&f));
    CHECK_EQUAL(0.0f, property.getData().getValue());
    CHECK_EQUAL(-180.0f, property.getData().getLowerBound());
    CHECK_EQUAL(180.0f, property.getData().getUpperBound());
}
