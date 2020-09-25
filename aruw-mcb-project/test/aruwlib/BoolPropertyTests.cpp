#include <aruwlib/BoolProperty.hpp>
#include <gtest/gtest.h>

using aruwlib::BoolProperty;

TEST(BoolProperty, Default_constructor_constructs_false_bool)
{
    BoolProperty p;

    EXPECT_EQ(false, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ("false", p.toString());
}

TEST(BoolProperty, Single_arg_constructor_allows_for_bool_specification)
{
    BoolProperty p(true);

    EXPECT_EQ(true, p);
    EXPECT_EQ(nullptr, p.getPropertyName());
    EXPECT_EQ(p.toString(), "true");
}

TEST(BoolProperty, Two_arg_constructor_allows_for_bool_and_name_specification)
{
    BoolProperty p(true, "the property");

    EXPECT_EQ(true, p);
    EXPECT_EQ("the property", p.getPropertyName());
    EXPECT_EQ("true", p.toString());
}

TEST(BoolProperty, Copy_constructor_copies_property_data_and_name)
{
    BoolProperty p1(true, "the property");
    BoolProperty p2(p1);

    EXPECT_EQ(true, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("true", p2.toString());
}

TEST(BoolProperty, Equals_operator_copies_property_data_and_name)
{
    BoolProperty p1(true, "the property");
    BoolProperty p2;

    p2 = p1;
    EXPECT_EQ(true, p1);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("true", p2.toString());
    p2 = true;
    EXPECT_EQ(true, p2);
    EXPECT_EQ("the property", p2.getPropertyName());
    EXPECT_EQ("true", p2.toString());
}

TEST(BoolProperty, And_equals_operator_applied_correctly)
{
    BoolProperty p1(true);
    BoolProperty p2(true);
    BoolProperty p3(false);

    EXPECT_EQ(true, p1 &= p2);
    EXPECT_EQ(false, p1 &= p3);
    EXPECT_EQ(true, p2 &= true);
}

TEST(BoolProperty, Or_equals_operator_applied_correctly)
{
    BoolProperty p1(false);
    BoolProperty p2(false);
    BoolProperty p3(true);

    EXPECT_EQ(false, p1 |= p2);
    EXPECT_EQ(true, p1 |= p3);
    EXPECT_EQ(true, p1 |= false);
    EXPECT_EQ(false, p2 |= false);
}

TEST(BoolProperty, getSerializationArrSize_returns_sizeof_bool)
{
    BoolProperty p(true, "the property");

    EXPECT_EQ(sizeof(bool), p.getSerializationArrSize());
}

TEST(BoolProperty, serializeData)
{
    BoolProperty p(true, "the property");
    uint8_t *arr = new uint8_t[p.getSerializationArrSize()];

    p.serializeData(arr);
    EXPECT_EQ(0x01, arr[0]);
}

TEST(BoolProperty, setProperty_updates_data)
{
    BoolProperty p;

    p.setProperty(true);
    EXPECT_EQ(true, p);
    EXPECT_EQ("true", p.toString());
}
