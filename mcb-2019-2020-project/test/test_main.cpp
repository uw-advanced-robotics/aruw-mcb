#include <iostream>

#include <aruwlib/Drivers.hpp>
#include <gmock/gmock.h>

class Environment : public testing::Environment
{
public:
    ~Environment() override {}

    void SetUp() override { aruwlib::Drivers::reset(); }

    void TearDown() override { aruwlib::Drivers::reset(); }
};  // class Environment

int main(int argc, char **argv)
{
    aruwlib::Drivers::initialize();
    testing::AddGlobalTestEnvironment(new Environment);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
