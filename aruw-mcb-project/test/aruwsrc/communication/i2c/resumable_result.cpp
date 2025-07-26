#include <gtest/gtest.h>
#include <iostream>

#include "tap/architecture/clock.hpp"
#include "tap/architecture/periodic_timer.hpp"
#include "tap/architecture/timeout.hpp"

#include "modm/processing/resumable.hpp"

class MyClass : public modm::Resumable<10>
{
public:
    bool errorTimeoutWithTime(){
        bool istimedout = errorTimeout.execute();
        std::cout << "Error timeout with time called: " << clock.time << " Timeout: " << errorTimeout.remainingTime << std::endl;
        clock.time += 1;
        return istimedout;
    }

    modm::ResumableResult<bool> doSomething()
    {
        RF_BEGIN(1);

        if (!stepDone)
        {
            stepDone = true;
            RF_YIELD();  // simulate async step
        }

        RF_RETURN(true);
        RF_END_RETURN(false);
    }
    modm::ResumableResult<bool> falseFunction()
    {
        RF_BEGIN(2);
        // while(true){
        //     RF_YIELD();
        // }
        RF_WAIT_UNTIL(false || errorTimeoutWithTime());
        if (errorTimeout.remainingTime <= 2000)
        {
            std::cout << "Timeout occurred, returning false" << std::endl;
            erroredOut = true;
            RF_RETURN(false);
        }
        RF_END_RETURN(false);
        
    }
    modm::ResumableResult<bool> configureWriteReadWithTimeout()
    {
        bool success = false;
        RF_BEGIN(3);

        RF_WAIT_UNTIL(RF_CALL(falseFunction()) || errorTimeoutWithTime() || erroredOut);
        if (errorTimeout.remainingTime <= 2000 || erroredOut){
            std::cout << "Timeout occurred, returning false" << std::endl;
            erroredOut = true;
            RF_RETURN(false);
        }
        
        erroredOut = false;
        errorTimeout.restart(errorTimeoutTime);
        RF_END_RETURN(true);
    }

    void wtfistimer()
    {
        errorTimeout.restart(errorTimeoutTime);  // Start timeout
        while (!errorTimeout.execute())
        {
            // Simulate some work
            clock.time += 1;
            
        }
    }

    tap::arch::clock::ClockStub clock;
    uint32_t timeout = 1200;
    uint32_t errorTimeoutTime = timeout * 400;

    tap::arch::PeriodicMicroTimer errorTimeout;
    bool erroredOut = false;
    bool debug = false;

private:
    bool stepDone = false;
};

TEST(ResumableTest, RunsToCompletion)
{
    MyClass obj;
    modm::ResumableResult<bool> result(0);  // default to start state

    // Loop until it finishes
    while ((result = obj.doSomething()).getState() == modm::rf::Running)
    {
        // Could simulate other system ticks here
    }

    EXPECT_EQ(result.getState(), modm::rf::Stop);
    EXPECT_TRUE(result.getResult());
}

TEST(ResumableTest, ConfigureWriteReadWithTimeout)
{
    MyClass obj;

    // Start the resumable
    obj.errorTimeout.restart(obj.errorTimeoutTime);
    modm::ResumableResult<bool> result = obj.configureWriteReadWithTimeout();

    // Loop until it finishes
    while (result.getState() == modm::rf::Running)
    {
        // Simulate system ticks
        result = obj.configureWriteReadWithTimeout();
    }

    EXPECT_EQ(result.getState(), modm::rf::Stop);
    EXPECT_EQ(obj.errorTimeout.remainingTime - 1000, 0);
    EXPECT_FALSE(result.getResult());
    EXPECT_TRUE(obj.erroredOut);
}

TEST(ResumableTest, TimeoutFunctionality)
{
    MyClass obj;

    // Start the timeout
    obj.wtfistimer();

    // Check if the timeout was executed
    EXPECT_EQ(obj.errorTimeout.remainingTime - 1000, 0);
}
