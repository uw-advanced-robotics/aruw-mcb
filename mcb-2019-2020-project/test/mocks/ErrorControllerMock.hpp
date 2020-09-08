#ifndef ERROR_CONTROLLER_MOCK_HPP_
#define ERROR_CONTROLLER_MOCK_HPP_

#include <aruwlib/errors/error_controller.hpp>
#include <gmock/gmock.h>

class ErrorControllerMock : public aruwlib::errors::ErrorController
{
public:
    MOCK_METHOD(void, addToErrorList, (const aruwlib::errors::SystemError& error), (override));
    MOCK_METHOD(void, update, (), (override));
};  // class ErrorControllerMock

#endif  // ERROR_CONTROLLER_MOCK_HPP_
