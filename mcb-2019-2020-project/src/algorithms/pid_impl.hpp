/*
 * Copyright (c) 2009-2010, Martin Rosekeit
 * Copyright (c) 2009-2011, Fabian Greif
 * Copyright (c) 2010, Georgi Grinshpun
 * Copyright (c) 2012, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef __PID_IMPL_HPP__
#define __PID_IMPL_HPP__

#include <utility>
#include <modm/platform/clock/rcc.hpp>

namespace aruwlib
{

namespace algorithms
{

template<typename T, unsigned int ScaleFactor>
Pid<T, ScaleFactor>::Parameter::Parameter(
        const float& kp, const float& ki, const float& kd,
        const T& maxErrorSum, const T& maxOutput) :
    kp(static_cast<T>(kp * ScaleFactor)),
    ki(static_cast<T>(ki * ScaleFactor)),
    kd(static_cast<T>(kd * ScaleFactor)),
    maxErrorSum(static_cast<T>(maxErrorSum * ScaleFactor)),
    maxOutput(maxOutput)
{
}

// -----------------------------------------------------------------------------
template<typename T, unsigned int ScaleFactor>
Pid<T, ScaleFactor>::Pid(
        const float& kp, const float& ki, const float& kd,
        const T& maxErrorSum, const T& maxOutput) :
    parameter(kp, ki, kd, maxErrorSum, maxOutput)
{
    this->reset();
}

// -----------------------------------------------------------------------------
template<typename T, unsigned int ScaleFactor>
Pid<T, ScaleFactor>::Pid(
        const Parameter& parameter) :
    parameter(parameter)
{
    this->reset();
}

template<typename T, unsigned int ScaleFactor>
void Pid<T, ScaleFactor>::reset()
{
    this->errorSum = 0;
    this->lastError = 0;
    this->output = 0;
    this->lastTime = DWT->CYCCNT;
}

template<typename T, unsigned int ScaleFactor>
void Pid<T, ScaleFactor>::setParameter(const Parameter& parameter)
{
    this->parameter = parameter;
}

template<typename T, unsigned int ScaleFactor>
void Pid<T, ScaleFactor>::update(const T& input, bool externalLimitation)
{
    bool limitation = externalLimitation;

    T tempErrorSum = errorSum + input;
    if (tempErrorSum > this->parameter.maxErrorSum) {
        tempErrorSum = this->parameter.maxErrorSum;
    }
    else if (tempErrorSum < -this->parameter.maxErrorSum)
    {
        tempErrorSum = -this->parameter.maxErrorSum;
    }

    WideType tmp = 0;

    // Main difference between modm::Pid and our Pid, we add a time element
    uint32_t currTime = DWT->CYCCNT;
    uint32_t dt = (
        currTime < this->lastTime ?
        0xffffffff - this->lastTime + currTime :
        currTime - this->lastTime) / static_cast<float>(modm::clock::fcpu_kHz);

    tmp += static_cast<WideType>(this->parameter.kp) * input;
    tmp += static_cast<WideType>(this->parameter.ki * dt) * (tempErrorSum);
    tmp += static_cast<WideType>(this->parameter.kd / dt) * (input - this->lastError);

    tmp = tmp / ScaleFactor;

    if (tmp > this->parameter.maxOutput) {
        this->output = this->parameter.maxOutput;
        limitation = true;
    }
    else if (tmp < -this->parameter.maxOutput)
    {
        this->output = -this->parameter.maxOutput;
        limitation = true;
    }
    else
    {
        this->output = tmp;
    }

    // If an external limitation (saturation somewhere in the control loop) is
    // applied the error sum will only be decremented, never incremented.
    // This is done to help the system to leave the saturated state.
    if (!limitation || (std::abs(tempErrorSum) < std::abs(this->errorSum)))
    {
        this->errorSum = tempErrorSum;
    }
    this->lastTime = currTime;
    this->lastError = input;
}
}  // namespace algorithms
}  // namespace aruwlib
#endif  // __PID_IMPL_HPP__
