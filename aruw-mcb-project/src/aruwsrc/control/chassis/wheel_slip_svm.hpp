/*
 * Copyright (c) 2023-2024 Advanced Robotics at the University of Washington <robomstr@uw.edu>
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

#ifndef WHEEL_SLIP_SVM_HPP_
#define WHEEL_SLIP_SVM_HPP_

#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace aruwsrc::control::chassis
{
struct ModelConfig
{
    double C = 1.0;
    double gamma = 0.5;
    int degree = 2;

    std::array<double, 4> mean{};
    std::array<double, 4> stdDev{};

    double bias = 0.0;

    std::vector<std::array<double, 4>> svX;
    std::vector<double> svAlpha;

    std::size_t numSupportVectors() const { return svX.size(); }

    void print(const char* varName = "MY_MODEL") const;
};

class WheelSlipSVM
{
public:
    using Sample = std::array<double, 4>;

    enum Param : int
    {
        TRANS_VEL = 0,
        ROT_VEL = 1,
        TRANS_ACC = 2,
        ROT_ACC = 3
    };

    struct TrainStats
    {
        int numSamples;
        int numSupportVectors;
        int numSlip;
        double trainAccuracy;
    };

    WheelSlipSVM() = default;
    explicit WheelSlipSVM(const ModelConfig& cfg) { loadConfig(cfg); }

    void setKernel(double C = 1.0, double gamma = 0.5, int degree = 2);

    TrainStats train(const std::vector<Sample>& X, const std::vector<int>& yRaw);

    void loadConfig(const ModelConfig& cfg);
    const ModelConfig& getConfig() const { return config_; }

    bool predict(const Sample& x) const;
    double decisionFunction(const Sample& x) const;

    double findMaxSafeParam(int k, const Sample& knownVals) const;

    bool isTrained() const { return trained; }
    int numSupportVectors() const { return static_cast<int>(config_.svX.size()); }

private:
    ModelConfig config_;
    bool trained = false;

    double kernelEval(const Sample& a, const Sample& b) const;
    Sample scaleVec(const Sample& x) const;
    void fitScaler(const std::vector<Sample>& X);
    void runSMO(const std::vector<Sample>& Xs, const std::vector<int>& y, int maxPasses = 5);
};

class DataCollector
{
public:
    using Sample = WheelSlipSVM::Sample;

    struct Result
    {
        WheelSlipSVM::TrainStats stats;
        ModelConfig config;
    };

    DataCollector() = default;

    void setKernel(double C, double gamma, int degree);
    void addSample(const Sample& x, bool slip);

    std::size_t sampleCount() const { return X_.size(); }

    Result finalize();

private:
    std::vector<Sample> X_;
    std::vector<int> y_;
    double C_ = 1.0;
    double gamma_ = 0.5;
    int degree_ = 2;
    bool finalized_ = false;
};
}  // namespace aruwsrc::control::chassis
#endif  // WHEEL_SLIP_SVM_HPP_