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
#include "wheel_slip_svm.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <stdexcept>
namespace aruwsrc::control::chassis
{
// TODO: change all error handling such that if it errors, this essentially just allows a pass
// through where the svm just passes through and doesn't affect output

// in theory, call this, and you can paste the output into the constants file :/
void ModelConfig::print(const char* varName) const
{
    std::printf("\ninline const ModelConfig %s = {\n", varName);
    std::printf("    .C      = %.17g,\n", C);
    std::printf("    .gamma  = %.17g,\n", gamma);
    std::printf("    .degree = %d,\n", degree);

    std::printf(
        "    .mean   = {%.17g, %.17g, %.17g, %.17g},\n",
        mean[0],
        mean[1],
        mean[2],
        mean[3]);
    std::printf(
        "    .stdDev = {%.17g, %.17g, %.17g, %.17g},\n",
        stdDev[0],
        stdDev[1],
        stdDev[2],
        stdDev[3]);

    std::printf("    .bias   = %.17g,\n", bias);

    std::printf("    .svX    = {\n");
    for (std::size_t i = 0; i < svX.size(); ++i)
        std::printf(
            "        {%.17g, %.17g, %.17g, %.17g}%s\n",
            svX[i][0],
            svX[i][1],
            svX[i][2],
            svX[i][3],
            (i + 1 < svX.size()) ? "," : "");
    std::printf("    },\n");

    std::printf("    .svAlpha = {\n        ");
    for (std::size_t i = 0; i < svAlpha.size(); ++i)
    {
        std::printf("%.17g", svAlpha[i]);
        if (i + 1 < svAlpha.size()) std::printf(", ");
        if ((i + 1) % 4 == 0 && i + 1 < svAlpha.size()) std::printf("\n        ");
    }
    std::printf("\n    },\n};\n\n");
}

void WheelSlipSVM::setKernel(double C, double gamma, int degree)
{
    config_.C = C;
    config_.gamma = gamma;
    config_.degree = degree;
}

double WheelSlipSVM::kernelEval(const Sample& a, const Sample& b) const
{
    double dot = 0.0;
    for (int i = 0; i < 4; ++i)
    {
        dot += a[i] * b[i];
    }
    return std::pow(config_.gamma * dot + 1.0, static_cast<double>(config_.degree));
}

void WheelSlipSVM::fitScaler(const std::vector<Sample>& X)
{
    config_.mean.fill(0.0);
    config_.stdDev.fill(0.0);
    const double n = static_cast<double>(X.size());

    for (const auto& x : X)
    {
        for (int j = 0; j < 4; ++j)
        {
            config_.mean[j] += x[j];
        }
    }

    for (int j = 0; j < 4; ++j)
    {
        config_.mean[j] /= n;
    }

    for (const auto& x : X)
        for (int j = 0; j < 4; ++j)
        {
            double d = x[j] - config_.mean[j];
            config_.stdDev[j] += d * d;
        }
    for (int j = 0; j < 4; ++j)
    {
        config_.stdDev[j] = std::sqrt(config_.stdDev[j] / n);
        if (config_.stdDev[j] < 1e-10) config_.stdDev[j] = 1.0;
    }
}

WheelSlipSVM::Sample WheelSlipSVM::scaleVec(const Sample& x) const
{
    Sample out;
    for (int j = 0; j < 4; ++j)
    {
        out[j] = (x[j] - config_.mean[j]) / config_.stdDev[j];
    }
    return out;
}

void WheelSlipSVM::runSMO(const std::vector<Sample>& Xs, const std::vector<int>& y, int maxPasses)
{
    const int n = static_cast<int>(Xs.size());
    std::vector<double> alpha(n, 0.0);
    double b = 0.0;

    std::vector<std::vector<double>> K(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i)
    {
        for (int j = i; j < n; ++j)
        {
            K[i][j] = K[j][i] = kernelEval(Xs[i], Xs[j]);
        }
    }

    auto decisionAt = [&](int i) {
        double s = -b;
        for (int k = 0; k < n; ++k)
        {
            s += alpha[k] * y[k] * K[i][k];
        }
        return s;
    };

    std::mt19937 rng(42);
    int passes = 0;

    while (passes < maxPasses)
    {
        int changed = 0;

        for (int i = 0; i < n; ++i)
        {
            double Ei = decisionAt(i) - y[i];

            bool violated =
                (y[i] * Ei < -1e-3 && alpha[i] < config_.C) || (y[i] * Ei > 1e-3 && alpha[i] > 0.0);
            if (!violated) continue;

            std::uniform_int_distribution<int> dist(0, n - 2);
            int j = dist(rng);
            if (j >= i) ++j;

            double Ej = decisionAt(j) - y[j];
            double aiOld = alpha[i];
            double ajOld = alpha[j];

            double L, H;
            if (y[i] != y[j])
            {
                L = std::max(0.0, ajOld - aiOld);
                H = std::min(config_.C, config_.C + ajOld - aiOld);
            }
            else
            {
                L = std::max(0.0, aiOld + ajOld - config_.C);
                H = std::min(config_.C, aiOld + ajOld);
            }
            if (L >= H) continue;

            double eta = 2.0 * K[i][j] - K[i][i] - K[j][j];
            if (eta >= 0.0) continue;

            double ajNew = std::min(H, std::max(L, ajOld - y[j] * (Ei - Ej) / eta));
            if (std::abs(ajNew - ajOld) < 1e-5) continue;

            double aiNew = aiOld + y[i] * y[j] * (ajOld - ajNew);
            alpha[i] = aiNew;
            alpha[j] = ajNew;

            double b1 =
                b + Ei + y[i] * (aiNew - aiOld) * K[i][i] + y[j] * (ajNew - ajOld) * K[i][j];
            double b2 =
                b + Ej + y[i] * (aiNew - aiOld) * K[i][j] + y[j] * (ajNew - ajOld) * K[j][j];

            if (aiNew > 0.0 && aiNew < config_.C)
                b = b1;
            else if (ajNew > 0.0 && ajNew < config_.C)
                b = b2;
            else
                b = (b1 + b2) / 2.0;

            ++changed;
        }

        passes = (changed == 0) ? passes + 1 : 0;
    }

    config_.svX.clear();
    config_.svAlpha.clear();
    for (int i = 0; i < n; ++i)
        if (alpha[i] > 1e-5)
        {
            config_.svX.push_back(Xs[i]);
            config_.svAlpha.push_back(alpha[i] * static_cast<double>(y[i]));
        }

    config_.bias = b;
}

WheelSlipSVM::TrainStats WheelSlipSVM::train(
    const std::vector<Sample>& X,
    const std::vector<int>& yRaw)
{
    if (X.empty() || X.size() != yRaw.size())
        throw std::runtime_error("x and y are supposed to be non-empty and same size");

    std::vector<int> y(yRaw.size());
    for (std::size_t i = 0; i < yRaw.size(); ++i)
    {
        y[i] = (yRaw[i] <= 0) ? -1 : 1;
    }

    fitScaler(X);

    std::vector<Sample> Xs;
    Xs.reserve(X.size());
    for (const auto& x : X)
    {
        Xs.push_back(scaleVec(x));
    }

    runSMO(Xs, y);
    trained = true;

    int correct = 0, nSlip = 0;
    for (std::size_t i = 0; i < Xs.size(); ++i)
    {
        double s = -config_.bias;
        for (std::size_t sv = 0; sv < config_.svX.size(); ++sv)
            s += config_.svAlpha[sv] * kernelEval(config_.svX[sv], Xs[i]);
        if (((s >= 0.0) ? 1 : -1) == y[i]) ++correct;
        if (y[i] == 1) ++nSlip;
    }

    return TrainStats{
        static_cast<int>(X.size()),
        static_cast<int>(config_.svX.size()),
        nSlip,
        static_cast<double>(correct) / static_cast<double>(X.size())};
}

void WheelSlipSVM::loadConfig(const ModelConfig& cfg)
{
    if (cfg.svX.size() != cfg.svAlpha.size())
        throw std::runtime_error("svX and svAlpha arent the same size");
    config_ = cfg;
    trained = true;
}

double WheelSlipSVM::decisionFunction(const Sample& x) const
{
    if (!trained) throw std::runtime_error("called decision function when not trained");
    Sample xs = scaleVec(x);
    double s = -config_.bias;
    for (std::size_t i = 0; i < config_.svX.size(); ++i)
        s += config_.svAlpha[i] * kernelEval(config_.svX[i], xs);
    return s;
}

bool WheelSlipSVM::predict(const Sample& x) const { return decisionFunction(x) >= 0.0; }

// so the way this works is k is the index of the parameter you wanna max out. knownVals
// contains the values that you already know/want to keep, and the value at index k is zero'd out.
// follows index conventions established earlier in the file: trans_vel is 0, rot_vel is 1,
// trans_accel is 2, rot_accel is 3
double WheelSlipSVM::findMaxSafeParam(int k, const Sample& knownVals) const
{
    if (!trained) throw std::runtime_error("called findmaxsafeparam when not trained");

    Sample xsKnown = scaleVec(knownVals);

    double A = 0.0, B = 0.0, C = -config_.bias;

    for (std::size_t i = 0; i < config_.svX.size(); ++i)
    {
        // Rᵢ partial dot product over known dimensions only??
        double Ri = 0.0;
        for (int j = 0; j < 4; ++j)
        {
            if (j != k) Ri += config_.svX[i][j] * xsKnown[j];
        }
        double xik = config_.svX[i][k];
        double gR1 = config_.gamma * Ri + 1.0;  // (γRᵢ + 1)

        A += config_.svAlpha[i] * (config_.gamma * config_.gamma) * (xik * xik);
        B += config_.svAlpha[i] * 2.0 * config_.gamma * xik * gR1;
        C += config_.svAlpha[i] * gR1 * gR1;
    }

    if (C < 0.0) return -1.0;  // slipping at t=0

    double tScaled;
    double disc = B * B - 4.0 * A * C;

    if (std::abs(A) < 1e-12)
    {
        if (std::abs(B) < 1e-12) return (C >= 0.0) ? std::numeric_limits<double>::infinity() : -1.0;
        tScaled = -C / B;
    }
    else if (disc < 0.0)
    {
        // we know it's safe for all t if parabola never crosses 0
        return std::numeric_limits<double>::infinity();
    }
    else
    {
        double sq = std::sqrt(disc);
        double t1 = (-B - sq) / (2.0 * A);  // smaller root
        double t2 = (-B + sq) / (2.0 * A);  // larger root

        // Take first positive root (first slip boundary as t increases)
        if (t1 > 1e-9)
            tScaled = t1;
        else if (t2 > 1e-9)
            tScaled = t2;
        else
            return -1.0;
    }

    // dont forget to unscale from normalized to irl units
    return tScaled * config_.stdDev[k] + config_.mean[k];
}

void DataCollector::setKernel(double C, double gamma, int degree)
{
    C_ = C;
    gamma_ = gamma;
    degree_ = degree;
}

void DataCollector::addSample(const Sample& x, bool slip)
{
    if (finalized_)
        throw std::runtime_error("data collector error; cannot add samples after finalize()");
    X_.push_back(x);
    y_.push_back(slip ? 1 : 0);
}

DataCollector::Result DataCollector::finalize()
{
    if (finalized_)
        throw std::runtime_error("data collector error; finalize() called multiple times");
    if (X_.empty()) throw std::runtime_error("data collector finalized without collecting samples");

    finalized_ = true;

    WheelSlipSVM svm;
    svm.setKernel(C_, gamma_, degree_);
    auto stats = svm.train(X_, y_);

    return Result{stats, svm.getConfig()};
}
}  // namespace aruwsrc::control::chassis