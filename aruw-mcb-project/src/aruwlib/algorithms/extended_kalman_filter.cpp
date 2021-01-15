#include "extended_kalman_filter.hpp"

#include <iostream>

template <uint8_t N, uint8_t M>
void print(const char *name, const modm::Matrix<float, N, M> &mat)
{
    std::cout << name << ":" << std::endl;
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            std::cout << mat[i][j] << "\t";
        }
        std::cout << std::endl;
    }
}
namespace aruwlib
{
namespace algorithms
{
// template class ExtendedKalmanFilter<6, 5>;

void ExtendedKalmanFilter::filterData(const MeasurementVector &z)
{
    x = fFunc(x);
    // print("x", x);

    auto jF = jFFunc(x);
    // print("jF", jF);
    p = jF * p * jF.asTransposed() + q;
    // print("p", p);
    // print("h(x)", hFunc(x));
    x = x + k * (z - hFunc(x));
    // print("k", k);
    // print("x", x);
    modm::Matrix<float, M, N> jH = jHFunc(x);
    modm::Matrix<float, N, M> jHTrans = jH.asTransposed();
    SquareMeasurementMatrix innovationCovariance = (jH * p) * jHTrans + r;
    SquareMeasurementMatrix innovationCovarianceInverse;
    inverse(innovationCovariance, &innovationCovarianceInverse);
    k = (p * jHTrans) * innovationCovarianceInverse;
    p = (i - k * jH) * p;
    print("x", x);
}
}  // namespace algorithms
}  // namespace aruwlib
