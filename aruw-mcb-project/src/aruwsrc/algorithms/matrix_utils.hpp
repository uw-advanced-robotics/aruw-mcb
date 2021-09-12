#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

#include <cinttypes>

#include "modm/architecture/utils.hpp"

#include "arm_math.h"

namespace aruwsrc::algorithms
{
/**
 * Wraps an `arm_mat_instance_f32` and its associated array.
 * This is done to make it clear which arrays are associated
 * with which `arm_mat_instance_f32` instances.
 */
template <uint16_t ROWS, uint16_t COLS>
struct CMSISMat
{
    float data[ROWS * COLS];
    arm_matrix_instance_f32 matrix;

    CMSISMat() : data(), matrix{ROWS, COLS, data} {}

    CMSISMat(const float (&initialData)[ROWS * COLS])
    {
        memcpy(data, initialData, sizeof(initialData));
        arm_mat_init_f32(&matrix, ROWS, COLS, data);
    }

    // Delete the copy constructor, create a move constructor. This will
    // Avoid us doing costly copys but will still allow move semantics.
    CMSISMat(CMSISMat &other) = delete;
    CMSISMat(CMSISMat &&other) = default;

    /**
     * Construct identity matrix in the current CMSISMat
     */
    bool constructIdentityMatrix()
    {
        if (ROWS != COLS)
        {
            return false;
        }

        for (int i = 0; i < ROWS; i++)
        {
            for (int j = 0; j < COLS; j++)
            {
                data[i * ROWS + j] = (i == j) ? 1 : 0;
            }
        }

        return true;
    }
};

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, B_COLS> operator+(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(
        A_ROWS == B_ROWS && A_COLS == B_COLS,
        "Invalid size of CMSISMat matricies in operator+");

    CMSISMat<A_ROWS, A_COLS> c;
    arm_mat_add_f32(&a.matrix, &b.matrix, &c.matrix);
    return c;
}

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, B_COLS> operator-(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(
        A_ROWS == B_ROWS && A_COLS == B_COLS,
        "Invalid size of CMSISMat matricies in operator-");

    CMSISMat<A_ROWS, A_COLS> c;
    arm_mat_sub_f32(&a.matrix, &b.matrix, &c.matrix);
    return c;
}

template <uint16_t A_ROWS, uint16_t A_COLS, uint16_t B_ROWS, uint16_t B_COLS>
inline CMSISMat<A_ROWS, B_COLS> operator*(
    const CMSISMat<A_ROWS, A_COLS> &a,
    const CMSISMat<B_ROWS, B_COLS> &b)
{
    static_assert(A_COLS == B_ROWS, "Invalid size of CMSISMat matricies in operator*");

    CMSISMat<A_ROWS, B_COLS> c;
    arm_mat_mult_f32(&a.matrix, &b.matrix, &c.matrix);
    return c;
}

}  // namespace aruwsrc::algorithms

#endif  // MATRIX_UTILS_HPP_
