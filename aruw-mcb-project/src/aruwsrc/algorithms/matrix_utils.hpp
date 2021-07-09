#ifndef MATRIX_UTILS_HPP_
#define MATRIX_UTILS_HPP_

#include <cinttypes>

#include "modm/architecture/utils.hpp"

/**
 * Wraps an `arm_mat_instance_f32` and its associated array.
 * This is done to make it clear which arrays are associated
 * with which `arm_mat_instance_f32` instances.
 */
template <uint16_t ROWS, uint16_t COLS>
struct CMSISMat
{
    arm_matrix_instance_f32 matrix;
    float data[ROWS * COLS];

    CMSISMat() { arm_mat_init_f32(&matrix, ROWS, COLS, data); }

    CMSISMat(const float (&initialData)[ROWS * COLS])
    {
        memcpy(&data, &initialData, MODM_ARRAY_SIZE(initialData));
        arm_mat_init_f32(&matrix, ROWS, COLS, data);
    }
};

#endif  // MATRIX_UTILS_HPP_
